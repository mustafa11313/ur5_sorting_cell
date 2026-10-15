// SortingCellSystem: a world plugin for Gazebo Fortress that runs the
// "production line" side of the sorting cell.
//
//  * Conveyor  - moves every free part lying on the belt at the commanded speed.
//  * Spawner   - spawns randomized parts (type, defect, pose, scale, colour)
//                at the upstream end of the belt.
//  * Grasp     - a grasp-assist: when the gripper closes on a part that sits
//                between the fingers, the part is rigidly attached to the TCP
//                until the gripper opens again. This replaces friction grasping,
//                which is notoriously unstable in simulation.
//  * Bookkeeping - publishes ground-truth part poses and events
//                ("spawned", "binned", "missed", "dropped") used for evaluation,
//                and removes parts that have been sorted or lost.
//
// Transport interface (Ignition topics, bridged to ROS 2 by ros_gz_bridge):
//   sub /conveyor/cmd_speed   ignition.msgs.Double   belt speed [m/s]
//   pub /conveyor/speed       ignition.msgs.Double
//   sub /gripper/attach       ignition.msgs.Boolean  true = close/attach
//   pub /gripper/attached     ignition.msgs.StringMsg name of held part or ""
//   sub /parts/spawn_enable   ignition.msgs.Boolean
//   sub /parts/clear          ignition.msgs.Boolean  remove all parts
//   pub /parts/ground_truth   ignition.msgs.Pose_V   world poses of all parts
//   pub /parts/events         ignition.msgs.StringMsg "<event> <part> [<bin>]"

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Root.hh>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace sorting_cell
{
namespace gz = ignition::gazebo;
using ignition::math::Pose3d;
using ignition::math::Vector3d;
using ignition::math::Quaterniond;

namespace
{
double secs(const std::chrono::steady_clock::duration & d)
{
  return std::chrono::duration<double>(d).count();
}

template<typename T>
T param(const std::shared_ptr<const sdf::Element> & sdf, const std::string & key, T def)
{
  if (!sdf || !sdf->HasElement(key)) {
    return def;
  }
  return sdf->Get<T>(key);
}

std::string colourStr(const Vector3d & c)
{
  std::ostringstream ss;
  ss << c.X() << " " << c.Y() << " " << c.Z() << " 1";
  return ss.str();
}

std::string material(const Vector3d & c)
{
  return "<material><ambient>" + colourStr(c * 0.6) + "</ambient><diffuse>" + colourStr(c) +
         "</diffuse><specular>0.2 0.2 0.2 1</specular></material>";
}
}  // namespace

struct PartType
{
  std::string name;
  Vector3d colour;
};

struct Bin
{
  std::string name;
  double cx, cy, sx, sy, zTop;
};

struct PartState
{
  gz::Entity entity{gz::kNullEntity};
  std::string name;
  double inBinTime{0.0};
  std::string binName;
};

class SortingCellSystem
  : public gz::System,
  public gz::ISystemConfigure,
  public gz::ISystemPreUpdate,
  public gz::ISystemPostUpdate
{
public:
  void Configure(
    const gz::Entity & world, const std::shared_ptr<const sdf::Element> & sdf,
    gz::EntityComponentManager &, gz::EventManager & eventMgr) override
  {
    world_ = world;
    eventMgr_ = &eventMgr;

    robotModel_ = param<std::string>(sdf, "robot_model", "ur5");
    tcpLink_ = param<std::string>(sdf, "tcp_link", "wrist_3_link");
    tcpOffset_ = param<Pose3d>(sdf, "tcp_offset", Pose3d(0, 0, 0.16, 0, 0, 0));
    attachTol_ = param<double>(sdf, "attach_tolerance", 0.03);
    slipProb_ = param<double>(sdf, "slip_probability", 0.0);

    beltMinX_ = param<double>(sdf, "belt_min_x", -1.6);
    beltMaxX_ = param<double>(sdf, "belt_max_x", 1.6);
    beltY_ = param<double>(sdf, "belt_center_y", 0.6);
    beltWidth_ = param<double>(sdf, "belt_width", 0.3);
    beltZ_ = param<double>(sdf, "belt_surface_z", 0.72);
    speed_ = param<double>(sdf, "belt_speed", 0.08);

    spawnX_ = param<double>(sdf, "spawn_x", -1.4);
    spawnInterval_ = param<double>(sdf, "spawn_interval", 6.0);
    spawnEnabled_ = param<bool>(sdf, "spawn_enabled", true);
    yJitter_ = param<double>(sdf, "spawn_y_jitter", 0.07);
    defectRatio_ = param<double>(sdf, "defect_ratio", 0.25);
    touchingProb_ = param<double>(sdf, "touching_probability", 0.0);
    scaleJitter_ = param<double>(sdf, "scale_jitter", 0.0);
    colourJitter_ = param<double>(sdf, "colour_jitter", 0.0);
    maxParts_ = param<int>(sdf, "max_parts", 0);
    rng_.seed(param<int>(sdf, "seed", 42));

    types_ = {
      {"cube", {0.15, 0.35, 0.85}},
      {"cylinder", {0.95, 0.70, 0.10}},
      {"hex_nut", {0.55, 0.57, 0.60}},
    };
    if (sdf && sdf->HasElement("types")) {
      std::istringstream ss(sdf->Get<std::string>("types"));
      std::vector<PartType> keep;
      std::string t;
      while (ss >> t) {
        for (const auto & pt : types_) {
          if (pt.name == t) {keep.push_back(pt);}
        }
      }
      if (!keep.empty()) {types_ = keep;}
    }

    if (sdf && sdf->HasElement("bin")) {
      auto el = std::const_pointer_cast<sdf::Element>(sdf)->GetElement("bin");
      while (el) {
        Bin b;
        b.name = el->Get<std::string>("name");
        auto c = el->Get<ignition::math::Vector2d>("center");
        auto s = el->Get<ignition::math::Vector2d>("size");
        b.cx = c.X(); b.cy = c.Y(); b.sx = s.X(); b.sy = s.Y();
        b.zTop = el->Get<double>("z_top");
        bins_.push_back(b);
        el = el->GetNextElement("bin");
      }
    }

    node_.Subscribe("/conveyor/cmd_speed", &SortingCellSystem::OnSpeed, this);
    node_.Subscribe("/gripper/attach", &SortingCellSystem::OnAttach, this);
    node_.Subscribe("/parts/spawn_enable", &SortingCellSystem::OnSpawnEnable, this);
    node_.Subscribe("/parts/clear", &SortingCellSystem::OnClear, this);
    speedPub_ = node_.Advertise<ignition::msgs::Double>("/conveyor/speed");
    attachedPub_ = node_.Advertise<ignition::msgs::StringMsg>("/gripper/attached");
    gtPub_ = node_.Advertise<ignition::msgs::Pose_V>("/parts/ground_truth");
    eventPub_ = node_.Advertise<ignition::msgs::StringMsg>("/parts/events");

    ignmsg << "[SortingCell] belt " << beltMinX_ << ".." << beltMaxX_ << " @ " << speed_
           << " m/s, spawn every " << spawnInterval_ << " s, " << bins_.size() << " bins\n";
  }

  void PreUpdate(const gz::UpdateInfo & info, gz::EntityComponentManager & ecm) override
  {
    if (!creator_) {
      creator_ = std::make_unique<gz::SdfEntityCreator>(ecm, *eventMgr_);
    }
    if (info.paused) {
      return;
    }
    const double now = secs(info.simTime);
    const double dt = secs(info.dt);

    std::lock_guard<std::mutex> lock(mutex_);

    if (clearRequested_) {
      for (auto & [name, p] : parts_) {
        creator_->RequestRemoveEntity(p.entity, true);
      }
      parts_.clear();
      attached_.reset();
      clearRequested_ = false;
      attachedDirty_ = true;
    }

    // --- spawning -------------------------------------------------------
    if (spawnEnabled_ && now >= nextSpawn_ &&
      (maxParts_ <= 0 || spawnedCount_ < maxParts_))
    {
      const double y = beltY_ + Uniform(-yJitter_, yJitter_);
      SpawnPart(ecm, spawnX_ + Uniform(-0.02, 0.02), y);
      if (touchingProb_ > 0.0 && Uniform(0, 1) < touchingProb_ &&
        (maxParts_ <= 0 || spawnedCount_ < maxParts_))
      {
        // second part right next to the first one (touching / clustered),
        // either behind it on the belt or beside it
        if (Uniform(0, 1) < 0.5) {
          SpawnPart(ecm, spawnX_ + 0.058 + Uniform(-0.003, 0.003), y + Uniform(-0.01, 0.01));
        } else {
          const double side = (y > beltY_) ? -1.0 : 1.0;
          SpawnPart(ecm, spawnX_ + Uniform(-0.01, 0.01), y + side * (0.058 + Uniform(-0.003, 0.003)));
        }
      }
      nextSpawn_ = now + spawnInterval_ * Uniform(0.8, 1.2);
    }

    // --- grasp requests -------------------------------------------------
    std::optional<Pose3d> tcp = TcpPose(ecm);
    if (attachRequest_.has_value()) {
      bool wantAttach = *attachRequest_;
      attachRequest_.reset();
      if (wantAttach && !attached_ && tcp) {
        TryAttach(ecm, *tcp);
      } else if (!wantAttach && attached_) {
        Release(ecm);
      }
      attachedDirty_ = true;
    }

    // --- attached part follows the TCP ----------------------------------
    if (attached_ && tcp) {
      auto it = parts_.find(attached_->first);
      if (it == parts_.end()) {
        attached_.reset();
        attachedDirty_ = true;
      } else {
        SetPoseCmd(ecm, it->second.entity, (*tcp) * attached_->second);
        SetVelCmd(ecm, it->second.entity, Vector3d::Zero);
      }
    }

    // --- conveyor, bins and lost parts ----------------------------------
    std::vector<std::string> toRemove;
    for (auto & [name, p] : parts_) {
      if (attached_ && attached_->first == name) {
        continue;
      }
      auto poseComp = ecm.Component<gz::components::Pose>(p.entity);
      if (!poseComp) {
        continue;
      }
      const Pose3d & pose = poseComp->Data();
      const Vector3d & pos = pose.Pos();

      const bool onBelt = pos.X() > beltMinX_ && pos.X() < beltMaxX_ &&
        std::abs(pos.Y() - beltY_) < beltWidth_ / 2.0 &&
        pos.Z() > beltZ_ - 0.01 && pos.Z() < beltZ_ + 0.08;

      if (onBelt && std::abs(speed_) > 1e-6) {
        // velocity command is expressed in the model frame
        Vector3d worldVel(speed_, 0, 0);
        SetVelCmd(ecm, p.entity, pose.Rot().Inverse() * worldVel);
      } else if (onBelt) {
        SetVelCmd(ecm, p.entity, Vector3d::Zero);
      } else {
        ClearVelCmd(ecm, p.entity);
      }

      if (pos.X() >= beltMaxX_ - 0.05 && pos.Z() > beltZ_ - 0.05) {
        Event("missed " + name);
        toRemove.push_back(name);
        continue;
      }

      const Bin * inBin = nullptr;
      for (const auto & b : bins_) {
        if (std::abs(pos.X() - b.cx) < b.sx / 2 && std::abs(pos.Y() - b.cy) < b.sy / 2 &&
          pos.Z() < b.zTop)
        {
          inBin = &b;
        }
      }
      if (inBin) {
        if (p.binName != inBin->name) {
          p.binName = inBin->name;
          p.inBinTime = 0.0;
        }
        p.inBinTime += dt;
        if (p.inBinTime > 1.5) {
          Event("binned " + name + " " + inBin->name);
          toRemove.push_back(name);
        }
        continue;
      }
      p.binName.clear();
      p.inBinTime = 0.0;

      if (pos.Z() < 0.3) {
        Event("dropped " + name);
        toRemove.push_back(name);
      }
    }
    for (const auto & name : toRemove) {
      creator_->RequestRemoveEntity(parts_[name].entity, true);
      parts_.erase(name);
    }
  }

  void PostUpdate(const gz::UpdateInfo & info, const gz::EntityComponentManager & ecm) override
  {
    const double now = secs(info.simTime);
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto & e : pendingEvents_) {
      ignition::msgs::StringMsg m;
      m.set_data(e);
      eventPub_.Publish(m);
    }
    pendingEvents_.clear();

    if (attachedDirty_ || now - lastStatePub_ > 0.5) {
      ignition::msgs::StringMsg m;
      m.set_data(attached_ ? attached_->first : "");
      attachedPub_.Publish(m);
      attachedDirty_ = false;
    }

    if (now - lastStatePub_ >= 0.1) {
      lastStatePub_ = now;
      ignition::msgs::Double s;
      s.set_data(speed_);
      speedPub_.Publish(s);

      ignition::msgs::Pose_V pv;
      pv.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(now));
      pv.mutable_header()->mutable_stamp()->set_nsec(
        static_cast<int32_t>((now - std::floor(now)) * 1e9));
      for (const auto & [name, p] : parts_) {
        auto poseComp = ecm.Component<gz::components::Pose>(p.entity);
        if (!poseComp) {continue;}
        auto * msg = pv.add_pose();
        msg->set_name(name);
        ignition::msgs::Set(msg, poseComp->Data());
      }
      gtPub_.Publish(pv);
    }
  }

private:
  double Uniform(double a, double b)
  {
    return std::uniform_real_distribution<double>(a, b)(rng_);
  }

  void Event(const std::string & e)
  {
    pendingEvents_.push_back(e);
    igndbg << "[SortingCell] " << e << "\n";
  }

  std::string PartSdf(
    const std::string & name, const PartType & type, bool defect, double scale,
    const Vector3d & colour, const Pose3d & pose, double & height)
  {
    std::ostringstream geo;     // collision + visual body geometry
    double mass = 0.05;
    double topZ = 0.0;
    std::ostringstream body;
    auto boxGeom = [](double x, double y, double z) {
        std::ostringstream g;
        g << "<geometry><box><size>" << x << " " << y << " " << z << "</size></box></geometry>";
        return g.str();
      };
    const std::string surface =
      "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>"
      "<contact><ode><kp>1e5</kp><kd>10</kd><max_vel>0.1</max_vel><min_depth>0.0005</min_depth>"
      "</ode></contact></surface>";
    if (type.name == "cube") {
      double s = 0.04 * scale;
      height = s;
      topZ = s / 2;
      std::string g = boxGeom(s, s, s);
      body << "<collision name='c'>" << g << surface << "</collision>"
           << "<visual name='v'>" << g << material(colour) << "</visual>";
      mass = 0.06;
    } else if (type.name == "cylinder") {
      double r = 0.021 * scale, h = 0.045 * scale;
      height = h;
      topZ = h / 2;
      std::ostringstream g;
      g << "<geometry><cylinder><radius>" << r << "</radius><length>" << h
        << "</length></cylinder></geometry>";
      body << "<collision name='c'>" << g.str() << surface << "</collision>"
           << "<visual name='v'>" << g.str() << material(colour) << "</visual>";
      mass = 0.05;
    } else {  // hex_nut: union of three rotated boxes = regular hexagonal prism
      double af = 0.05 * scale;            // across flats
      double side = af / std::sqrt(3.0);   // edge length
      double h = 0.025 * scale;
      height = h;
      topZ = h / 2;
      for (int i = 0; i < 3; ++i) {
        std::ostringstream p;
        p << "<pose>0 0 0 0 0 " << i * M_PI / 3.0 << "</pose>";
        std::string g = boxGeom(side, af, h);
        body << "<collision name='c" << i << "'>" << p.str() << g << surface << "</collision>"
             << "<visual name='v" << i << "'>" << p.str() << g << material(colour) << "</visual>";
      }
      // dark centre hole (visual only) so the nut reads as a nut from above
      std::ostringstream hole;
      hole << "<visual name='hole'><pose>0 0 " << h / 2 << " 0 0 0</pose><geometry><cylinder><radius>"
           << 0.011 * scale << "</radius><length>0.001</length></cylinder></geometry>"
           << material({0.08, 0.08, 0.08}) << "</visual>";
      body << hole.str();
      mass = 0.04;
    }
    if (defect) {
      // rust-coloured damage patch on the top face, at a random offset/angle
      double s = (type.name == "hex_nut" ? 0.05 : 0.04) * scale;
      double ox = Uniform(-0.15, 0.15) * s, oy = Uniform(-0.15, 0.15) * s;
      std::ostringstream d;
      d << "<visual name='defect'><pose>" << ox << " " << oy << " " << topZ + 0.0006
        << " 0 0 " << Uniform(0, M_PI) << "</pose>" << boxGeom(0.75 * s, 0.32 * s, 0.001)
        << material({0.85, 0.12, 0.05}) << "</visual>";
      body << d.str();
    }
    const double i = mass * 0.04 * 0.04 / 6.0;
    std::ostringstream sdf;
    sdf << std::setprecision(6)
        << "<?xml version='1.0'?><sdf version='1.9'><model name='" << name << "'>"
        << "<pose>" << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z() << " "
        << pose.Rot().Roll() << " " << pose.Rot().Pitch() << " " << pose.Rot().Yaw() << "</pose>"
        << "<link name='link'><inertial><mass>" << mass << "</mass><inertia><ixx>" << i
        << "</ixx><iyy>" << i << "</iyy><izz>" << i << "</izz></inertia></inertial>"
        << body.str() << "</link></model></sdf>";
    return sdf.str();
  }

  void SpawnPart(gz::EntityComponentManager & ecm, double x, double y)
  {
    const PartType & base = types_[std::uniform_int_distribution<size_t>(0, types_.size() - 1)(rng_)];
    const bool defect = Uniform(0, 1) < defectRatio_;
    const double scale = 1.0 + Uniform(-scaleJitter_, scaleJitter_);
    Vector3d colour = base.colour;
    if (colourJitter_ > 0) {
      colour += Vector3d(
        Uniform(-colourJitter_, colourJitter_), Uniform(-colourJitter_, colourJitter_),
        Uniform(-colourJitter_, colourJitter_));
      colour.X(std::clamp(colour.X(), 0.0, 1.0));
      colour.Y(std::clamp(colour.Y(), 0.0, 1.0));
      colour.Z(std::clamp(colour.Z(), 0.0, 1.0));
    }
    std::ostringstream name;
    name << "part_" << std::setw(4) << std::setfill('0') << spawnedCount_ << "__" << base.name
         << "__" << (defect ? "defect" : "ok");

    double height = 0.04;
    // first pass to get the height for the spawn z
    PartSdf(name.str(), base, false, scale, colour, Pose3d::Zero, height);
    Pose3d pose(x, y, beltZ_ + height / 2 + 0.002,
      0, 0, Uniform(-M_PI, M_PI));
    std::string sdfStr = PartSdf(name.str(), base, defect, scale, colour, pose, height);

    sdf::Root root;
    auto errors = root.LoadSdfString(sdfStr);
    if (!errors.empty() || !root.Model()) {
      ignerr << "[SortingCell] failed to build part SDF: " << errors[0].Message() << "\n";
      return;
    }
    gz::Entity e = creator_->CreateEntities(root.Model());
    creator_->SetParent(e, world_);
    parts_[name.str()] = PartState{e, name.str(), 0.0, ""};
    ++spawnedCount_;
    (void)ecm;
    Event("spawned " + name.str());
  }

  std::optional<Pose3d> TcpPose(gz::EntityComponentManager & ecm)
  {
    if (tcpEntity_ == gz::kNullEntity) {
      gz::Entity model = ecm.EntityByComponents(
        gz::components::Model(), gz::components::Name(robotModel_),
        gz::components::ParentEntity(world_));
      if (model == gz::kNullEntity) {return std::nullopt;}
      tcpEntity_ = ecm.EntityByComponents(
        gz::components::Link(), gz::components::Name(tcpLink_),
        gz::components::ParentEntity(model));
      if (tcpEntity_ == gz::kNullEntity) {return std::nullopt;}
    }
    return gz::worldPose(tcpEntity_, ecm) * tcpOffset_;
  }

  void TryAttach(gz::EntityComponentManager & ecm, const Pose3d & tcp)
  {
    double best = std::numeric_limits<double>::max();
    std::string bestName;
    for (const auto & [name, p] : parts_) {
      auto poseComp = ecm.Component<gz::components::Pose>(p.entity);
      if (!poseComp) {continue;}
      double d = poseComp->Data().Pos().Distance(tcp.Pos());
      if (d < best) {
        best = d;
        bestName = name;
      }
    }
    if (bestName.empty() || best > attachTol_) {
      std::ostringstream ss;
      ss << "grasp_failed none_in_reach";
      if (!bestName.empty()) {
        ss << " nearest=" << bestName << " dist=" << std::fixed << std::setprecision(4) << best
           << " tcp=" << tcp.Pos();
      }
      Event(ss.str());
      return;
    }
    if (slipProb_ > 0 && Uniform(0, 1) < slipProb_) {
      Event("grasp_failed slipped " + bestName);
      return;
    }
    const Pose3d partPose = ecm.Component<gz::components::Pose>(parts_[bestName].entity)->Data();
    attached_ = std::make_pair(bestName, tcp.Inverse() * partPose);
    Event("attached " + bestName);
  }

  void Release(gz::EntityComponentManager & ecm)
  {
    auto it = parts_.find(attached_->first);
    if (it != parts_.end()) {
      ClearVelCmd(ecm, it->second.entity);
      Event("released " + it->first);
    }
    attached_.reset();
  }

  void SetPoseCmd(gz::EntityComponentManager & ecm, gz::Entity e, const Pose3d & pose)
  {
    auto comp = ecm.Component<gz::components::WorldPoseCmd>(e);
    if (!comp) {
      ecm.CreateComponent(e, gz::components::WorldPoseCmd(pose));
    } else {
      comp->Data() = pose;
      ecm.SetChanged(e, gz::components::WorldPoseCmd::typeId, gz::ComponentState::OneTimeChange);
    }
  }

  void SetVelCmd(gz::EntityComponentManager & ecm, gz::Entity e, const Vector3d & v)
  {
    auto lin = ecm.Component<gz::components::LinearVelocityCmd>(e);
    if (!lin) {
      ecm.CreateComponent(e, gz::components::LinearVelocityCmd(v));
    } else {
      lin->Data() = v;
    }
    auto ang = ecm.Component<gz::components::AngularVelocityCmd>(e);
    if (!ang) {
      ecm.CreateComponent(e, gz::components::AngularVelocityCmd(Vector3d::Zero));
    } else {
      ang->Data() = Vector3d::Zero;
    }
  }

  void ClearVelCmd(gz::EntityComponentManager & ecm, gz::Entity e)
  {
    // The physics system keeps applying a velocity command (zeroed after each
    // step) as long as the component exists, so it must be removed for the
    // part to move freely again.
    if (ecm.Component<gz::components::LinearVelocityCmd>(e)) {
      ecm.RemoveComponent<gz::components::LinearVelocityCmd>(e);
    }
    if (ecm.Component<gz::components::AngularVelocityCmd>(e)) {
      ecm.RemoveComponent<gz::components::AngularVelocityCmd>(e);
    }
  }

  // transport callbacks (run on transport threads)
  void OnSpeed(const ignition::msgs::Double & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = m.data();
  }
  void OnAttach(const ignition::msgs::Boolean & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    attachRequest_ = m.data();
  }
  void OnSpawnEnable(const ignition::msgs::Boolean & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    spawnEnabled_ = m.data();
  }
  void OnClear(const ignition::msgs::Boolean & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (m.data()) {clearRequested_ = true;}
  }

  gz::Entity world_{gz::kNullEntity};
  gz::Entity tcpEntity_{gz::kNullEntity};
  std::unique_ptr<gz::SdfEntityCreator> creator_;  // created on first PreUpdate (needs the ECM)
  gz::EventManager * eventMgr_{nullptr};
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher speedPub_, attachedPub_, gtPub_, eventPub_;
  std::mutex mutex_;
  std::mt19937 rng_;

  std::string robotModel_, tcpLink_;
  Pose3d tcpOffset_;
  double attachTol_{0.03}, slipProb_{0.0};
  double beltMinX_, beltMaxX_, beltY_, beltWidth_, beltZ_, speed_;
  double spawnX_, spawnInterval_, yJitter_, defectRatio_, touchingProb_, scaleJitter_, colourJitter_;
  bool spawnEnabled_{true}, clearRequested_{false}, attachedDirty_{true};
  int maxParts_{0}, spawnedCount_{0};
  double nextSpawn_{2.0}, lastStatePub_{-1.0};
  std::vector<PartType> types_;
  std::vector<Bin> bins_;
  std::map<std::string, PartState> parts_;
  std::optional<std::pair<std::string, Pose3d>> attached_;
  std::optional<bool> attachRequest_;
  std::vector<std::string> pendingEvents_;
};

}  // namespace sorting_cell

IGNITION_ADD_PLUGIN(
  sorting_cell::SortingCellSystem, ignition::gazebo::System,
  sorting_cell::SortingCellSystem::ISystemConfigure,
  sorting_cell::SortingCellSystem::ISystemPreUpdate,
  sorting_cell::SortingCellSystem::ISystemPostUpdate)
