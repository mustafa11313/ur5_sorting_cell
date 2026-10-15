"""Sorting cell task logic: a py_trees behavior tree driving perception, MoveIt 2 and the belt.

    cycle_or_recover (Selector)
    ├── sort_one_part (Sequence)
    │   ├── WaitForPart        belt runs until a part reaches the pick position
    │   ├── StopBelt
    │   ├── Settle             let the part come to rest, get fresh detections
    │   ├── Retry x2
    │   │   └── pick (Sequence)
    │   │       ├── SelectTarget   most downstream part + collision-free grasp
    │   │       └── PickPart       approach, close, verify, lift
    │   ├── ResumeBelt         belt restarts while the robot places the part
    │   ├── PlacePart          transfer to the bin chosen from the class
    │   └── ReportResult
    └── recover (Sequence)
        ├── ReportFailure
        ├── GoReady
        └── ResumeBelt

Independently of the tree, a belt supervisor stops the belt whenever a part is
about to leave the robot's reach while the robot is busy (e.g. placing).
"""
import math
import threading
import time

import py_trees
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene
from py_trees.common import Status
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float64, String
from vision_msgs.msg import Detection3DArray

from sorting_cell_interfaces.msg import SortEvent

from .cell import bin_for_class, load_cell
from .grasp import PartFootprint, plan_grasp
from .motion import MotionClient, make_pose, tool_down_quaternion

READY_JOINTS = {  # tool down, beside the robot, out of the camera's view
    'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.5708, 'elbow_joint': 1.5708,
    'wrist_1_joint': -1.5708, 'wrist_2_joint': -1.5708, 'wrist_3_joint': 0.0,
}


# angle between shoulder_pan and the TCP direction in the ready pose (UR shoulder/wrist offsets)
PAN_OFFSET = math.atan2(0.109, 0.487)


def seed_towards(x, y):
    """Ready-pose configuration turned towards (x, y): a good IK seed for poses there."""
    seed = dict(READY_JOINTS)
    seed['shoulder_pan_joint'] = math.atan2(y, x) - PAN_OFFSET
    return seed


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Part:
    """A perceived part (from vision_msgs/Detection3D)."""

    def __init__(self, det, stamp):
        c = det.bbox.center
        self.id = det.id
        self.x, self.y, self.z = c.position.x, c.position.y, c.position.z
        self.yaw = quat_yaw(c.orientation)
        self.length, self.width, self.height = det.bbox.size.x, det.bbox.size.y, det.bbox.size.z
        hyp = det.results[0].hypothesis if det.results else None
        self.cls = hyp.class_id if hyp else 'unknown'
        self.conf = hyp.score if hyp else 0.0
        self.stamp = stamp

    @property
    def ptype(self):
        return self.cls.replace('_defect', '')

    def footprint(self):
        return PartFootprint(self.x, self.y, self.yaw, self.length, self.width, self.height,
                             self.ptype)


# --------------------------------------------------------------------------- behaviours
class TaskBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name, task):
        super().__init__(name)
        self.task = task


class AsyncAction(TaskBehaviour):
    """Runs `run()` in a worker thread; RUNNING until it returns True/False."""

    def initialise(self):
        self._result = None
        self._thread = threading.Thread(target=self._wrap, daemon=True)
        self._thread.start()

    def _wrap(self):
        try:
            self._result = bool(self.run())
        except Exception as e:  # noqa: BLE001  (report, do not kill the tree)
            self.task.get_logger().error(f'{self.name}: {e}')
            self.task.failure_reason = f'{self.name}: {e}'
            self._result = False

    def update(self):
        if self._result is None:
            return Status.RUNNING
        return Status.SUCCESS if self._result else Status.FAILURE

    def run(self):
        raise NotImplementedError


class WaitForPart(TaskBehaviour):
    def initialise(self):
        self.task.robot_busy = False
        self.task.set_belt_hold(False)

    def update(self):
        parts = self.task.fresh_parts(max_age=1.0)
        trigger = self.task.params['pick_x']
        if any(p.x >= trigger and p.x <= self.task.params['pick_window'][1] for p in parts):
            return Status.SUCCESS
        return Status.RUNNING


class StopBelt(TaskBehaviour):
    def update(self):
        self.task.robot_busy = True
        self.task.set_belt_hold(True)
        self.task.cycle_start = self.task.now()
        return Status.SUCCESS


class ResumeBelt(TaskBehaviour):
    def update(self):
        self.task.set_belt_hold(False)
        return Status.SUCCESS


class Settle(TaskBehaviour):
    """Wait until the belt is stopped and detections newer than the stop exist."""

    def initialise(self):
        self.t0 = self.task.now()

    def update(self):
        waited = self.task.now() - self.t0
        if waited < self.task.params['settle_time']:
            return Status.RUNNING
        # images taken after the belt has stopped (camera time, not arrival time)
        if self.task.fresh_parts(max_age=2.0, newer_than=self.t0 + 0.3):
            return Status.SUCCESS
        return Status.FAILURE if waited > 4.0 else Status.RUNNING


class SelectTarget(TaskBehaviour):
    def update(self):
        t = self.task
        lo, hi = t.params['pick_window']
        parts = t.fresh_parts(max_age=1.0)
        candidates = sorted((p for p in parts if lo <= p.x <= hi), key=lambda p: -p.x)
        for part in candidates:
            neighbours = [p.footprint() for p in parts if p.id != part.id]
            grasp = plan_grasp(part.footprint(), neighbours, belt_z=t.belt_z,
                               reference_psi=t.params['reference_yaw'])
            if grasp is None:
                t.get_logger().info(f'{part.id}: no collision-free grasp, trying the next part')
                continue
            t.target, t.grasp = part, grasp
            t.target_bin = bin_for_class(part.cls, part.conf, t.params['min_confidence'])
            t.get_logger().info(
                f'target {part.id} {part.cls} ({part.conf:.2f}) at ({part.x:.3f}, {part.y:.3f}) '
                f'grasp yaw {math.degrees(grasp.psi):.0f} deg -> {t.target_bin}')
            return Status.SUCCESS
        t.failure_reason = t.failure_reason or 'no graspable part in the pick window'
        return Status.FAILURE


class PickPart(AsyncAction):
    def run(self):
        t = self.task
        g = t.grasp
        t.attempts += 1
        q = tool_down_quaternion(g.psi)
        approach = make_pose((g.x, g.y, g.z + t.params['approach_height']), q)
        grasp = make_pose((g.x, g.y, g.z), q)

        if not t.motion.set_gripper(g.opening).ok:
            return t.fail('gripper did not open', SortEvent.RESULT_PLANNING_FAILED)
        res = t.motion.move_to_pose(approach, 'PTP', seed=seed_towards(g.x, g.y))
        if not res.ok:
            return t.fail(f'approach: {res.message}', SortEvent.RESULT_PLANNING_FAILED)
        res = t.motion.move_to_pose(grasp, 'LIN', fallback=None, velocity_scaling=0.3)
        if not res.ok:
            return t.fail(f'descend: {res.message}', SortEvent.RESULT_PLANNING_FAILED)
        t.motion.set_gripper(g.close_to)
        attached = t.grasp_and_verify()
        if not attached:
            t.release()
            t.motion.set_gripper(g.opening)
            t.motion.move_to_pose(approach, 'LIN', fallback='OMPL')
            return t.fail('nothing in the gripper', SortEvent.RESULT_GRASP_FAILED)
        t.attach_to_planning_scene()
        res = t.motion.move_to_pose(approach, 'LIN', fallback='OMPL', velocity_scaling=0.3)
        if not res.ok:
            return t.fail(f'lift: {res.message}', SortEvent.RESULT_PLANNING_FAILED)
        return True


class PlacePart(AsyncAction):
    def run(self):
        t = self.task
        x, y, floor = t.cell['bins'][t.target_bin]
        q = tool_down_quaternion(t.grasp.psi)   # keep the wrist where it is
        above = make_pose((x, y, floor + t.params['place_height']), q)
        res = t.motion.move_to_pose(above, 'PTP', seed=seed_towards(x, y))
        if not res.ok:
            t.release()
            t.detach_from_planning_scene()
            return t.fail(f'transfer: {res.message}', SortEvent.RESULT_PLANNING_FAILED)
        t.release()
        t.detach_from_planning_scene()
        t.motion.set_gripper(0.08)
        t.cycle_time = t.now() - t.cycle_start
        res = t.motion.move_to_joints(READY_JOINTS, 'PTP')
        if not res.ok:
            t.get_logger().warn(f'return to ready failed: {res.message}')
        t.robot_busy = False
        return True


class GoReady(AsyncAction):
    def run(self):
        t = self.task
        t.release()
        t.detach_from_planning_scene()
        t.motion.set_gripper(0.08)
        return t.motion.move_to_joints(READY_JOINTS, 'PTP').ok


class ReportResult(TaskBehaviour):
    def update(self):
        self.task.report(SortEvent.RESULT_SORTED)
        return Status.SUCCESS


class ReportFailure(TaskBehaviour):
    def update(self):
        t = self.task
        if t.target is not None or t.attempts > 0:
            t.report(t.failure_code)
        return Status.SUCCESS


# --------------------------------------------------------------------------- node
class SortingTask(Node):
    def __init__(self):
        super().__init__('sorting_task')
        self.declare_parameter('cell_config', '')
        self.declare_parameter('belt_speed', 0.08)
        self.declare_parameter('pick_x', 0.0)
        self.declare_parameter('pick_window', [-0.2, 0.3])
        self.declare_parameter('stop_x', 0.2)
        self.declare_parameter('spawn_enabled', True)
        self.declare_parameter('settle_time', 0.6)
        self.declare_parameter('approach_height', 0.10)
        self.declare_parameter('place_height', 0.24)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('reference_yaw', 0.0)
        self.declare_parameter('velocity_scaling', 0.8)
        self.declare_parameter('acceleration_scaling', 0.7)
        self.declare_parameter('tick_rate', 20.0)

        p = {n: self.get_parameter(n).value for n in (
            'belt_speed', 'pick_x', 'pick_window', 'stop_x', 'settle_time', 'approach_height',
            'place_height', 'min_confidence', 'reference_yaw')}
        self.params = p
        self.cell = load_cell(self.get_parameter('cell_config').value or None)
        self.belt_z = self.cell['belt']['surface_z']

        self.parts = []
        self.parts_stamp = 0.0
        self.attached_part = ''
        self.belt_hold = False
        self.robot_busy = False
        self.scene_attached = False
        self.cycle = 0
        self._reset_cycle()

        reentrant = ReentrantCallbackGroup()
        self.motion = MotionClient(self, self.get_parameter('velocity_scaling').value,
                                   self.get_parameter('acceleration_scaling').value)
        self.belt_pub = self.create_publisher(Float64, '/conveyor/cmd_speed', 10)
        self.spawn_pub = self.create_publisher(Bool, '/parts/spawn_enable', 10)
        self.spawn_released = False
        self.ready = False
        self.attach_pub = self.create_publisher(Bool, '/gripper/attach', 10)
        self.event_pub = self.create_publisher(SortEvent, '/sorting/events', 10)
        self.tree_pub = self.create_publisher(String, '/sorting/tree', 1)
        self.create_subscription(Detection3DArray, '/perception/parts', self._on_parts, 10,
                                 callback_group=reentrant)
        self.create_subscription(String, '/gripper/attached', self._on_attached, 10,
                                 callback_group=reentrant)
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                               callback_group=reentrant)
        self.tree = self._build_tree()
        self._last_tree = ''
        self.create_timer(1.0 / self.get_parameter('tick_rate').value, self._tick,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.1, self._supervise_belt, callback_group=reentrant)

    # ------------------------------------------------------------- tree
    def _build_tree(self):
        pick = py_trees.composites.Sequence('pick', memory=True, children=[
            SelectTarget('SelectTarget', self), PickPart('PickPart', self)])
        sort_one = py_trees.composites.Sequence('sort_one_part', memory=True, children=[
            WaitForPart('WaitForPart', self),
            StopBelt('StopBelt', self),
            Settle('Settle', self),
            py_trees.decorators.Retry('retry_pick', pick, num_failures=2),
            ResumeBelt('ResumeBelt', self),
            PlacePart('PlacePart', self),
            ReportResult('ReportResult', self),
        ])
        recover = py_trees.composites.Sequence('recover', memory=True, children=[
            ReportFailure('ReportFailure', self), GoReady('GoReady', self),
            ResumeBelt('ResumeBelt', self)])
        root = py_trees.composites.Selector('cycle_or_recover', memory=True,
                                            children=[sort_one, recover])
        return py_trees.trees.BehaviourTree(root)

    def _tick(self):
        if not self.ready:
            return
        self.tree.tick()
        status = self.tree.root.status
        if status in (Status.SUCCESS, Status.FAILURE):
            self._reset_cycle()
        text = py_trees.display.unicode_tree(self.tree.root, show_status=True)
        if text != self._last_tree:
            self._last_tree = text
            self.tree_pub.publish(String(data=text))

    def _reset_cycle(self):
        self.target = None
        self.grasp = None
        self.target_bin = ''
        self.attempts = 0
        self.failure_reason = ''
        self.failure_code = SortEvent.RESULT_LOST
        self.cycle_start = self.now()
        self.cycle_time = 0.0
        self.held_part = ''

    # ------------------------------------------------------------- state
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_parts(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # image time
        self.parts = [Part(d, stamp) for d in msg.detections]
        self.parts_stamp = stamp

    def _on_attached(self, msg):
        self.attached_part = msg.data

    def fresh_parts(self, max_age=0.5, newer_than=None):
        if newer_than is not None and self.parts_stamp < newer_than:
            return []
        if self.now() - self.parts_stamp > max_age:
            return []
        return list(self.parts)

    def set_belt_hold(self, hold):
        self.belt_hold = hold
        self._supervise_belt()

    def _supervise_belt(self):
        if not self.ready:
            # hold the line until the robot is up (MoveIt, controllers and the
            # detector can take a while on a slow machine), so nothing passes unsorted
            self.belt_pub.publish(Float64(data=0.0))
            self.spawn_pub.publish(Bool(data=False))
            return
        if not self.spawn_released:
            self.spawn_pub.publish(Bool(data=bool(self.get_parameter('spawn_enabled').value)))
            self.spawn_released = True
        speed = 0.0 if self.belt_hold else self.params['belt_speed']
        if speed > 0.0 and self.robot_busy:
            # a part is about to leave the reachable window while the robot is busy
            if any(p.x > self.params['stop_x'] for p in self.fresh_parts(max_age=1.0)):
                speed = 0.0
        self.belt_pub.publish(Float64(data=float(speed)))

    def fail(self, reason, code):
        self.failure_reason = reason
        self.failure_code = code
        self.get_logger().warn(f'cycle {self.cycle}: {reason}')
        return False

    # ------------------------------------------------------------- gripper
    def grasp_and_verify(self, timeout=1.5):
        self.attached_part = ''
        self.attach_pub.publish(Bool(data=True))
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.attached_part:
                self.held_part = self.attached_part
                return True
            time.sleep(0.05)
        return False

    def release(self):
        self.attach_pub.publish(Bool(data=False))

    def _apply_scene(self, scene):
        if self.scene_client.service_is_ready():
            self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))

    def attach_to_planning_scene(self):
        g, part = self.grasp, self.target
        aco = AttachedCollisionObject()
        aco.link_name = 'tcp'
        aco.touch_links = ['left_finger', 'right_finger', 'gripper_base_link']
        aco.object.header.frame_id = 'tcp'
        aco.object.id = 'held_part'
        size = max(part.length, part.width) + 0.004
        aco.object.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[size, size, part.height + 0.004]))
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.z = (g.z - (self.belt_z + part.height / 2.0))   # part centre below TCP
        aco.object.primitive_poses.append(pose)
        aco.object.operation = CollisionObject.ADD
        self.scene_attached = True
        self._apply_scene(PlanningScene(is_diff=True, robot_state=self._attached_state(aco)))

    def _attached_state(self, aco):
        state = RobotState(is_diff=True)
        state.attached_collision_objects.append(aco)
        return state

    def detach_from_planning_scene(self):
        if not self.scene_attached:
            return
        self.scene_attached = False
        aco = AttachedCollisionObject()
        aco.object.id = 'held_part'
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True, robot_state=self._attached_state(aco))
        rm = CollisionObject(id='held_part', operation=CollisionObject.REMOVE)
        rm.header.frame_id = 'world'
        scene.world.collision_objects.append(rm)
        self._apply_scene(scene)

    # ------------------------------------------------------------- reporting
    def report(self, result):
        ev = SortEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.cycle = self.cycle
        if self.target is not None:
            ev.part_id = self.target.id
            ev.predicted_class = self.target.cls
            ev.confidence = float(self.target.conf)
            ev.pick_pose = make_pose((self.grasp.x, self.grasp.y, self.grasp.z),
                                     tool_down_quaternion(self.grasp.psi)) if self.grasp else Pose()
        ev.target_bin = self.target_bin
        ev.attached_part = self.held_part
        ev.result = result
        ev.failure_reason = self.failure_reason if result != SortEvent.RESULT_SORTED else ''
        ev.attempts = self.attempts
        ev.cycle_time = float(self.cycle_time or (self.now() - self.cycle_start))
        self.event_pub.publish(ev)
        names = {0: 'SORTED', 1: 'GRASP_FAILED', 2: 'PLANNING_FAILED', 3: 'LOST'}
        self.get_logger().info(
            f'cycle {self.cycle}: {names[result]} {ev.predicted_class} -> {ev.target_bin} '
            f'({ev.cycle_time:.1f} s) {ev.failure_reason}')
        self.cycle += 1

    def startup(self):
        self.get_logger().info('waiting for move_group and the gripper controller ...')
        if not self.motion.wait_for_servers(timeout=120.0):
            raise RuntimeError('move_group / gripper controller not available')
        res = self.motion.move_to_joints(READY_JOINTS, 'PTP')
        self.get_logger().info(f'moved to ready: {res.message}')
        self.motion.set_gripper(0.08)
        self.get_logger().info('waiting for the part detector ...')
        while rclpy.ok() and self.parts_stamp == 0.0:
            time.sleep(0.2)
        self.ready = True
        self.get_logger().info('sorting started')


def main(args=None):
    rclpy.init(args=args)
    node = SortingTask()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.startup()
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.belt_pub.publish(Float64(data=0.0))
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
