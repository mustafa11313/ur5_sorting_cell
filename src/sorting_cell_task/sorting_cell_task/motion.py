"""Blocking motion helpers on top of MoveIt 2's MoveGroup action and the gripper controller.

All methods block the calling thread and must therefore be called from a worker
thread while the node is spun by a MultiThreadedExecutor (see sorting_task.py).

Planner choice per motion:
    'PTP'  Pilz point-to-point (joint space, fast and repeatable)
    'LIN'  Pilz linear Cartesian motion (approach / retreat)
    'OMPL' RRTConnect (collision-avoiding fallback for transfers)
"""
import math
import threading
from dataclasses import dataclass

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
                             MotionPlanRequest, OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_GROUP = 'ur_manipulator'
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
              'wrist_2_joint', 'wrist_3_joint']
TCP_LINK = 'tcp'
FINGER_JOINTS = ['left_finger_joint', 'right_finger_joint']
MAX_FINGER = 0.04

PLANNERS = {
    'PTP': ('pilz_industrial_motion_planner', 'PTP'),
    'LIN': ('pilz_industrial_motion_planner', 'LIN'),
    'OMPL': ('ompl', 'RRTConnectkConfigDefault'),
}

ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
               if k.isupper() and isinstance(v, int)}


@dataclass
class MotionResult:
    ok: bool
    message: str = ''
    duration: float = 0.0


def tool_down_quaternion(yaw):
    """TCP orientation with the approach axis (z) pointing down, rotated by yaw about world z.

    R = Rz(yaw) * Rx(pi)  ->  quaternion (cos(yaw/2), sin(yaw/2), 0, 0) * (0, 1, 0, 0)
    """
    c, s = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    # q = qz(yaw) * qx(pi); qz = (0,0,s,c) xyzw, qx(pi) = (1,0,0,0) xyzw
    return (c, s, 0.0, 0.0)


def make_pose(xyz, quat):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    return p


def _wait(future, timeout):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout):
        return None
    return future.result()


class MotionClient:
    def __init__(self, node, velocity_scaling=0.5, acceleration_scaling=0.4, frame='world'):
        self.node = node
        self.frame = frame
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        group = ReentrantCallbackGroup()
        self.move_group = ActionClient(node, MoveGroup, 'move_action', callback_group=group)
        self.gripper = ActionClient(node, FollowJointTrajectory,
                                    '/gripper_controller/follow_joint_trajectory',
                                    callback_group=group)
        self.arm = ActionClient(node, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory', callback_group=group)
        self.ik_client = node.create_client(GetPositionIK, '/compute_ik', callback_group=group)

    def wait_for_servers(self, timeout=60.0):
        return (self.move_group.wait_for_server(timeout_sec=timeout) and
                self.gripper.wait_for_server(timeout_sec=timeout) and
                self.arm.wait_for_server(timeout_sec=timeout) and
                self.ik_client.wait_for_service(timeout_sec=timeout))

    def ik(self, pose, seed, timeout=0.2):
        """Collision-aware IK for the TCP, seeded with a joint dict. Returns a joint dict or None.

        Seeding matters: with a seed in the desired arm configuration (elbow up,
        wrist down) the solver returns the nearby solution, so joint-space PTP
        motions do not swing through the pedestal or the conveyor.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = ARM_GROUP
        req.ik_request.ik_link_name = TCP_LINK
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = Duration(seconds=timeout).to_msg()
        req.ik_request.robot_state.joint_state.name = list(seed)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed.values()]
        req.ik_request.pose_stamped = PoseStamped(pose=pose)
        req.ik_request.pose_stamped.header.frame_id = self.frame
        res = _wait(self.ik_client.call_async(req), 5.0)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n in ARM_JOINTS}

    # ------------------------------------------------------------------ arm
    def _request(self, planner, velocity_scaling=None, acceleration_scaling=None):
        pipeline, planner_id = PLANNERS[planner]
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.pipeline_id = pipeline
        req.planner_id = planner_id
        req.num_planning_attempts = 3 if planner == 'OMPL' else 1
        req.allowed_planning_time = 5.0 if planner == 'OMPL' else 2.0
        req.max_velocity_scaling_factor = velocity_scaling or self.velocity_scaling
        req.max_acceleration_scaling_factor = acceleration_scaling or self.acceleration_scaling
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = self.frame
        ws = req.workspace_parameters
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.5, -1.5, 0.0
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.5, 1.5, 2.0
        return req

    def _send(self, req, timeout=30.0):
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        t0 = self.node.get_clock().now()
        handle = _wait(self.move_group.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return MotionResult(False, 'goal rejected by move_group')
        res = _wait(handle.get_result_async(), timeout)
        dt = (self.node.get_clock().now() - t0).nanoseconds * 1e-9
        if res is None:
            return MotionResult(False, 'move_group timeout', dt)
        code = res.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return MotionResult(False, ERROR_NAMES.get(code, str(code)), dt)
        return MotionResult(True, 'ok', dt)

    def move_to_pose(self, pose, planner='PTP', fallback='OMPL', seed=None, **scaling):
        """Move the TCP to a geometry_msgs/Pose in the world frame.

        With a seed (joint dict) a PTP motion is planned to the seeded IK
        solution in joint space instead of letting the planner pick any solution.
        """
        if seed is not None and planner == 'PTP':
            joints = self.ik(pose, seed)
            if joints is not None:
                result = self.move_to_joints(joints, 'PTP', fallback=fallback, **scaling)
                if result.ok:
                    return result
            self.node.get_logger().warn('seeded IK/PTP failed, planning to the pose instead')
        req = self._request(planner, **scaling)
        req.goal_constraints.append(self._pose_constraints(pose))
        result = self._send(req)
        if not result.ok and fallback and fallback != planner:
            self.node.get_logger().warn(
                f'{planner} failed ({result.message}), retrying with {fallback}')
            req = self._request(fallback, **scaling)
            req.goal_constraints.append(self._pose_constraints(pose))
            result = self._send(req)
        return result

    def move_to_joints(self, joints, planner='PTP', fallback='OMPL', **scaling):
        req = self._request(planner, **scaling)
        c = Constraints()
        for name, value in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        result = self._send(req)
        if not result.ok and fallback and fallback != planner:
            req.pipeline_id, req.planner_id = PLANNERS[fallback]
            req.allowed_planning_time = 5.0
            result = self._send(req)
        return result

    def _pose_constraints(self, pose, pos_tol=0.001, rot_tol=0.01):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.frame
        pc.link_name = TCP_LINK
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        pc.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[pose])
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = TCP_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = rot_tol
        oc.absolute_z_axis_tolerance = rot_tol
        oc.weight = 1.0
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return c

    # -------------------------------------------------------------- gripper
    def set_gripper(self, opening, duration=0.4, timeout=5.0):
        """Command the total finger opening [m] (0 = closed, 0.08 = fully open)."""
        q = max(0.0, min(MAX_FINGER, opening / 2.0))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = FINGER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [q, q]
        pt.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points.append(pt)
        handle = _wait(self.gripper.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            return MotionResult(False, 'gripper goal rejected')
        res = _wait(handle.get_result_async(), timeout)
        if res is None:
            return MotionResult(False, 'gripper timeout')
        ok = res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return MotionResult(ok, 'ok' if ok else f'gripper error {res.result.error_code}')
