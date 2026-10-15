"""Estimate the overhead camera's extrinsics (world -> camera_link) from ArUco markers.

Two markers with known world poses (cell.yaml: markers) lie next to the belt.
Their 8 corners are detected in a series of frames, the camera pose is solved
with PnP and averaged, and the result is written in the same YAML format as
sorting_cell_bringup/config/camera_extrinsics.yaml:

    ros2 run sorting_cell_perception calibrate_camera --ros-args \
        -p cell_config:=$(ros2 pkg prefix sorting_cell_bringup)/share/sorting_cell_bringup/config/cell.yaml \
        -p output:=/tmp/camera_extrinsics.yaml
    ros2 launch sorting_cell_bringup sorting_cell.launch.py camera_extrinsics:=/tmp/camera_extrinsics.yaml

The node also prints the difference to the extrinsics currently on TF, so it
doubles as a check that the camera has not been bumped.
"""
import math

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from .ros_utils import matrix_to_rpy, rpy_to_matrix, transform_to_matrix

# camera_link (x forward) -> camera_optical_frame (z forward), as in sim.launch.py
T_LINK_OPTICAL = np.eye(4)
T_LINK_OPTICAL[:3, :3] = rpy_to_matrix(-math.pi / 2, 0.0, -math.pi / 2)


def marker_corners_world(x, y, z, yaw, size):
    """World coordinates of the 4 marker corners in ArUco order (TL, TR, BR, BL).

    Seen from above with the marker's yaw = 0, its top edge faces +y.
    """
    s = size / 2.0
    local = np.array([[-s, s], [s, s], [s, -s], [-s, -s]])
    c, sn = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -sn], [sn, c]])
    xy = local @ R.T + np.array([x, y])
    return np.column_stack((xy, np.full(4, z)))


def solve_camera_pose(image_points, world_points, K):
    """PnP -> 4x4 T_world_camoptical and RMS reprojection error [px]."""
    ok, rvec, tvec = cv2.solvePnP(world_points.astype(np.float64),
                                  image_points.astype(np.float64), K, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None
    proj, _ = cv2.projectPoints(world_points, rvec, tvec, K, None)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - image_points) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    T_cam_world = np.eye(4)
    T_cam_world[:3, :3] = R
    T_cam_world[:3, 3] = tvec.ravel()
    return np.linalg.inv(T_cam_world), err


def average_poses(Ts):
    """Average rigid transforms (translation mean, rotation via SVD projection)."""
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    M = np.sum([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class CalibrateCamera(Node):
    def __init__(self):
        super().__init__('calibrate_camera')
        self.declare_parameter('cell_config', '')
        self.declare_parameter('output', 'camera_extrinsics.yaml')
        self.declare_parameter('frames', 20)
        cell_path = self.get_parameter('cell_config').value
        if not cell_path:
            raise RuntimeError('parameter cell_config (path to cell.yaml) is required')
        with open(cell_path) as f:
            cell = yaml.safe_load(f)['cell']
        self.size = cell.get('marker_size', 0.08)
        self.markers = {m['id']: m for m in cell['markers']}
        self.frames = self.get_parameter('frames').value
        self.output = self.get_parameter('output').value
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.bridge = CvBridge()
        self.K = None
        self.poses, self.errors = [], []
        self.done = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self._on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'looking for markers {sorted(self.markers)}')

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_image(self, msg):
        if self.K is None or self.done:
            return
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            self.get_logger().warn('no markers visible', throttle_duration_sec=3.0)
            return
        img_pts, world_pts = [], []
        for c, i in zip(corners, ids.ravel()):
            m = self.markers.get(int(i))
            if m is None:
                continue
            img_pts.append(c.reshape(4, 2))
            world_pts.append(marker_corners_world(m['x'], m['y'], m['top_z'], m['yaw'], self.size))
        if len(img_pts) < len(self.markers):
            return
        T_world_opt, err = solve_camera_pose(np.vstack(img_pts), np.vstack(world_pts), self.K)
        if T_world_opt is None:
            return
        self.poses.append(T_world_opt @ np.linalg.inv(T_LINK_OPTICAL))
        self.errors.append(err)
        if len(self.poses) >= self.frames:
            self._finish()

    def _finish(self):
        self.done = True
        T = average_poses(self.poses)
        roll, pitch, yaw = matrix_to_rpy(T[:3, :3])
        xyz = [round(float(v), 5) for v in T[:3, 3]]
        rpy = [round(float(v), 5) for v in (roll, pitch, yaw)]
        with open(self.output, 'w') as f:
            f.write('# Estimated by calibrate_camera from ArUco markers\n')
            yaml.safe_dump({'camera_link': {'parent': 'world', 'xyz': xyz, 'rpy': rpy}}, f,
                           default_flow_style=None)
        self.get_logger().info(
            f'camera_link: xyz={xyz} rpy={rpy}  (reprojection RMS {np.mean(self.errors):.2f} px, '
            f'{len(self.poses)} frames) -> {self.output}')
        try:
            tf = self.tf_buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())
            T_tf = transform_to_matrix(tf.transform)
            dt = np.linalg.norm(T_tf[:3, 3] - T[:3, 3])
            dR = T_tf[:3, :3].T @ T[:3, :3]
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
            self.get_logger().info(
                f'difference to the extrinsics on TF: {1000 * dt:.1f} mm, {ang:.2f} deg')
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateCamera()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
