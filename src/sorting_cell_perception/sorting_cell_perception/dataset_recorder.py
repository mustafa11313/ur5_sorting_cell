"""Record an automatically labelled YOLO dataset from the simulation.

Labels come for free in simulation: every part blob found in the depth image
is matched with the ground-truth part poses published by the Gazebo plugin
(/parts/ground_truth, names like "part_0007__cube__defect"). Blobs that hold
several touching parts are split between them, so clustered scenes are
labelled correctly too.

Output layout (Ultralytics format):
    <output_dir>/images/{train,val}/xxxx.jpg
    <output_dir>/labels/{train,val}/xxxx.txt
    <output_dir>/data.yaml

    ros2 run sorting_cell_perception dataset_recorder --ros-args \
        -p output_dir:=$HOME/sorting_dataset -p max_images:=1500
"""
import os
import random

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .classes import CLASS_NAMES, gt_class_from_part_name
from .geometry import BeltRegion, height_map, part_mask
from .ros_utils import transform_to_matrix


def project(points_world, T_world_cam, K):
    """Nx3 world points -> Nx2 pixels (and depth)."""
    T_cam_world = np.linalg.inv(T_world_cam)
    p = points_world @ T_cam_world[:3, :3].T + T_cam_world[:3, 3]
    uv = p @ K.T
    return uv[:, :2] / uv[:, 2:3], p[:, 2]


def label_frame(depth, K, T_world_cam, belt, gt, min_area=60):
    """Return a list of (class_name, x1, y1, x2, y2) boxes for one frame.

    gt: dict part_name -> world xyz of the part centre.
    """
    pts, height, valid = height_map(depth, K, T_world_cam, belt)
    mask = part_mask(height, valid, belt)
    n, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    if n <= 1 or not gt:
        return []
    names = list(gt)
    centers = np.array([gt[k] for k in names])
    uv, _ = project(centers, T_world_cam, K)
    h, w = depth.shape
    boxes = []
    for label in range(1, n):
        ys, xs = np.nonzero(labels == label)
        if len(xs) < min_area:
            continue
        # which ground-truth parts does this blob contain?
        members = []
        for i, (u, v) in enumerate(uv):
            ui, vi = int(round(u)), int(round(v))
            if 0 <= ui < w and 0 <= vi < h:
                y0, y1, x0, x1 = max(0, vi - 4), vi + 5, max(0, ui - 4), ui + 5
                if (labels[y0:y1, x0:x1] == label).any():
                    members.append(i)
        if not members:
            continue
        if len(members) == 1:
            owner = np.zeros(len(xs), dtype=int)
        else:  # touching parts: nearest projected centre wins
            d = (xs[None, :] - uv[members, 0:1]) ** 2 + (ys[None, :] - uv[members, 1:2]) ** 2
            owner = np.argmin(d, axis=0)
        for k, gi in enumerate(members):
            sel = owner == k
            if sel.sum() < min_area:
                continue
            cls = gt_class_from_part_name(names[gi])
            if cls not in CLASS_NAMES:
                continue
            x1, x2, y1, y2 = xs[sel].min(), xs[sel].max(), ys[sel].min(), ys[sel].max()
            # drop parts cut by the image border (ambiguous for the classifier)
            if x1 <= 1 or y1 <= 1 or x2 >= w - 2 or y2 >= h - 2:
                continue
            boxes.append((cls, int(x1), int(y1), int(x2), int(y2)))
    return boxes


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        self.declare_parameter('output_dir', os.path.expanduser('~/sorting_dataset'))
        self.declare_parameter('period', 0.7)          # sim seconds between saved frames
        self.declare_parameter('max_images', 1500)
        self.declare_parameter('val_fraction', 0.15)
        self.declare_parameter('prefix', 'img')
        self.declare_parameter('cell_config', '')
        self.declare_parameter('previews', 8)

        self.out = self.get_parameter('output_dir').value
        self.period = self.get_parameter('period').value
        self.max_images = self.get_parameter('max_images').value
        self.val_fraction = self.get_parameter('val_fraction').value
        self.prefix = self.get_parameter('prefix').value
        self.previews = self.get_parameter('previews').value
        cell = self.get_parameter('cell_config').value
        self.belt = BeltRegion()
        if cell:
            with open(cell) as f:
                b = yaml.safe_load(f)['cell']['belt']
            self.belt = BeltRegion(surface_z=b['surface_z'], center_y=b['center_y'],
                                   width=b['width'], min_x=b['min_x'], max_x=b['max_x'])
        for split in ('train', 'val'):
            os.makedirs(os.path.join(self.out, 'images', split), exist_ok=True)
            os.makedirs(os.path.join(self.out, 'labels', split), exist_ok=True)
        os.makedirs(os.path.join(self.out, 'previews'), exist_ok=True)
        self._write_data_yaml()

        self.bridge = CvBridge()
        self.K = None
        self.T = None
        self.gt = {}
        self.last_save = -1e9
        self.count = len(os.listdir(os.path.join(self.out, 'images', 'train'))) + \
            len(os.listdir(os.path.join(self.out, 'images', 'val')))
        self.saved_now = 0
        self.rng = random.Random(len(self.prefix) + self.count)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/parts/ground_truth', self._on_gt, 10)
        rgb = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, '/camera/depth/image_raw',
                           qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, depth], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._on_images)
        self.get_logger().info(f'recording to {self.out} ({self.count} images already there)')

    def _write_data_yaml(self):
        data = {
            'path': os.path.abspath(self.out),
            'train': 'images/train',
            'val': 'images/val',
            'names': {i: n for i, n in enumerate(CLASS_NAMES)},
        }
        with open(os.path.join(self.out, 'data.yaml'), 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_gt(self, msg):
        self.gt = {t.child_frame_id: np.array([t.transform.translation.x,
                                               t.transform.translation.y,
                                               t.transform.translation.z])
                   for t in msg.transforms}

    def _on_images(self, rgb_msg, depth_msg):
        if self.K is None or self.saved_now >= self.max_images:
            return
        t = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        if t - self.last_save < self.period:
            return
        if self.T is None:
            try:
                tf = self.tf_buffer.lookup_transform('world', depth_msg.header.frame_id,
                                                     rclpy.time.Time())
            except Exception:  # noqa: BLE001
                return
            self.T = transform_to_matrix(tf.transform)
        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        boxes = label_frame(depth, self.K, self.T, self.belt, dict(self.gt))
        if not boxes:
            return
        self.last_save = t
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        split = 'val' if self.rng.random() < self.val_fraction else 'train'
        name = f'{self.prefix}_{self.count:05d}'
        h, w = bgr.shape[:2]
        cv2.imwrite(os.path.join(self.out, 'images', split, name + '.jpg'), bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(os.path.join(self.out, 'labels', split, name + '.txt'), 'w') as f:
            for cls, x1, y1, x2, y2 in boxes:
                cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
                bw, bh = (x2 - x1 + 1) / w, (y2 - y1 + 1) / h
                f.write(f'{CLASS_NAMES.index(cls)} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n')
        if self.saved_now < self.previews:
            prev = bgr.copy()
            for cls, x1, y1, x2, y2 in boxes:
                col = (0, 0, 255) if cls.endswith('defect') else (0, 255, 0)
                cv2.rectangle(prev, (x1, y1), (x2, y2), col, 1)
                cv2.putText(prev, cls, (x1, max(10, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            col, 1, cv2.LINE_AA)
            cv2.imwrite(os.path.join(self.out, 'previews', name + '.jpg'), prev)
        self.count += 1
        self.saved_now += 1
        if self.saved_now % 50 == 0 or self.saved_now == self.max_images:
            self.get_logger().info(f'{self.saved_now}/{self.max_images} images saved')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        while rclpy.ok() and node.saved_now < node.max_images:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
