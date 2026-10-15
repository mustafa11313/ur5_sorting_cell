"""Part detector: RGB-D image -> classified, tracked part poses in the world frame.

Pipeline per frame
  1. depth -> world point cloud -> height above the belt -> part blobs
     (metric centre, yaw, footprint, height; see geometry.py)
  2. classification
       backend=yolo:      YOLO boxes on the RGB image, matched to depth blobs
                          (a blob covering several boxes is split between them)
       backend=classical: geometry + colour rules (classical.py)
  3. tracking along the belt + label voting (tracker.py)
  4. publish vision_msgs/Detection3DArray on /perception/parts, a TF frame per
     part (part_<id>) and an annotated debug image.

Detection3D fields: id = "part_<track id>", bbox.center = part centre with the
yaw of the footprint's long side, bbox.size = (length, width, height);
results[0].hypothesis = (class name, confidence).
"""
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from . import classical
from .geometry import BeltRegion, measure, segment, split_by_boxes
from .ros_utils import transform_to_matrix, yaw_to_quat
from .tracker import BeltTracker

COLOURS = {
    'cube': (220, 120, 40), 'cylinder': (40, 200, 240), 'hex_nut': (180, 180, 180),
}


def _box_iou_with_mask(box, mask):
    x1, y1, x2, y2 = box
    sub = mask[max(0, y1):y2, max(0, x1):x2]
    return float(sub.sum()) / max(1, (x2 - x1) * (y2 - y1))


class DetectorNode(Node):
    def __init__(self):
        super().__init__('part_detector')
        self.declare_parameter('backend', 'classical')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('cell_config', '')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_tf', True)

        self.bridge = CvBridge()
        self.world_frame = self.get_parameter('world_frame').value
        self.belt = self._load_belt(self.get_parameter('cell_config').value)
        self.tracker = BeltTracker()
        self.belt_speed = 0.0
        self.K = None
        self.T_world_cam = None
        self.frame_times = []

        backend = self.get_parameter('backend').value
        self.yolo = None
        if backend == 'yolo':
            model = self.get_parameter('model_path').value
            if not model or not os.path.exists(model):
                self.get_logger().warn(
                    f'YOLO model "{model}" not found, falling back to the classical backend. '
                    'Train one with: ros2 run sorting_cell_perception train_yolo')
            else:
                from .yolo_backend import YoloDetector
                self.yolo = YoloDetector(model, self.get_parameter('confidence_threshold').value,
                                         self.get_parameter('device').value)
                self.get_logger().info(f'YOLO backend loaded: {model} classes={self.yolo.names}')
        self.backend = 'yolo' if self.yolo else 'classical'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.parts_pub = self.create_publisher(Detection3DArray, '/perception/parts', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 2)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, '/conveyor/speed', self._on_speed, 10)
        rgb = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, '/camera/depth/image_raw',
                           qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, depth], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._on_images)
        self.get_logger().info(f'part detector running with the {self.backend} backend')

    # ------------------------------------------------------------------ setup
    def _load_belt(self, path):
        belt = BeltRegion()
        if path:
            with open(path) as f:
                b = yaml.safe_load(f)['cell']['belt']
            belt = BeltRegion(surface_z=b['surface_z'], center_y=b['center_y'], width=b['width'],
                              min_x=b['min_x'], max_x=b['max_x'])
        return belt

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_speed(self, msg):
        self.belt_speed = msg.data

    def _camera_pose(self, frame_id):
        if self.T_world_cam is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, frame_id, rclpy.time.Time())
            except Exception as e:  # noqa: BLE001  (tf2 raises several types)
                self.get_logger().warn(f'waiting for camera TF: {e}', throttle_duration_sec=5.0)
                return None
            self.T_world_cam = transform_to_matrix(tf.transform)   # static: cache it
        return self.T_world_cam

    # --------------------------------------------------------------- pipeline
    def _on_images(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        T = self._camera_pose(depth_msg.header.frame_id)
        if T is None:
            return
        t0 = time.perf_counter()
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0

        blobs, pts, height = segment(depth, self.K, T, self.belt)
        detections = self._classify(bgr, blobs, pts, height)

        stamp = rgb_msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        tracks = self.tracker.update(detections, t, self.belt_speed)
        self._publish(tracks, rgb_msg.header)
        self.frame_times.append(time.perf_counter() - t0)
        if self.get_parameter('publish_debug_image').value and self.debug_pub.get_subscription_count():
            self._publish_debug(bgr, tracks, rgb_msg.header)
        if len(self.frame_times) >= 100:
            self.get_logger().info(
                f'{self.backend}: {1000 * np.mean(self.frame_times):.1f} ms/frame, '
                f'{len(self.tracker.tracks)} tracks')
            self.frame_times.clear()

    def _classify(self, bgr, blobs, pts, height):
        if self.yolo is None:
            return [(b, *classical.classify(b, bgr)) for b in blobs]

        boxes = self.yolo(bgr)
        out = []
        # assign each YOLO box to the blob it overlaps most
        owners = {}
        for i, (_, _, box) in enumerate(boxes):
            scores = [_box_iou_with_mask(box, b.mask) for b in blobs]
            if scores and max(scores) > 0.15:
                owners.setdefault(int(np.argmax(scores)), []).append(i)
        for bi, box_ids in owners.items():
            blob = blobs[bi]
            if len(box_ids) == 1:
                name, conf, _ = boxes[box_ids[0]]
                out.append((blob, name, conf))
                continue
            # several parts in one depth blob (touching): split it between the boxes
            sub_masks = split_by_boxes(blob.mask, [boxes[i][2] for i in box_ids])
            for i, m in zip(box_ids, sub_masks):
                b = measure(m, pts, height)
                if b is not None:
                    b.center[2] = self.belt.surface_z + b.height / 2.0
                    out.append((b, boxes[i][0], boxes[i][1]))
        return out

    # ---------------------------------------------------------------- outputs
    def _publish(self, tracks, header):
        arr = Detection3DArray()
        arr.header.stamp = header.stamp
        arr.header.frame_id = self.world_frame
        tfs = []
        for tr in tracks:
            name, conf = tr.label
            det = Detection3D()
            det.header = arr.header
            det.id = f'part_{tr.track_id}'
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            c = det.bbox.center
            c.position.x, c.position.y, c.position.z = map(float, tr.center)
            q = yaw_to_quat(tr.yaw)
            c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = q
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = tr.length, tr.width, tr.height
            hyp.pose.pose = c
            arr.detections.append(det)

            ts = TransformStamped()
            ts.header = arr.header
            ts.child_frame_id = det.id
            ts.transform.translation.x = c.position.x
            ts.transform.translation.y = c.position.y
            ts.transform.translation.z = c.position.z
            ts.transform.rotation = c.orientation
            tfs.append(ts)
        self.parts_pub.publish(arr)
        if tfs and self.get_parameter('publish_tf').value:
            self.tf_broadcaster.sendTransform(tfs)

    def _publish_debug(self, bgr, tracks, header):
        img = bgr.copy()
        for tr in tracks:
            name, conf = tr.label
            ptype = name.replace('_defect', '')
            colour = (0, 0, 255) if name.endswith('_defect') else COLOURS.get(ptype, (0, 255, 0))
            if tr.blob is not None:
                contours, _ = cv2.findContours(tr.blob.mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, contours, -1, colour, 2)
                x, y, _, _ = tr.blob.bbox
                cv2.putText(img, f'#{tr.track_id} {name} {conf:.2f}', (x, max(12, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
        cv2.putText(img, f'{self.backend}  belt {self.belt_speed:.2f} m/s', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header = header
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
