#!/usr/bin/env python3
"""Record the showcase camera and the detection overlay side by side into an MP4 and a GIF.

Start the cell with the showcase camera enabled, then run this script:
    ros2 launch sorting_cell_bringup sorting_cell.launch.py world_overrides:="showcase_camera=true"
    python3 scripts/record_video.py --seconds 60 --out docs/media/demo
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.bridge = CvBridge()
        self.latest = {}
        self.create_subscription(Image, '/showcase/image', lambda m: self._store('scene', m), 2)
        self.create_subscription(Image, '/perception/debug_image',
                                 lambda m: self._store('detections', m), 2)

    def _store(self, key, msg):
        self.latest[key] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def frame(self):
        if 'scene' not in self.latest:
            return None
        scene = self.latest['scene']
        det = self.latest.get('detections')
        if det is None:
            return scene
        h = scene.shape[0]
        det = cv2.resize(det, (int(det.shape[1] * h / det.shape[0]), h))
        return np.hstack((scene, det))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=60.0)
    p.add_argument('--fps', type=float, default=10.0)
    p.add_argument('--out', default='demo')
    p.add_argument('--gif-width', type=int, default=720)
    p.add_argument('--gif-speedup', type=int, default=2, help='keep every n-th frame in the GIF')
    args = p.parse_args()

    rclpy.init()
    node = Recorder()
    frames = []
    t_end = time.time() + args.seconds
    next_t = time.time()
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.time() >= next_t:
            f = node.frame()
            if f is not None:
                frames.append(f)
            next_t += 1.0 / args.fps
    node.destroy_node()
    rclpy.shutdown()
    if not frames:
        raise SystemExit('no frames received; is the showcase camera enabled?')

    h, w = frames[0].shape[:2]
    writer = cv2.VideoWriter(args.out + '.mp4', cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (w, h))
    for f in frames:
        writer.write(cv2.resize(f, (w, h)))
    writer.release()

    from PIL import Image as PILImage
    scale = args.gif_width / w
    gif = [PILImage.fromarray(cv2.cvtColor(cv2.resize(f, (args.gif_width, int(h * scale))),
                                           cv2.COLOR_BGR2RGB)).quantize(colors=128)
           for f in frames[::args.gif_speedup]]
    gif[0].save(args.out + '.gif', save_all=True, append_images=gif[1:], loop=0,
                duration=int(1000 / args.fps), optimize=True)
    print(f'wrote {args.out}.mp4 and {args.out}.gif ({len(frames)} frames)')


if __name__ == '__main__':
    main()
