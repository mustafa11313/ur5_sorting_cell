"""Thin wrapper around an Ultralytics YOLO detector."""
import numpy as np


class YoloDetector:
    def __init__(self, model_path, conf=0.4, device='cpu', imgsz=640):
        from ultralytics import YOLO  # imported lazily: optional dependency
        self.model = YOLO(model_path)
        self.conf = conf
        self.device = device
        self.imgsz = imgsz
        self.names = self.model.names

    def __call__(self, bgr):
        """Return a list of (class_name, confidence, (x1, y1, x2, y2))."""
        res = self.model.predict(bgr, conf=self.conf, device=self.device, imgsz=self.imgsz,
                                 verbose=False)[0]
        out = []
        if res.boxes is None:
            return out
        xyxy = res.boxes.xyxy.cpu().numpy()
        cls = res.boxes.cls.cpu().numpy().astype(int)
        conf = res.boxes.conf.cpu().numpy()
        for box, c, p in zip(xyxy, cls, conf):
            out.append((self.names[int(c)], float(p), tuple(np.round(box).astype(int))))
        return out
