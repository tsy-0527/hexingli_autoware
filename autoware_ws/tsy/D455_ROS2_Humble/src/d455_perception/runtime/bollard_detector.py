"""Optional same-frame YOLO detection and a deliberately simple RGB-D baseline."""
import hashlib
import math
from pathlib import Path
import time

import numpy as np


def measure_boxes(depth, scale, boxes, names=None, image_shape=None, confidence=.4):
    if not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError('Confidence must be between 0 and 1.')
    names = {0: 'shidunzi'} if names is None else names
    if depth is not None:
        if depth.ndim != 2 or depth.dtype != np.uint16:
            raise ValueError('Expected aligned uint16 depth.')
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('Invalid depth scale.')
        h, w = depth.shape
    else:
        h, w = image_shape[:2]
    detections = []
    for row in boxes:
        if len(row) != 6 or not np.isfinite(row).all():
            continue
        x1, y1, x2, y2, score, cls = map(float, row)
        if cls not in names or not confidence < score <= 1:
            continue
        x1, x2 = np.clip([x1, x2], 0, w)
        y1, y2 = np.clip([y1, y2], 0, h)
        if x2 <= x1 or y2 <= y1:
            continue
        detection = {'bbox_xyxy': [float(x1), float(y1), float(x2), float(y2)],
                     'confidence': score, 'class_id': int(cls), 'class_name': names[int(cls)],
                     'depth_roi_xyxy': None, 'roi_median_z_m': None,
                     'valid_fraction': 0., 'valid_pixels': 0, 'depth_status': 'not_available'}
        if depth is None:
            detections.append(detection)
            continue
        # shortcut: central 40% can contain occluders/background; verify each target type against measured distances.
        rx1, rx2 = int(math.floor(x1+.3*(x2-x1))), int(math.ceil(x2-.3*(x2-x1)))
        ry1, ry2 = int(math.floor(y1+.3*(y2-y1))), int(math.ceil(y2-.3*(y2-y1)))
        roi = depth[ry1:ry2, rx1:rx2]
        values = roi[roi > 0]
        fraction = float(values.size/roi.size) if roi.size else 0.
        enough = values.size >= 9 and fraction >= .5
        detection.update({
            'depth_roi_xyxy': [rx1, ry1, rx2, ry2],
            'roi_median_z_m': float(np.median(values)*scale) if enough else None,
            'valid_fraction': fraction, 'valid_pixels': int(values.size),
            'depth_status': 'unverified_estimate' if enough else 'unknown'})
        detections.append(detection)
    return detections


def measure_boxes_projected(projected, scale, boxes, names=None, image_shape=None, confidence=.4):
    """Measure depth for boxes using points projected from raw depth into color."""
    if not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError('Confidence must be between 0 and 1.')
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid depth scale.')
    names = {0: 'shidunzi'} if names is None else names
    if image_shape is None:
        raise ValueError('image_shape is required for projected depth.')
    h, w = image_shape[:2]
    u, v, values_mm = projected[:3]
    detections = []
    for row in boxes:
        if len(row) != 6 or not np.isfinite(row).all():
            continue
        x1, y1, x2, y2, score, cls = map(float, row)
        if cls not in names or not confidence < score <= 1:
            continue
        x1, x2 = np.clip([x1, x2], 0, w)
        y1, y2 = np.clip([y1, y2], 0, h)
        if x2 <= x1 or y2 <= y1:
            continue
        detection = {'bbox_xyxy': [float(x1), float(y1), float(x2), float(y2)],
                     'confidence': score, 'class_id': int(cls), 'class_name': names[int(cls)],
                     'depth_roi_xyxy': None, 'roi_median_z_m': None,
                     'valid_fraction': 0., 'valid_pixels': 0, 'depth_status': 'not_available'}
        rx1, rx2 = int(math.floor(x1+.3*(x2-x1))), int(math.ceil(x2-.3*(x2-x1)))
        ry1, ry2 = int(math.floor(y1+.3*(y2-y1))), int(math.ceil(y2-.3*(y2-y1)))
        area = max(1, (rx2-rx1)*(ry2-ry1))
        mask = (u >= rx1) & (u < rx2) & (v >= ry1) & (v < ry2)
        values = values_mm[mask]
        fraction = min(1.0, float(values.size)/area)
        enough = values.size >= 9 and fraction >= .5
        detection.update({
            'depth_roi_xyxy': [rx1, ry1, rx2, ry2],
            'roi_median_z_m': float(np.median(values)*scale) if enough else None,
            'valid_fraction': fraction, 'valid_pixels': int(values.size),
            'depth_status': 'unverified_estimate' if enough else 'unknown'})
        detections.append(detection)
    return detections


class BollardDetector:
    """Historical public name; supports the old bollard model and the new three-class model."""
    def __init__(self, weights, device='cpu', imgsz=960, confidence=.4):
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('Confidence must be between 0 and 1.')
        self.confidence = confidence
        path = Path(weights).resolve()
        if not path.is_file():
            raise FileNotFoundError(f'Detection weights not found: {path}')
        import ultralytics
        from ultralytics import YOLO
        self.model = YOLO(str(path))
        self.names = dict(self.model.names)
        if self.model.task != 'detect' or self.names not in ({0: 'shidunzi'}, {0: 'car', 1: 'shidunzi', 2: 'xingren'}):
            raise ValueError('Expected old shidunzi weights or new 0=car, 1=shidunzi, 2=xingren weights.')
        self.device, self.imgsz = device, imgsz
        self.info = {'weights_file': path.name, 'weights_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'ultralytics_version': ultralytics.__version__, 'device_requested': device,
            'imgsz': imgsz, 'confidence_rule': 'strictly greater than per-frame confidence_threshold',
            'initial_confidence_threshold': confidence, 'class_names': self.names,
            'depth_method': 'central 40 percent of clipped box; positive depth median',
            'minimum_valid_pixels': 9, 'minimum_valid_fraction': .5,
            'distance_semantics': 'aligned depth ROI Z estimate; not verified object range',
            'scheduling': 'synchronous same-frame inference; may reduce capture FPS'}
        self.model.predict(np.zeros((480, 640, 3), np.uint8), **self.options())

    def options(self):
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('Confidence must be between 0 and 1.')
        return dict(conf=self.confidence, classes=list(self.names), imgsz=self.imgsz, device=self.device, verbose=False, save=False)

    def predict(self, arrays, scale):
        color, depth = arrays['color_bgr'], arrays['depth_aligned_to_color']
        if color.shape[:2] != depth.shape:
            raise ValueError('Color and aligned depth must share the same pixel grid.')
        return self.predict_color(color, depth, scale)

    def predict_color(self, color, depth=None, scale=None):
        begin = time.perf_counter()
        options = self.options()
        result = self.model.predict(color, **options)[0]
        boxes = result.boxes.data.detach().cpu().numpy() if result.boxes is not None else []
        return {'detections': measure_boxes(depth, scale, boxes, self.names, color.shape, options['conf']),
                'confidence_threshold': options['conf'],
                'processing_ms': (time.perf_counter()-begin)*1000}

    def predict_projected(self, arrays, scale):
        color, projected = arrays['color_bgr'], arrays['depth_projected']
        return self.predict_color_projected(color, projected, scale)

    def predict_color_projected(self, color, projected, scale=0.001):
        begin = time.perf_counter()
        options = self.options()
        result = self.model.predict(color, **options)[0]
        boxes = result.boxes.data.detach().cpu().numpy() if result.boxes is not None else []
        return {'detections': measure_boxes_projected(projected, scale, boxes, self.names, color.shape, options['conf']),
                'confidence_threshold': options['conf'],
                'processing_ms': (time.perf_counter()-begin)*1000}


def draw_detections(image, prediction, source_size):
    import cv2
    w, h = source_size
    sx, sy = image.shape[1]/w, image.shape[0]/h
    occupied = []
    for d in prediction['detections']:
        x1, y1, x2, y2 = [round(v*(sx if i % 2 == 0 else sy)) for i, v in enumerate(d['bbox_xyxy'])]
        z = d['roi_median_z_m']
        ink = {'car': (255, 180, 50), 'shidunzi': (70, 220, 80), 'xingren': (50, 190, 255)}[d['class_name']]
        cv2.rectangle(image, (x1, y1), (x2, y2), ink, 2)
        label = f"{ {'car': 'car', 'shidunzi': 'bollard', 'xingren': 'person'}[d['class_name']]} {d['confidence']:.2f}"
        if d['depth_roi_xyxy'] is not None:
            rx1, ry1, rx2, ry2 = [round(v*(sx if i % 2 == 0 else sy)) for i, v in enumerate(d['depth_roi_xyxy'])]
            cv2.rectangle(image, (rx1, ry1), (rx2, ry2), ink, 1)
            label += f" Z~{z:.2f}m" if z is not None else ' Z=UNKNOWN'
            label += f" v={d['valid_fraction']:.0%}"
        width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .43, 1)[0][0]
        tx, ty = max(0, min(x1, image.shape[1]-width-2)), max(16, min(y1-5, image.shape[0]-4))
        candidates = list(range(ty, 15, -20)) + list(range(ty+20, image.shape[0]-3, 20))
        for candidate in candidates:
            bounds = (tx, candidate-15, tx+width+2, candidate+4)
            if not any(bounds[0]<b[2] and bounds[2]>b[0] and bounds[1]<b[3] and bounds[3]>b[1] for b in occupied):
                ty = candidate
                break
        occupied.append((tx,ty-15,tx+width+2,ty+4))
        cv2.rectangle(image,(tx,ty-15),(min(image.shape[1]-1,tx+width+2),ty+4),(0,0,0),-1)
        cv2.putText(image, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, .43, ink, 1, cv2.LINE_AA)
