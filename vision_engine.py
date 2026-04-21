import cv2
import numpy as np
import torch
import os
import time
from ultralytics import YOLO


CONF_THRESHOLDS = {
    'stone':   0.35,
    'paper':   0.60,
    'default': 0.60,
}

CONFIRM_FRAMES = {
    'stone':   3,
    'default': 1,
}


def preprocess_frame(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    l_eq = clahe.apply(l)
    enhanced = cv2.cvtColor(cv2.merge([l_eq, a, b]), cv2.COLOR_LAB2BGR)
    blurred  = cv2.GaussianBlur(enhanced, (0, 0), sigmaX=2)
    return cv2.addWeighted(enhanced, 1.6, blurred, -0.6, 0)


class VisionSystem:
    def __init__(self, model_path="model/best.pt"):
        if torch.cuda.is_available():
            self.compute_device = '0'
            self.use_half = True
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            self.compute_device = 'mps'
            self.use_half = False
        else:
            self.compute_device = 'cpu'
            self.use_half = False

        print(f"[*] Vision Engine initialized. Device: '{self.compute_device}'")

        if not os.path.exists(model_path):
            print(f"[!] ERROR: Model not found at {model_path}")
            self.model = None
        else:
            self.model = YOLO(model_path)
            print("[*] AI Model loaded successfully.")

        # Perspective transform: pixel → mm
        self.px_corners = np.array(
            [[387, 409], [126, 403], [140, 112], [391, 111]], dtype=np.float32
        )
        self.mm_corners = np.array(
            [[0, 0], [130, 0], [130, 130], [0, 130]], dtype=np.float32
        )
        self.transform_matrix = cv2.getPerspectiveTransform(
            self.px_corners, self.mm_corners
        )

        # Vision box boundaries in mm
        self.BOX_X_MIN = 0.0
        self.BOX_X_MAX = 130.0
        self.BOX_Y_MIN = 0.0
        self.BOX_Y_MAX = 130.0

        # Object must be above this Y to count as a valid exit
        # (avoids false exits from tracking glitches mid-box)
        self.EXIT_Y_THRESHOLD = 110.0

        # Active tracking: only objects confirmed inside the vision box
        self.active_objects = {}   # track_id → latest data dict
        self.exited_ids     = set()  # IDs already handed to robot
        self.frame_counts   = {}   # track_id → consecutive confirmed frames

        # FIX: track how many frames an object has been MISSING
        # If missing only 1-2 frames it's a tracking glitch — don't treat as exit
        # If missing 3+ frames at EXIT_Y_THRESHOLD → genuine exit
        self.missing_frames = {}   # track_id → consecutive missing frame count
        self.MISSING_TOLERANCE = 2  # frames of missing before we act on disappearance

    def _conf_for(self, label):
        return CONF_THRESHOLDS.get(label, CONF_THRESHOLDS['default'])

    def _confirm_needed(self, label):
        return CONFIRM_FRAMES.get(label, CONFIRM_FRAMES['default'])

    def _in_box(self, robot_x, robot_y):
        """Check if physical coordinates are inside the vision box."""
        return (self.BOX_X_MIN <= robot_x <= self.BOX_X_MAX and
                self.BOX_Y_MIN <= robot_y <= self.BOX_Y_MAX)

    def get_targets(self, frame):
        """
        Detect and track objects inside the vision box.
        Returns only objects that have genuinely exited the bottom edge.

        Two bugs fixed vs previous version:
        1. Out-of-box detections: boundary check now happens BEFORE adding
           to visible_this_frame — out-of-box objects are completely ignored.
        2. False exits from tracking glitches: objects must be missing for
           MISSING_TOLERANCE frames before being treated as exited.
           This prevents a brief 1-frame drop in tracking from triggering
           a premature pick.
        """
        if self.model is None:
            return [], frame

        enhanced = preprocess_frame(frame)

        results = self.model.track(
            enhanced,
            conf=0.25,
            iou=0.4,
            imgsz=1280,
            device=self.compute_device,
            persist=True,
            half=self.use_half,
            verbose=False,
            tracker="bytetrack.yaml"
        )

        newly_exited   = []
        annotated_frame = frame.copy()

        # Draw vision box boundary
        cv2.polylines(
            annotated_frame, [np.int32(self.px_corners)], True, (0, 255, 255), 2
        )

        # IDs seen AND inside the box this frame
        in_box_this_frame = set()

        for r in results:
            if r.boxes.id is None:
                continue

            for idx in range(len(r.boxes)):
                cls_idx = int(r.boxes.cls[idx])
                if cls_idx == 0:
                    continue

                label_name = self.model.names[cls_idx]
                confidence = float(r.boxes.conf[idx])

                if confidence < self._conf_for(label_name):
                    continue

                track_id = int(r.boxes.id[idx])
                box = r.boxes[idx]

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy_px = (x1 + x2) // 2, (y1 + y2) // 2

                pixel_point   = np.array([[[cx, cy_px]]], dtype=np.float32)
                physical_point = cv2.perspectiveTransform(pixel_point, self.transform_matrix)
                robot_x = float(physical_point[0][0][0])
                robot_y = float(physical_point[0][0][1])

                # Only process detections INSIDE the vision box.
                # Out-of-box detections are silently ignored — no drawing.
                if not self._in_box(robot_x, robot_y):
                    continue

                # Inside box — process normally
                in_box_this_frame.add(track_id)

                # Reset missing frame count since object is visible again
                self.missing_frames.pop(track_id, None)

                # Confirmation buffer
                self.frame_counts[track_id] = self.frame_counts.get(track_id, 0) + 1
                frames_seen = self.frame_counts[track_id]
                needed      = self._confirm_needed(label_name)
                confirmed   = frames_seen >= needed

                if confirmed:
                    self.active_objects[track_id] = {
                        'id':        track_id,
                        'label':     label_name,
                        'robot_x':   robot_x,
                        'robot_y':   robot_y,
                        'last_seen': time.time(),
                    }

                # Debug overlay
                if label_name == 'stone':
                    color  = (0, 255, 100) if confirmed else (0, 140, 255)
                    status = f"STONE {confidence:.2f} [{frames_seen}/{needed}]"
                else:
                    color  = (0, 200, 255) if confirmed else (100, 100, 100)
                    status = f"{label_name} {confidence:.2f}"

                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.circle(annotated_frame, (cx, cy_px), 5, (0, 255, 0), -1)
                cv2.putText(annotated_frame, f"[{track_id}] {status}",
                            (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(annotated_frame, f"X:{robot_x:.0f} Y:{robot_y:.0f}",
                            (x1, y1 - 8),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # FIX 2: Tolerance-based exit detection.
        # An object is only treated as exited after it has been missing from
        # the vision box for MISSING_TOLERANCE consecutive frames.
        # This prevents 1-frame tracking drops from triggering false exits.
        confirmed_active = set(self.active_objects.keys())
        just_disappeared = confirmed_active - in_box_this_frame

        for track_id in just_disappeared:
            if track_id in self.exited_ids:
                # Already handed off — clean up active tracking
                self.active_objects.pop(track_id, None)
                self.frame_counts.pop(track_id, None)
                continue

            # Increment missing frame counter
            self.missing_frames[track_id] = self.missing_frames.get(track_id, 0) + 1
            missing_count = self.missing_frames[track_id]

            if missing_count < self.MISSING_TOLERANCE:
                # Too soon — could be a tracking glitch, keep waiting
                continue

            # Object has been missing long enough — treat as genuine exit
            obj = self.active_objects.pop(track_id, None)
            self.frame_counts.pop(track_id, None)
            self.missing_frames.pop(track_id, None)

            if obj is None:
                continue

            if obj['robot_y'] >= self.EXIT_Y_THRESHOLD:
                # Genuine exit through bottom edge — trigger pick
                self.exited_ids.add(track_id)
                print(f"[>>] {obj['label']} ID:{track_id} exited at "
                      f"X={obj['robot_x']:.1f} Y={obj['robot_y']:.1f}")

                newly_exited.append({
                    'id':        track_id,
                    'label':     obj['label'],
                    'robot_x':   obj['robot_x'],
                    'robot_y':   obj['robot_y'],
                    'exit_time': obj['last_seen'],
                })

                cv2.putText(annotated_frame,
                            f"[{track_id}] {obj['label'].upper()} → ARM",
                            (30, 30 + 28 * len(newly_exited)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 100), 2)
            else:
                # Disappeared mid-box — tracking loss, not a real exit
                print(f"[?] ID:{track_id} lost mid-box at Y={obj['robot_y']:.1f}mm — ignoring")

        # Memory hygiene
        if len(self.exited_ids) > 200:
            self.exited_ids.clear()

        return newly_exited, annotated_frame