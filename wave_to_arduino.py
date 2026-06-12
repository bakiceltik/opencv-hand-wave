#!/usr/bin/env python3
"""Detect a hand wave from the webcam (MediaPipe Hand Landmarker) and notify an Arduino.

Only an actual hand triggers the detector: MediaPipe locates hand landmarks,
so faces, bodies, and background motion are never candidates. A wave is an
open palm moving left/right with enough amplitude and direction changes.
"""

from __future__ import annotations

import argparse
import collections
import glob
import math
import pathlib
import sys
import time
import urllib.request
from dataclasses import dataclass
from typing import Deque

import cv2
import mediapipe as mp
import serial
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

WRIST = 0
PALM_CENTER = 9  # middle finger MCP, a stable palm point
FINGER_TIP_PIP_PAIRS = ((8, 6), (12, 10), (16, 14), (20, 18))
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
DEFAULT_MODEL_PATH = pathlib.Path(__file__).resolve().parent / "models" / "hand_landmarker.task"


@dataclass
class MotionSample:
    timestamp: float
    x: float
    y: float


class WaveDetector:
    """Tracks the palm center over time and decides when the motion is a wave."""

    def __init__(
        self,
        *,
        min_amplitude: float,
        min_direction_step: float,
        direction_changes_needed: int,
        history_seconds: float,
        max_vertical_range: float,
        min_wave_duration_seconds: float,
        max_gap_seconds: float,
    ) -> None:
        self.min_amplitude = min_amplitude
        self.min_direction_step = min_direction_step
        self.direction_changes_needed = direction_changes_needed
        self.history_seconds = history_seconds
        self.max_vertical_range = max_vertical_range
        self.min_wave_duration_seconds = min_wave_duration_seconds
        self.max_gap_seconds = max_gap_seconds
        self.samples: Deque[MotionSample] = collections.deque()
        self.last_sample_at: float | None = None

    def reset(self) -> None:
        self.samples.clear()
        self.last_sample_at = None

    def update(self, center: tuple[float, float] | None, now: float) -> bool:
        self._trim_history(now)

        if center is None:
            if self.last_sample_at is None or (now - self.last_sample_at) > self.max_gap_seconds:
                self.reset()
            return False

        if self.last_sample_at is not None and (now - self.last_sample_at) > self.max_gap_seconds:
            self.reset()

        self.samples.append(MotionSample(timestamp=now, x=center[0], y=center[1]))
        self.last_sample_at = now
        self._trim_history(now)
        return self._looks_like_wave()

    def _trim_history(self, now: float) -> None:
        while self.samples and (now - self.samples[0].timestamp) > self.history_seconds:
            self.samples.popleft()

    def _looks_like_wave(self) -> bool:
        if len(self.samples) < 6:
            return False

        first_timestamp = self.samples[0].timestamp
        last_timestamp = self.samples[-1].timestamp
        if (last_timestamp - first_timestamp) < self.min_wave_duration_seconds:
            return False

        xs = [sample.x for sample in self.samples]
        ys = [sample.y for sample in self.samples]
        amplitude = max(xs) - min(xs)
        if amplitude < self.min_amplitude:
            return False

        vertical_range = max(ys) - min(ys)
        if vertical_range > self.max_vertical_range:
            return False

        signs: list[int] = []
        previous_x = xs[0]
        for current_x in xs[1:]:
            delta = current_x - previous_x
            if abs(delta) < self.min_direction_step:
                continue

            sign = 1 if delta > 0 else -1
            if not signs or sign != signs[-1]:
                signs.append(sign)
            previous_x = current_x

        direction_changes = max(0, len(signs) - 1)
        return direction_changes >= self.direction_changes_needed


class SerialSignalSender:
    def __init__(self, *, port: str | None, baudrate: int, dry_run: bool) -> None:
        self.port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self.connection: serial.Serial | None = None

        if not dry_run and port is not None:
            self.connection = serial.Serial(port, baudrate=baudrate, timeout=1)
            time.sleep(2.0)

    def send_wave(self) -> None:
        message = "WAVE"
        if self.dry_run or self.connection is None:
            print(f"[dry-run] {message}")
            return

        self.connection.write(f"{message}\n".encode("utf-8"))
        self.connection.flush()
        print(f"[serial] sent {message} to {self.port}")

    def close(self) -> None:
        if self.connection is not None and self.connection.is_open:
            self.connection.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect a hand wave from the webcam and send WAVE to an Arduino."
    )
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--serial-port",
        default="auto",
        help="Arduino serial device path, or 'auto' to guess one.",
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate.")
    parser.add_argument("--dry-run", action="store_true", help="Run without sending serial data.")
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Disable the OpenCV preview window.",
    )
    parser.add_argument(
        "--model-path",
        type=pathlib.Path,
        default=DEFAULT_MODEL_PATH,
        help="Path to the MediaPipe hand_landmarker.task model (downloaded if missing).",
    )
    parser.add_argument(
        "--cooldown-seconds",
        type=float,
        default=2.0,
        help="Minimum delay between WAVE signals.",
    )
    parser.add_argument(
        "--wave-amplitude",
        type=float,
        default=0.08,
        help="Minimum horizontal travel as a fraction of frame width.",
    )
    parser.add_argument(
        "--direction-step",
        type=float,
        default=0.015,
        help="Ignore back-and-forth jitter below this fraction of frame width.",
    )
    parser.add_argument(
        "--direction-changes",
        type=int,
        default=3,
        help="How many left/right direction changes count as a wave.",
    )
    parser.add_argument(
        "--history-seconds",
        type=float,
        default=1.5,
        help="Time window used to look for the wave motion.",
    )
    parser.add_argument(
        "--max-vertical-range",
        type=float,
        default=0.18,
        help="Maximum vertical drift as a fraction of frame height allowed during a wave.",
    )
    parser.add_argument(
        "--min-wave-duration",
        type=float,
        default=0.35,
        help="Minimum time in seconds that the left/right motion should span.",
    )
    parser.add_argument(
        "--max-gap-seconds",
        type=float,
        default=0.35,
        help="Reset the tracked motion if the hand disappears longer than this.",
    )
    parser.add_argument(
        "--min-extended-fingers",
        type=int,
        default=3,
        help="Fingers (excluding thumb) that must be extended for an open-palm wave.",
    )
    parser.add_argument(
        "--detection-confidence",
        type=float,
        default=0.6,
        help="MediaPipe minimum hand detection confidence.",
    )
    parser.add_argument(
        "--tracking-confidence",
        type=float,
        default=0.5,
        help="MediaPipe minimum hand tracking confidence.",
    )
    return parser.parse_args()


def guess_serial_port() -> str | None:
    candidates: list[str] = []
    for pattern in (
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.wchusbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ):
        candidates.extend(sorted(glob.glob(pattern)))
    return candidates[0] if candidates else None


def ensure_model(model_path: pathlib.Path) -> pathlib.Path:
    if model_path.exists():
        return model_path

    model_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading hand landmarker model to {model_path} ...")
    urllib.request.urlretrieve(MODEL_URL, model_path)
    return model_path


def open_camera(index: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if capture.isOpened():
        return capture

    capture.release()
    capture = cv2.VideoCapture(index)
    return capture


def count_extended_fingers(landmarks) -> int:
    """Count non-thumb fingers whose tip is farther from the wrist than its PIP joint."""
    wrist = landmarks[WRIST]
    extended = 0
    for tip_index, pip_index in FINGER_TIP_PIP_PAIRS:
        tip = landmarks[tip_index]
        pip = landmarks[pip_index]
        tip_distance = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
        pip_distance = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
        if tip_distance > pip_distance:
            extended += 1
    return extended


def draw_hand(frame, landmarks) -> None:
    height, width = frame.shape[:2]
    points = [(int(lm.x * width), int(lm.y * height)) for lm in landmarks]
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, points[start], points[end], (0, 200, 255), 2)
    for px, py in points:
        cv2.circle(frame, (px, py), 4, (0, 255, 0), -1)


def main() -> int:
    args = parse_args()

    serial_port = None if args.dry_run else (
        guess_serial_port() if args.serial_port == "auto" else args.serial_port
    )
    if not args.dry_run and serial_port is None:
        print(
            "No Arduino-like serial port found. Connect the board or pass --serial-port /dev/...",
            file=sys.stderr,
        )
        return 2

    model_path = ensure_model(args.model_path)

    sender = SerialSignalSender(port=serial_port, baudrate=args.baudrate, dry_run=args.dry_run)
    detector = WaveDetector(
        min_amplitude=args.wave_amplitude,
        min_direction_step=args.direction_step,
        direction_changes_needed=args.direction_changes,
        history_seconds=args.history_seconds,
        max_vertical_range=args.max_vertical_range,
        min_wave_duration_seconds=args.min_wave_duration,
        max_gap_seconds=args.max_gap_seconds,
    )

    capture = open_camera(args.camera_index)
    if not capture.isOpened():
        print(f"Could not open camera index {args.camera_index}.", file=sys.stderr)
        sender.close()
        return 1

    landmarker_options = vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=args.detection_confidence,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=args.tracking_confidence,
    )
    landmarker = vision.HandLandmarker.create_from_options(landmarker_options)

    last_trigger_at = 0.0
    started_at = time.monotonic()

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("Camera frame could not be read.", file=sys.stderr)
                return 1

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            timestamp_ms = int((time.monotonic() - started_at) * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            now = time.time()
            palm_center: tuple[float, float] | None = None
            hand_landmarks = None
            open_palm = False

            if result.hand_landmarks:
                hand_landmarks = result.hand_landmarks[0]
                open_palm = count_extended_fingers(hand_landmarks) >= args.min_extended_fingers
                if open_palm:
                    palm = hand_landmarks[PALM_CENTER]
                    palm_center = (palm.x, palm.y)

            detected = detector.update(palm_center, now)
            just_triggered = False

            if detected and (now - last_trigger_at) >= args.cooldown_seconds:
                sender.send_wave()
                last_trigger_at = now
                just_triggered = True
                detector.reset()

            if just_triggered:
                status_text = "WAVE sent"
            elif detected:
                status_text = "wave detected"
            elif hand_landmarks is not None and not open_palm:
                status_text = "hand found, open your palm"
            elif hand_landmarks is not None:
                status_text = "hand found, wave it"
            else:
                status_text = "watching for a hand"

            if not args.no_preview:
                if hand_landmarks is not None:
                    draw_hand(frame, hand_landmarks)
                if palm_center is not None:
                    height, width = frame.shape[:2]
                    cx = int(palm_center[0] * width)
                    cy = int(palm_center[1] * height)
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)

                cv2.putText(
                    frame,
                    status_text,
                    (18, 34),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Wave To Arduino", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):
                    detector.reset()

    finally:
        capture.release()
        landmarker.close()
        sender.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
