#!/usr/bin/env python3
"""Detect a hand wave and track a face from the webcam, driving an Arduino.

Hand detection uses MediaPipe Hand Landmarker, so only a real hand can send
the WAVE signal: faces, bodies, and background motion are never candidates.
A wave is an open palm moving left/right with enough amplitude and direction
changes.

Face tracking uses MediaPipe Face Detector: the horizontal position of the
largest face steers a pan servo. The script keeps the servo angle as state,
nudges it toward the face in small steps, and sends `ANG:<degrees>` lines to
the Arduino whenever the angle changes.
"""

from __future__ import annotations

import argparse
import collections
import glob
import math
import pathlib
import sys
import threading
import time
import urllib.error
import urllib.request
import urllib.parse
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

HAND_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
FACE_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/face_detector/"
    "blaze_face_short_range/float16/latest/blaze_face_short_range.tflite"
)
MODELS_DIR = pathlib.Path(__file__).resolve().parent / "models"
DEFAULT_HAND_MODEL_PATH = MODELS_DIR / "hand_landmarker.task"
DEFAULT_FACE_MODEL_PATH = MODELS_DIR / "blaze_face_short_range.tflite"


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


class PanController:
    """Nudges a servo angle toward the face center in small, smooth steps."""

    def __init__(
        self,
        *,
        start_angle: float,
        min_angle: float,
        max_angle: float,
        gain_deg: float,
        max_step_deg: float,
        deadband: float,
        smoothing: float,
        invert: bool,
    ) -> None:
        self.angle = float(start_angle)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.gain_deg = gain_deg
        self.max_step_deg = max_step_deg
        self.deadband = deadband
        self.smoothing = smoothing
        self.invert = invert
        self.smoothed_x: float | None = None
        self.last_sent_angle: int | None = None

    def update(self, face_center_x: float | None) -> int | None:
        """Feed the normalized face center x (0..1) or None; return a new angle to send."""
        if face_center_x is None:
            self.smoothed_x = None
            return None

        if self.smoothed_x is None:
            self.smoothed_x = face_center_x
        else:
            alpha = 1.0 - self.smoothing
            self.smoothed_x = self.smoothing * self.smoothed_x + alpha * face_center_x

        error = self.smoothed_x - 0.5
        if abs(error) < self.deadband:
            return None

        step = error * self.gain_deg
        step = max(-self.max_step_deg, min(self.max_step_deg, step))
        if self.invert:
            step = -step

        self.angle = max(self.min_angle, min(self.max_angle, self.angle + step))
        rounded = int(round(self.angle))
        if rounded == self.last_sent_angle:
            return None

        self.last_sent_angle = rounded
        return rounded


class SerialSignalSender:
    def __init__(self, *, port: str | None, baudrate: int, dry_run: bool) -> None:
        self.port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self.connection: serial.Serial | None = None

        if not dry_run and port is not None:
            try:
                self.connection = serial.Serial(port, baudrate=baudrate, timeout=1)
            except serial.SerialException as exc:
                available_ports = find_serial_port_candidates()
                available_text = ", ".join(available_ports) if available_ports else "none"
                raise SystemExit(
                    f"Could not open serial port {port}.\n"
                    f"Available ports: {available_text}\n"
                    "Tip: omit --serial-port to auto-detect the Arduino, and make sure "
                    "Arduino IDE Serial Monitor is closed."
                ) from exc
            print(f"[serial] connected to {port}, waiting for Arduino reset...")
            time.sleep(4.0)

    def send_line(self, message: str, *, quiet: bool = False) -> None:
        if self.dry_run or self.connection is None:
            if not quiet:
                print(f"[dry-run] {message}")
            return

        self.connection.write(f"{message}\n".encode("utf-8"))
        self.connection.flush()
        if not quiet:
            print(f"[serial] sent {message} to {self.port}")

    def send_wave(self) -> None:
        self.send_line("W")

    def send_middle_finger(self) -> None:
        self.send_line("MIDDLE_FINGER")

    def send_shoulder_up(self) -> None:
        self.send_line("SHOULDER_UP")

    def send_shoulder_down(self) -> None:
        self.send_line("SHOULDER_DOWN")

    def send_angle(self, angle: int) -> None:
        self.send_line(f"ANG:{angle}", quiet=True)

    def close(self) -> None:
        if self.connection is not None and self.connection.is_open:
            self.connection.close()


class HttpCommandSender:
    """Sends robot commands over WiFi to the ESP32-CAM's /cmd endpoint, which the
    ESP32-CAM forwards over its wired UART link to the Arduino -- no USB cable
    between the Arduino and this PC needed."""

    def __init__(self, *, base_url: str, dry_run: bool, timeout: float = 1.0) -> None:
        self.base_url = base_url
        self.dry_run = dry_run
        self.timeout = timeout

    def send_line(self, message: str, *, quiet: bool = False) -> None:
        if self.dry_run:
            if not quiet:
                print(f"[dry-run] {message}")
            return

        url = f"{self.base_url}/cmd?v={urllib.parse.quote(message)}"
        try:
            urllib.request.urlopen(url, timeout=self.timeout).read()
        except (urllib.error.URLError, OSError) as exc:
            print(f"[http] failed to send {message!r}: {exc}", file=sys.stderr)
            return

        if not quiet:
            print(f"[http] sent {message} to {self.base_url}")

    def send_wave(self) -> None:
        self.send_line("W")

    def send_middle_finger(self) -> None:
        self.send_line("MIDDLE_FINGER")

    def send_shoulder_up(self) -> None:
        self.send_line("SHOULDER_UP")

    def send_shoulder_down(self) -> None:
        self.send_line("SHOULDER_DOWN")

    def send_angle(self, angle: int) -> None:
        self.send_line(f"ANG:{angle}", quiet=True)

    def close(self) -> None:
        pass


class LatestFrameCapture:
    """Keeps only the newest frame so slow processing does not build visible lag."""

    def __init__(self, capture: cv2.VideoCapture) -> None:
        self.capture = capture
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.latest_frame = None
        self.frame_id = 0

    def start(self) -> None:
        self.thread.start()

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok:
                time.sleep(0.05)
                continue

            with self.condition:
                self.latest_frame = frame
                self.frame_id += 1
                self.condition.notify_all()

    def read(self, *, after_frame_id: int, timeout: float) -> tuple[int, object | None]:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.frame_id <= after_frame_id and not self.stop_event.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(timeout=remaining)

            if self.frame_id <= after_frame_id or self.latest_frame is None:
                return self.frame_id, None

            return self.frame_id, self.latest_frame.copy()

    def close(self) -> None:
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=1.0)
        self.capture.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect a hand wave (sends WAVE) and track the largest face "
            "(sends ANG:<degrees> for a pan servo) over serial to an Arduino."
        )
    )
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--camera-url",
        help=(
            "HTTP video stream URL, for example "
            "http://192.168.1.50:81/stream for an ESP32-CAM CameraWebServer."
        ),
    )
    parser.add_argument(
        "--serial-port",
        default="auto",
        help="Arduino serial device path, or 'auto' to guess one.",
    )
    parser.add_argument("--baudrate", type=int, default=9600, help="Serial baudrate (only used without --camera-url).")
    parser.add_argument(
        "--robot-port",
        type=int,
        default=80,
        help=(
            "ESP32-CAM HTTP port serving /cmd (camera_httpd in camera_web_server.ino). "
            "Only used with --camera-url: commands go over WiFi to the ESP32-CAM, which "
            "relays them over its wired UART link to the Arduino."
        ),
    )
    parser.add_argument("--dry-run", action="store_true", help="Run without sending serial data.")
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Disable the OpenCV preview window.",
    )
    mirror_group = parser.add_mutually_exclusive_group()
    mirror_group.add_argument(
        "--mirror",
        dest="mirror",
        action="store_true",
        help="Mirror the video horizontally before detection.",
    )
    mirror_group.add_argument(
        "--no-mirror",
        dest="mirror",
        action="store_false",
        help="Keep the video orientation unchanged.",
    )
    parser.set_defaults(mirror=None)
    parser.add_argument(
        "--hand-model-path",
        type=pathlib.Path,
        default=DEFAULT_HAND_MODEL_PATH,
        help="Path to the MediaPipe hand_landmarker.task model (downloaded if missing).",
    )
    parser.add_argument(
        "--face-model-path",
        type=pathlib.Path,
        default=DEFAULT_FACE_MODEL_PATH,
        help="Path to the MediaPipe blaze_face_short_range.tflite model (downloaded if missing).",
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
        "--middle-finger-cooldown-seconds",
        type=float,
        default=3.0,
        help="Minimum delay between MIDDLE_FINGER signals (triggers the shoulders forward/back move).",
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
    parser.add_argument(
        "--no-face-track",
        action="store_true",
        help="Disable face tracking and servo angle output.",
    )
    parser.add_argument(
        "--face-confidence",
        type=float,
        default=0.5,
        help="MediaPipe minimum face detection confidence.",
    )
    parser.add_argument(
        "--servo-start",
        type=int,
        default=90,
        help="Initial servo angle in degrees.",
    )
    parser.add_argument(
        "--servo-min",
        type=int,
        default=20,
        help="Lowest servo angle the tracker may command.",
    )
    parser.add_argument(
        "--servo-max",
        type=int,
        default=160,
        help="Highest servo angle the tracker may command.",
    )
    parser.add_argument(
        "--pan-gain",
        type=float,
        default=6.0,
        help="Degrees of correction per frame for a face at the frame edge.",
    )
    parser.add_argument(
        "--pan-max-step",
        type=float,
        default=2.5,
        help="Maximum servo movement in degrees per frame (keeps motion small).",
    )
    parser.add_argument(
        "--pan-deadband",
        type=float,
        default=0.06,
        help="No servo movement while the face is within this fraction of frame center.",
    )
    parser.add_argument(
        "--pan-smoothing",
        type=float,
        default=0.7,
        help="Face position smoothing factor 0..1 (higher = smoother, slower).",
    )
    parser.add_argument(
        "--invert-pan",
        action="store_true",
        help="Reverse the servo direction if the robot turns away from the face.",
    )
    parser.add_argument(
        "--angle-interval",
        type=float,
        default=0.05,
        help="Minimum seconds between ANG serial messages.",
    )
    return parser.parse_args()


def find_serial_port_candidates() -> list[str]:
    candidates: list[str] = []
    for pattern in (
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.wchusbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ):
        candidates.extend(sorted(glob.glob(pattern)))
    return candidates


def guess_serial_port() -> str | None:
    candidates = find_serial_port_candidates()
    return candidates[0] if candidates else None


def ensure_model(model_path: pathlib.Path, url: str) -> pathlib.Path:
    if model_path.exists():
        return model_path

    model_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading model to {model_path} ...")
    urllib.request.urlretrieve(url, model_path)
    return model_path


def normalize_camera_url(raw_url: str) -> str:
    value = raw_url.strip()
    if not value:
        raise ValueError("Camera URL cannot be empty.")

    if "://" not in value:
        value = f"http://{value}"

    parsed = urllib.parse.urlparse(value)
    if not parsed.netloc:
        raise ValueError(
            "Camera URL must include a host, for example 192.168.1.50 or esp32cam.local."
        )

    if parsed.path not in ("", "/"):
        return value

    host = parsed.hostname or ""
    if not host:
        raise ValueError("Camera URL host could not be parsed.")

    port = parsed.port if parsed.port is not None else 81
    netloc = host if port in (80, 443) and parsed.port is None else f"{host}:{port}"
    return urllib.parse.urlunparse(
        (parsed.scheme or "http", netloc, "/stream", "", "", "")
    )


def derive_control_url(raw_url: str, port: int) -> str:
    """Build the ESP32-CAM's main HTTP server URL (serves /cmd, /capture, ...) from the
    same host the user gave for the video stream. Defaults to port 80, the camera_httpd
    port in camera_web_server.ino -- the :81 stream server is a separate one."""
    value = raw_url.strip()
    if "://" not in value:
        value = f"http://{value}"

    parsed = urllib.parse.urlparse(value)
    host = parsed.hostname
    if not host:
        raise ValueError("Camera URL host could not be parsed.")

    netloc = host if port == 80 else f"{host}:{port}"
    return urllib.parse.urlunparse((parsed.scheme or "http", netloc, "", "", "", ""))


def open_camera(index: int, camera_url: str | None) -> cv2.VideoCapture:
    if camera_url is not None:
        capture = cv2.VideoCapture(camera_url)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    capture = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if capture.isOpened():
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    capture.release()
    capture = cv2.VideoCapture(index)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def finger_extended(landmarks, tip_index: int, pip_index: int, wrist) -> bool:
    """A finger is extended if its tip is farther from the wrist than its PIP joint."""
    tip = landmarks[tip_index]
    pip = landmarks[pip_index]
    tip_distance = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
    pip_distance = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
    return tip_distance > pip_distance


def count_extended_fingers(landmarks) -> int:
    """Count non-thumb fingers whose tip is farther from the wrist than its PIP joint."""
    wrist = landmarks[WRIST]
    return sum(
        finger_extended(landmarks, tip_index, pip_index, wrist)
        for tip_index, pip_index in FINGER_TIP_PIP_PAIRS
    )


def is_fist(landmarks) -> bool:
    """All non-thumb fingers folded."""
    return count_extended_fingers(landmarks) == 0


def is_middle_finger_gesture(landmarks) -> bool:
    """Only the middle finger extended, index/ring/pinky folded."""
    wrist = landmarks[WRIST]
    (index_tip, index_pip), (middle_tip, middle_pip), (ring_tip, ring_pip), (pinky_tip, pinky_pip) = (
        FINGER_TIP_PIP_PAIRS
    )
    return (
        finger_extended(landmarks, middle_tip, middle_pip, wrist)
        and not finger_extended(landmarks, index_tip, index_pip, wrist)
        and not finger_extended(landmarks, ring_tip, ring_pip, wrist)
        and not finger_extended(landmarks, pinky_tip, pinky_pip, wrist)
    )


def largest_face_bbox(detections) -> tuple[int, int, int, int] | None:
    """Return (x, y, w, h) in pixels for the biggest detected face."""
    best = None
    best_area = 0.0
    for detection in detections:
        box = detection.bounding_box
        area = float(box.width * box.height)
        if area > best_area:
            best_area = area
            best = (box.origin_x, box.origin_y, box.width, box.height)
    return best


def draw_hand(frame, landmarks) -> None:
    height, width = frame.shape[:2]
    points = [(int(lm.x * width), int(lm.y * height)) for lm in landmarks]
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, points[start], points[end], (0, 200, 255), 2)
    for px, py in points:
        cv2.circle(frame, (px, py), 4, (0, 255, 0), -1)


def main() -> int:
    args = parse_args()
    camera_source = None
    if args.camera_url is not None:
        try:
            camera_source = normalize_camera_url(args.camera_url)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
    mirror_frames = args.mirror if args.mirror is not None else camera_source is None

    hand_model_path = ensure_model(args.hand_model_path, HAND_MODEL_URL)
    face_model_path = None
    if not args.no_face_track:
        face_model_path = ensure_model(args.face_model_path, FACE_MODEL_URL)

    if args.camera_url is not None:
        # ESP32-CAM is the video source, so it's also the wired bridge to the Arduino:
        # commands go out over WiFi to its /cmd endpoint instead of a USB cable.
        try:
            control_url = derive_control_url(args.camera_url, args.robot_port)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        sender = HttpCommandSender(base_url=control_url, dry_run=args.dry_run)
    else:
        serial_port = None if args.dry_run else (
            guess_serial_port() if args.serial_port == "auto" else args.serial_port
        )
        if not args.dry_run and serial_port is None:
            print(
                "No Arduino-like serial port found. Connect the board or pass --serial-port /dev/...",
                file=sys.stderr,
            )
            return 2
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
    pan = PanController(
        start_angle=args.servo_start,
        min_angle=args.servo_min,
        max_angle=args.servo_max,
        gain_deg=args.pan_gain,
        max_step_deg=args.pan_max_step,
        deadband=args.pan_deadband,
        smoothing=args.pan_smoothing,
        invert=args.invert_pan,
    )

    capture = open_camera(args.camera_index, camera_source)
    if not capture.isOpened():
        if camera_source is not None:
            print(f"Could not open camera stream {camera_source}.", file=sys.stderr)
        else:
            print(f"Could not open camera index {args.camera_index}.", file=sys.stderr)
        sender.close()
        return 1

    if camera_source is not None:
        print(f"[camera] opened stream {camera_source}")
    else:
        print(f"[camera] opened local camera index {args.camera_index}")

    frame_source = LatestFrameCapture(capture)
    frame_source.start()

    hand_landmarker = vision.HandLandmarker.create_from_options(
        vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(hand_model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=args.detection_confidence,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=args.tracking_confidence,
        )
    )
    face_detector = None
    if face_model_path is not None:
        face_detector = vision.FaceDetector.create_from_options(
            vision.FaceDetectorOptions(
                base_options=mp_python.BaseOptions(model_asset_path=str(face_model_path)),
                running_mode=vision.RunningMode.VIDEO,
                min_detection_confidence=args.face_confidence,
            )
        )

    sender.send_angle(args.servo_start)
    last_trigger_at = 0.0
    last_middle_finger_at = 0.0
    middle_finger_armed = True
    fist_active = False
    last_angle_sent_at = 0.0
    started_at = time.monotonic()
    last_frame_id = 0

    try:
        while True:
            last_frame_id, frame = frame_source.read(after_frame_id=last_frame_id, timeout=5.0)
            if frame is None:
                print("Camera frame could not be read.", file=sys.stderr)
                return 1

            if mirror_frames:
                frame = cv2.flip(frame, 1)
            height, width = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            timestamp_ms = int((time.monotonic() - started_at) * 1000)

            hand_result = hand_landmarker.detect_for_video(mp_image, timestamp_ms)
            face_result = (
                face_detector.detect_for_video(mp_image, timestamp_ms)
                if face_detector is not None
                else None
            )

            now = time.time()

            # --- hand wave ---
            palm_center: tuple[float, float] | None = None
            hand_landmarks = None
            open_palm = False

            if hand_result.hand_landmarks:
                hand_landmarks = hand_result.hand_landmarks[0]
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

            # --- middle finger gesture ---
            middle_finger_gesture = (
                hand_landmarks is not None and is_middle_finger_gesture(hand_landmarks)
            )
            just_triggered_middle_finger = False

            if not middle_finger_gesture:
                middle_finger_armed = True
            elif middle_finger_armed and (
                now - last_middle_finger_at
            ) >= args.middle_finger_cooldown_seconds:
                sender.send_middle_finger()
                last_middle_finger_at = now
                middle_finger_armed = False
                just_triggered_middle_finger = True
                detector.reset()

            # --- fist -> raise/lower right shoulder ---
            fist_detected = hand_landmarks is not None and is_fist(hand_landmarks)
            if fist_detected and not fist_active:
                sender.send_shoulder_up()
                fist_active = True
            elif not fist_detected and fist_active:
                sender.send_shoulder_down()
                fist_active = False

            # --- face pan ---
            face_bbox = None
            if face_result is not None:
                face_bbox = largest_face_bbox(face_result.detections)

            face_center_x = None
            if face_bbox is not None:
                fx, fy, fw, fh = face_bbox
                face_center_x = (fx + fw / 2.0) / width

            new_angle = pan.update(face_center_x)
            if new_angle is not None and (now - last_angle_sent_at) >= args.angle_interval:
                sender.send_angle(new_angle)
                last_angle_sent_at = now

            # --- status / preview ---
            if just_triggered_middle_finger:
                status_text = "middle finger detected, shoulders moving"
            elif middle_finger_gesture:
                status_text = "middle finger gesture held"
            elif fist_active:
                status_text = "fist held, right shoulder up"
            elif just_triggered:
                status_text = "WAVE sent"
            elif detected:
                status_text = "wave detected"
            elif hand_landmarks is not None and not open_palm:
                status_text = "hand found, open your palm"
            elif hand_landmarks is not None:
                status_text = "hand found, wave it"
            elif face_bbox is not None:
                status_text = "tracking face"
            else:
                status_text = "watching"

            if not args.no_preview:
                if hand_landmarks is not None:
                    draw_hand(frame, hand_landmarks)
                if palm_center is not None:
                    cx = int(palm_center[0] * width)
                    cy = int(palm_center[1] * height)
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                if face_bbox is not None:
                    fx, fy, fw, fh = face_bbox
                    cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (255, 120, 0), 2)

                if face_detector is not None:
                    angle_text = f"servo {pan.last_sent_angle or args.servo_start} deg"
                    cv2.putText(
                        frame,
                        angle_text,
                        (18, height - 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 200, 0),
                        2,
                        cv2.LINE_AA,
                    )

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
        frame_source.close()
        hand_landmarker.close()
        if face_detector is not None:
            face_detector.close()
        sender.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
