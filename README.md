# Wave To Arduino

This project watches a camera feed and talks to an Arduino over serial:

- **Hand wave → `WAVE`**: when you wave an open hand left/right, the Arduino triggers the robot's `elSalla()` motion.
- **Optional face tracking → `ANG:<degrees>`**: the largest face in view can steer the robot body servo on pin `D3`, so the robot keeps "looking at" the person.

Hand detection uses **MediaPipe Hand Landmarker**, so only a real hand can trigger the wave — faces, body movement, and background motion are ignored entirely. A wave is recognized when an open palm moves left/right with enough amplitude and at least a few direction changes.

Face tracking uses **MediaPipe Face Detector**. The script keeps the servo angle as state and nudges it a couple of degrees per frame toward the face, with a deadband around frame center so the servo stays still when the person is roughly centered.

For ESP32-CAM over Wi-Fi, the most reliable starting point is **wave-only mode**. Face tracking is still available, but it adds enough per-frame work that slower or laggier streams can feel choppy.

## What was prepared on this machine

- A local Python 3.12 virtual environment at `.venv`
- A detector script at `wave_to_arduino.py`
- MediaPipe models at `models/` (hand + face, auto-downloaded if missing)
- An Arduino sketch at `arduino/wave_receiver/wave_receiver.ino`

## Install dependencies

```bash
python3.12 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

## Robot wiring

The Arduino sketch in this repo now matches the shield/pin layout from the robot code:

- `D3` → `govdeServo`
- `D5` → `dengeServo`
- `D6` → `solOmuz`
- `D9` → `sagOmuz`
- `D10` → `solKol`
- `D11` → `sagKol`
- `A0` → joystick `X`
- `D2` → joystick button `SW`
- `D7` → optional feedback output that goes high during `WAVE`

Power notes:

- Servo power should come from a supply that can handle all servos together.
- If you use an external 5V supply for the shield, the external supply `GND` and Arduino `GND` must be common.
- Keep the USB cable connected between the computer and Arduino, because Python sends commands over that USB serial link.

## Upload the Arduino sketch

1. Open `arduino/wave_receiver/wave_receiver.ino` in Arduino IDE.
2. Select your board and serial port.
3. Upload the sketch.

The sketch listens at `9600` baud and understands these line-based commands:

- `WAVE` — runs `elSalla()` and raises the built-in LED plus pin `7` for one second
- `ANG:<n>` — moves `govdeServo` on `D3` to `<n>` degrees (clamped to 20–160)

Optional manual serial commands:

- `LEFT_PUNCH`
- `RIGHT_PUNCH`
- `COME`

On boot the robot moves to its ready pose, keeps `dengeServo` on `D5` at the configured balance angle, and keeps joystick control active:

- joystick button → `elSalla()`
- joystick left → `solYumrukAt()`
- joystick right → `sagYumrukAt()`

## Find your Arduino serial port

```bash
ls /dev/cu.usb*
```

Typical examples on macOS are paths like `/dev/cu.usbmodem101` or `/dev/cu.usbserial-1410`.

## ESP32-CAM as the camera source

If you want to use an ESP32-CAM instead of the Mac webcam:

1. Flash the ESP32 `CameraWebServer` sketch to the ESP32-CAM.
2. Set `CAMERA_MODEL_AI_THINKER`.
3. Enter your Wi-Fi name and password.
4. Open Serial Monitor and note the ESP32-CAM IP address.
5. Keep the ESP32-CAM and this Mac on the same Wi-Fi network.

Important: the USB-C programmer board is only for power/programming/serial. The video feed reaches this Python script over Wi-Fi from the ESP32-CAM web server.

## Run the detector

Dry run without Arduino:

```bash
./.venv/bin/python wave_to_arduino.py --dry-run
```

Run with Arduino:

```bash
./.venv/bin/python wave_to_arduino.py --serial-port /dev/cu.usbserial-2130
```

If you omit `--serial-port`, the script tries to auto-detect a likely Arduino serial device.

Recommended first run with an ESP32-CAM stream: wave-only mode.

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.123 --serial-port /dev/cu.usbserial-2130 --no-face-track
```

If you want the lightest runtime while tuning detection, also disable the preview window:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.123 --serial-port /dev/cu.usbserial-2130 --no-face-track --no-preview
```

General ESP32-CAM stream example:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url http://192.168.1.50:81/stream --serial-port /dev/cu.usbserial-2130
```

You can also pass just the IP or hostname and the script will expand it to the usual ESP32-CAM stream path:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --serial-port /dev/cu.usbserial-2130
```

Mirror behavior is automatic:

- local webcam input is mirrored by default
- network streams such as ESP32-CAM are not mirrored by default

If you need to override that:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --mirror
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --no-mirror
```

The preview window shows the detected hand skeleton, the face box, the current body angle, and a status line:

- `watching` — nothing in view
- `tracking face` — a face is steering the servo
- `hand found, open your palm` — a hand is detected but the palm is not open
- `hand found, wave it` — open palm detected, waiting for the wave motion
- `WAVE sent` — signal sent to the Arduino

## Controls

- Press `q` to quit
- Press `r` to reset the tracked motion

## Tuning tips

All motion thresholds are fractions of the frame size, so they work at any camera resolution.

If the ESP32-CAM feed feels laggy or choppy, simplify first:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --serial-port /dev/cu.usbserial-2130 --no-face-track
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --serial-port /dev/cu.usbserial-2130 --no-face-track --no-preview
```

If the robot body turns **away** from your face instead of toward it (depends on how the servo is mounted):

```bash
./.venv/bin/python wave_to_arduino.py --invert-pan
```

If the body servo jitters or moves too eagerly:

```bash
./.venv/bin/python wave_to_arduino.py --pan-deadband 0.10 --pan-max-step 1.5 --pan-smoothing 0.85
```

If the body follows too slowly:

```bash
./.venv/bin/python wave_to_arduino.py --pan-gain 9 --pan-max-step 4 --pan-deadband 0.04
```

To disable face tracking and keep only the wave signal:

```bash
./.venv/bin/python wave_to_arduino.py --no-face-track
```

If the wave is too sensitive:

```bash
./.venv/bin/python wave_to_arduino.py --dry-run --wave-amplitude 0.12 --direction-changes 4
```

If it misses your hand wave:

```bash
./.venv/bin/python wave_to_arduino.py --dry-run --wave-amplitude 0.06 --direction-changes 2 --max-vertical-range 0.25
```

If your hand is not being detected at all (poor lighting, far away):

```bash
./.venv/bin/python wave_to_arduino.py --dry-run --detection-confidence 0.4 --min-extended-fingers 2
```

## Current machine status on 2026-06-12

- `python3.12` is available, `.venv` has all dependencies installed
- Both models are downloaded under `models/`
- Wave detector verified with synthetic motion tests (wave detected; linear drift and vertical motion correctly rejected)
- Pan controller verified with synthetic face positions (deadband holds still, small steps toward the face, clamps at servo limits, holds angle when the face disappears)
- Camera could not be tested from the automated session (macOS camera permission); run from your own terminal
- No Arduino serial device was visible when checked, so the end-to-end serial and servo test still depends on connecting the board

## How to load this onto the robot

1. Open [arduino/wave_receiver/wave_receiver.ino](/Users/bakiceltik/Documents/openvm/arduino/wave_receiver/wave_receiver.ino:1) in Arduino IDE.
2. Select the correct board and the USB serial port for the robot.
3. Upload the sketch.
4. If you are using ESP32-CAM, flash `CameraWebServer` to that board too and note its IP from Serial Monitor.
5. Close Arduino IDE's Serial Monitor if it is open. Only one app can use the robot serial port at a time.
6. Find the robot serial device on macOS:

```bash
ls /dev/cu.usb*
```

7. Start the Python side from the repo root.

For the Mac webcam:

```bash
./.venv/bin/python wave_to_arduino.py --serial-port /dev/cu.usbserial-2130
```

For ESP32-CAM wave-only mode:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --serial-port /dev/cu.usbserial-2130 --no-face-track
```

For ESP32-CAM with face tracking enabled:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --serial-port /dev/cu.usbserial-2130
```

If auto-detection works on your machine, this also works:

```bash
./.venv/bin/python wave_to_arduino.py
```

What happens after upload:

- waving your real hand at the camera sends `WAVE`, which runs `elSalla()`
- optional face tracking sends `ANG:<n>`, which rotates `govdeServo` on `D3`
- joystick still works locally on the robot even while the Python script is connected

If you want to test from Arduino IDE Serial Monitor instead of Python:

- set baud rate to `9600`
- choose `Newline` or `Both NL & CR`
- send `WAVE` or `ANG:90`
