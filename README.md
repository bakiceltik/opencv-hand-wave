# Wave To Arduino

This project watches your front camera and sends a `WAVE` message to an Arduino over serial when it sees you wave an open hand left and right.

Hand detection uses **MediaPipe Hand Landmarker**, so only a real hand can trigger it — faces, body movement, and background motion are ignored entirely. A wave is recognized when an open palm moves left/right with enough amplitude and at least a few direction changes.

## What was prepared on this machine

- A local Python 3.12 virtual environment at `.venv`
- A webcam detector script at `wave_to_arduino.py`
- The MediaPipe hand model at `models/hand_landmarker.task` (auto-downloaded if missing)
- An Arduino sketch at `arduino/wave_receiver/wave_receiver.ino`

## Install dependencies

```bash
python3.12 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

## Upload the Arduino sketch

1. Open `arduino/wave_receiver/wave_receiver.ino` in Arduino IDE.
2. Select your board and serial port.
3. Upload the sketch.

The sketch listens at `115200` baud and sets both the built-in LED and digital pin `7` high for one second whenever it receives `WAVE`.

## Find your Arduino serial port

```bash
ls /dev/cu.usb*
```

Typical examples on macOS are paths like `/dev/cu.usbmodem101` or `/dev/cu.usbserial-1410`.

## Run the detector

Dry run without Arduino:

```bash
./.venv/bin/python wave_to_arduino.py --dry-run
```

Run with Arduino:

```bash
./.venv/bin/python wave_to_arduino.py --serial-port /dev/cu.usbmodem101
```

If you omit `--serial-port`, the script tries to auto-detect a likely Arduino serial device.

The preview window shows the detected hand skeleton and a status line:

- `watching for a hand` — no hand in view
- `hand found, open your palm` — a hand is detected but the palm is not open
- `hand found, wave it` — open palm detected, waiting for the wave motion
- `WAVE sent` — signal sent to the Arduino

## Controls

- Press `q` to quit
- Press `r` to reset the tracked motion

## Tuning tips

All motion thresholds are fractions of the frame size, so they work at any camera resolution.

If it is too sensitive:

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
- The hand model is downloaded at `models/hand_landmarker.task`
- Detector logic verified with synthetic motion tests (wave detected; linear drift and vertical motion correctly rejected)
- Camera could not be tested from the automated session (macOS camera permission); run from your own terminal
- No Arduino serial device was visible when checked, so the end-to-end serial test still depends on connecting the board
