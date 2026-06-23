# Wave To Arduino

This project watches a camera feed and talks to an Arduino:

- **Hand wave → `WAVE`**: when you wave an open hand left/right, the Arduino triggers the robot's `elSalla()` motion.
- **Optional face tracking → `ANG:<degrees>`**: the largest face in view can steer the robot body servo on pin `D3`, so the robot keeps "looking at" the person.

**Wireless bridge (current setup):** when the camera source is an ESP32-CAM (`--camera-url`), the Python script no longer talks to the Arduino over USB at all. It sends commands over Wi-Fi to the ESP32-CAM's `/cmd` endpoint, and the ESP32-CAM forwards them over a short wired UART link straight into the Arduino. The Arduino only needs USB for flashing — once both boards are flashed and wired, the whole rig runs off the ESP32-CAM's Wi-Fi connection and the Arduino's own power supply, no cable back to the computer. See [ESP32-CAM ↔ Arduino wired bridge](#esp32-cam--arduino-wired-bridge) below.

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
- `D12` → HC-SR04 `TRIG`
- `D8` → HC-SR04 `ECHO`
- `A0` → joystick `X`
- `D2` → joystick button `SW`
- `D7` → optional feedback output that goes high during `WAVE`
- `D4` → wired RX from the ESP32-CAM's UART0 TX (see [ESP32-CAM ↔ Arduino wired bridge](#esp32-cam--arduino-wired-bridge))

Power notes:

- Servo power should come from a supply that can handle all servos together.
- If you use an external 5V supply for the shield, the external supply `GND` and Arduino `GND` must be common.
- The Arduino's USB cable is only needed to flash the sketch, not while running — commands now arrive over the ESP32-CAM wire, see below.
- **ESP32-CAM brownouts:** if the ESP32-CAM drops off Wi-Fi specifically when its `GND` wire to the Arduino/shield is connected, that's the shield's motor current sagging the shared power rail. Add a 100–470µF capacitor across the ESP32-CAM's `5V`/`GND` pins, or give the ESP32-CAM its own isolated supply (single-point ground only). Don't "fix" this by removing the `GND` wire — without a common ground the UART link won't work at all.

## ESP32-CAM ↔ Arduino wired bridge

This is what makes the Arduino USB-free at runtime. Two wires, no level shifter needed (ESP32-CAM TX is 3.3V, comfortably above the Arduino's ~3V input-high threshold):

- ESP32-CAM `GPIO1` (labeled `U0T` / `TX`) → Arduino `D4`
- ESP32-CAM `GND` → Arduino `GND` (required — without a common ground the link reads garbage, not just "no signal")

The Arduino never talks back to the ESP32-CAM, so the other half of the UART (`GPIO3`/`U0R` on the ESP32-CAM) is left unconnected.

Why `D4` and not an analog pin: it was tried on `A1` first, which never reliably received anything even though the ESP32-CAM side was confirmed (with a separate USB-serial probe) to be sending the right bytes — the motor shield likely blocks or doesn't pass through that analog header pin on this board. `D4` is a plain free digital pin and works.

How it works in firmware:

- `arduino/camera_web_server/app_httpd.cpp` adds a `/cmd?v=<command>` HTTP endpoint. A GET request to it writes `<command>` followed by a newline out over the ESP32-CAM's hardware UART0 (the same pins used for flashing/debug).
- `arduino/wave_receiver/wave_receiver.ino` listens on a `SoftwareSerial` on `D4` (see `ESP_RX_PIN`) in addition to the USB serial, so commands from either source are accepted. The single-character `W` shortcut (typing just `W` + Enter in a Serial Monitor) only works over USB — over the ESP32-CAM link it's disabled on purpose, because the ESP32-CAM's own Wi-Fi boot log can start with `W` (`"WiFi connecting..."`) and would otherwise misfire as a wave command.
- Both ends must run the same baud rate: `Serial.begin(9600)` in `camera_web_server.ino` and `espSerial.begin(9600)` (via `ESP_SERIAL_BAUDRATE`) in `wave_receiver.ino`. If you change one, change the other.
- `wave_to_arduino.py`'s `HttpCommandSender` sends to `http://<esp32-cam-ip>/cmd?v=<command>` (port 80, the camera's main control server — not the `:81` stream port). This kicks in automatically whenever `--camera-url` is given; pass `--robot-port` to override the port if you changed it in the sketch.

## Upload the Arduino sketch

1. Open `arduino/wave_receiver/wave_receiver.ino` in Arduino IDE (or `arduino-cli compile --fqbn arduino:avr:uno ./arduino/wave_receiver`).
2. Select your board and serial port.
3. Upload the sketch. The Arduino's USB cable can be unplugged again once this is done — it's not needed while running.

The sketch listens at `9600` baud and understands these line-based commands:

- `WAVE` — runs `elSalla()` and raises the built-in LED plus pin `7` for one second
- `ANG:<n>` — moves `govdeServo` on `D3` to `<n>` degrees (clamped to 20–160)

Optional manual serial commands:

- `LEFT_PUNCH`
- `RIGHT_PUNCH`
- `COME`

On boot the robot moves to its ready pose, keeps `dengeServo` on `D5` at the configured balance angle, and keeps joystick control active:

- joystick button → `gelGelIsareti()`
- joystick left → `solYumrukAt()`
- joystick right → `sagYumrukAt()`

HC-SR04 distance behavior in the current sketch:

- farther than `40 cm` → ready / waiting pose
- between `20 cm` and `40 cm` → guard pose
- closer than `20 cm` → one punch, alternating left and right each time, then returns to guard until the target moves away again

To watch the sensor live in Arduino IDE Serial Monitor, set baud to `9600`. The sketch prints lines like:

- `DIST_CM:48 STATE:BEKLE`
- `DIST_CM:27 STATE:GARD`
- `DIST:YUMRUK`

## Find your Arduino serial port

```bash
ls /dev/cu.usb*
```

Typical examples on macOS are paths like `/dev/cu.usbmodem101` or `/dev/cu.usbserial-1410`.

## ESP32-CAM as the camera source

If you want to use an ESP32-CAM instead of the Mac webcam:

1. Flash `arduino/camera_web_server/camera_web_server.ino` to the ESP32-CAM (`CAMERA_MODEL_AI_THINKER` is already selected in `board_config.h`).
2. Set your Wi-Fi name and password at the top of `camera_web_server.ino`.
3. To flash: wire a USB-serial programmer (FTDI or similar) to the ESP32-CAM (`TX`↔`RX`, `RX`↔`TX`, `GND`↔`GND`, `5V`↔`5V`), bridge `GPIO0` to `GND` to force flash mode, upload, then remove the `GPIO0`↔`GND` bridge and reset so it boots normally.
4. Open a serial monitor at `9600` baud (this sketch's baud, not the IDE default `115200`) and note the `WiFi connected` / IP line.
5. Keep the ESP32-CAM and this Mac on the same Wi-Fi network. The IP can change between boots if your router uses DHCP — recheck it if the script can't connect.
6. Wire the ESP32-CAM to the Arduino as described in [ESP32-CAM ↔ Arduino wired bridge](#esp32-cam--arduino-wired-bridge) so commands can reach the robot.

The video feed reaches this Python script over Wi-Fi from the ESP32-CAM's web server (`:81/stream`); commands go back the same way, over Wi-Fi, to the ESP32-CAM's `/cmd` endpoint (`:80`), which then relays them over the wired bridge to the Arduino.

## Run the detector

Dry run without Arduino:

```bash
./.venv/bin/python wave_to_arduino.py --dry-run
```

Run with the Mac webcam talking to an Arduino over USB (no ESP32-CAM, no wired bridge):

```bash
./.venv/bin/python wave_to_arduino.py --serial-port /dev/cu.usbserial-2130
```

If you omit `--serial-port`, the script tries to auto-detect a likely Arduino serial device. This `--serial-port` path is only used when `--camera-url` is **not** given.

Recommended first run with an ESP32-CAM stream: wave-only mode. `--serial-port` is not needed here — commands go over Wi-Fi to the ESP32-CAM's `/cmd` endpoint, then over the wired bridge to the Arduino.

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.123 --no-face-track
```

If you want the lightest runtime while tuning detection, also disable the preview window:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.123 --no-face-track --no-preview
```

General ESP32-CAM stream example:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url http://192.168.1.50:81/stream
```

You can also pass just the IP or hostname and the script will expand it to the usual ESP32-CAM stream path:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50
```

If you changed the ESP32-CAM's main HTTP port away from the default `80` in `camera_web_server.ino`, point the command channel at it with `--robot-port`:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --robot-port 8080
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
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --no-face-track
./.venv/bin/python wave_to_arduino.py --camera-url 192.168.1.50 --no-face-track --no-preview
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

## How to load this onto the robot (ESP32-CAM bridge, no Arduino USB at runtime)

1. Flash `arduino/wave_receiver/wave_receiver.ino` to the Arduino over USB (`arduino-cli upload -p <port> --fqbn arduino:avr:uno ./arduino/wave_receiver`, or Arduino IDE).
2. Flash `arduino/camera_web_server/camera_web_server.ino` to the ESP32-CAM (needs a USB-serial programmer + the `GPIO0`↔`GND` flash-mode jumper — see [ESP32-CAM as the camera source](#esp32-cam-as-the-camera-source)).
3. Wire the two boards together: ESP32-CAM `GPIO1`(`TX`) → Arduino `D4`, and a common `GND` between them. Full details in [ESP32-CAM ↔ Arduino wired bridge](#esp32-cam--arduino-wired-bridge).
4. Power both boards (the Arduino/shield can keep its own supply; if it shares one with the ESP32-CAM, see the brownout note above). The Arduino's USB cable can come out now — it was only needed for step 1.
5. Find the ESP32-CAM's IP: either watch its boot serial output at `9600` baud through a USB-serial programmer, or check your router's DHCP client list.
6. Start the Python side from the repo root, on the same Wi-Fi network as the ESP32-CAM:

```bash
./.venv/bin/python wave_to_arduino.py --camera-url <esp32-cam-ip> --no-face-track
```

(Drop `--no-face-track` once wave detection is confirmed working, to also enable face-tracking pan.)

What happens after this:

- waving your real hand at the camera sends `W` over Wi-Fi to the ESP32-CAM's `/cmd` endpoint, which writes it out its wired UART to the Arduino, which runs `elSalla()`
- optional face tracking sends `ANG:<n>` the same way, rotating `govdeServo` on `D3`
- joystick still works locally on the robot even while the Python script is connected

To instead run the Mac webcam straight to an Arduino over USB (the older, non-wireless path — no ESP32-CAM involved):

```bash
./.venv/bin/python wave_to_arduino.py --serial-port /dev/cu.usbserial-2130
```

If auto-detection works on your machine, this also works:

```bash
./.venv/bin/python wave_to_arduino.py
```

If you want to test the Arduino side directly instead of through Python:

- over USB in Arduino IDE Serial Monitor: baud `9600`, line ending `Newline` or `Both NL & CR`, send `WAVE` or `ANG:90`
- over the ESP32-CAM bridge: `curl "http://<esp32-cam-ip>/cmd?v=W"` or `curl "http://<esp32-cam-ip>/cmd?v=ANG:90"`
