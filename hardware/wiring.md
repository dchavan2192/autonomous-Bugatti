# Wiring Guide

## Overview

```
Raspberry Pi 5
├── GPIO12 (hardware PWM) ──────────────────────────── MG996R Servo (signal)
├── GPIO pin 6 (GND) ──────── common ground ─────────── Zeus Shield GND
│                                                            │
│                                                        Arduino Uno R3
│                                                            │
├── USB-A ──────────────── USB-B ──────────────────────── Arduino Uno R3
│   (/dev/arduino → ttyUSB0)                                │
│                                                        Zeus Shield
│                                                        ├── M2 → DC Motor (left rear)
│                                                        └── M3 → DC Motor (right rear)
│
└── USB-A ──────────────── Logitech C925e (/dev/video0)

Hailo-8L AI HAT ────────────── PCIe (M.2 HAT) ──────────── Pi 5 PCIe slot
(/dev/hailo0)
```

---

## Common Ground

All subsystems share a common ground reference.

| From | To | Notes |
|------|----|-------|
| Raspberry Pi 5 — GPIO pin 6 (GND) | SunFounder Zeus Shield GND | Required for PWM signal integrity; Pi and Zeus must share GND |
| Zeus Shield GND | Arduino Uno GND | Zeus sits on top of Arduino; grounds are shared via shield headers |

---

## Servo — MG996R (Front Steering)

| Signal | Pi GPIO | Notes |
|--------|---------|-------|
| PWM signal | GPIO12 (BCM) | Hardware PWM via `/sys/class/pwm/pwmchip0/pwm0` sysfs interface |
| VCC | External 5 V supply | Do NOT power from Pi 5 V rail — servo draws too much current |
| GND | Common ground | Tie to Pi GPIO pin 6 and Zeus shield GND |

### Hardware PWM Setup (Pi 5 — Ubuntu)

The Pi 5 requires a custom device-tree overlay to expose hardware PWM on GPIO12.
The overlay source is `pwm-pi5-fix.dts` in the repo root.

```bash
# Compile and install overlay (one-time setup)
sudo dtc -I dts -O dtb -o /boot/firmware/overlays/pwm-pi5-fix.dtbo pwm-pi5-fix.dts

# Add to /boot/firmware/config.txt
dtoverlay=pwm-pi5-fix
```

After reboot, `/sys/class/pwm/pwmchip0` is available without root (servo_node runs as the `ros` user with appropriate group membership or udev rule).

### Pulse Width Mapping

| Servo position | Duty cycle |
|----------------|-----------|
| Full left | 850 µs |
| Straight | 1500 µs |
| Full right | 2050 µs |

---

## Arduino Uno R3 ↔ Raspberry Pi 5

| Interface | Detail |
|-----------|--------|
| Physical | USB-A (Pi) → USB-B (Arduino) |
| Baud rate | 115 200 |
| Pi device | `/dev/arduino` (udev alias) |

### udev Rule

Create `/etc/udev/rules.d/99-arduino.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino"
```

Reload rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Serial Protocol (Pi → Arduino)

Commands are newline-terminated ASCII strings sent from `serial_node`:

| Command | Effect |
|---------|--------|
| `FORWARD:<pwm>\r\n` | Drive motors forward at PWM value (0–255) |
| `BACKWARD:<pwm>\r\n` | Drive motors backward |
| `STOP\r\n` | Cut motor power |

### Serial Protocol (Arduino → Pi)

| Message | Meaning |
|---------|---------|
| `DIST:<cm>` | HC-SR04 sonar reading in centimetres. Value `0` means out of range (clear path), not 0 cm. |

---

## HC-SR04 Ultrasonic Sensor

The sonar is wired to the Arduino, not directly to the Pi.

| HC-SR04 pin | Arduino pin |
|-------------|-------------|
| Trig | Pin 10 (also used for echo — single-pin library) |
| Echo | Pin 10 |
| VCC | 5 V |
| GND | GND |

Readings are polled by the Arduino firmware and sent to the Pi as `DIST:<cm>` at ~20 Hz.

---

## DC Motors — SunFounder Zeus Shield

| Motor | Zeus channel | Physical location |
|-------|-------------|-------------------|
| Left rear | M2 | Left rear wheel |
| Right rear | M3 | Right rear wheel |

Motor speed is controlled by PWM values 0–255 sent in the `FORWARD:` / `BACKWARD:` serial command.

---

## Hailo-8L AI HAT

The Hailo HAT connects via the Pi 5's PCIe slot (M.2 HAT adapter). No wiring beyond the PCIe ribbon is needed.

| Item | Detail |
|------|--------|
| Interface | PCIe Gen2 x1 via M.2 HAT |
| Device node | `/dev/hailo0` |
| Driver | HailoRT 4.20.0 |
| Model used | `yolov8s.hef` (compiled for Hailo-8L) |

---

## Camera — Logitech C925e

Connected via USB-A. Appears as `/dev/video0` under V4L2.
`detection_node` opens it directly with `cv2.VideoCapture(0, cv2.CAP_V4L2)`.
