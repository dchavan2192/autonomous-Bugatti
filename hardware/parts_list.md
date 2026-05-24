# Parts List

Complete bill of materials for the autonomous RC car.

## Compute

| Component | Spec | Notes |
|-----------|------|-------|
| Raspberry Pi 5 | 8 GB RAM | Main compute board, runs ROS2 Jazzy |
| Hailo-8L AI HAT | 13 TOPS, PCIe Gen2 | Neural network accelerator, exposed as `/dev/hailo0` |

## Chassis / Motors

| Component | Spec | Notes |
|-----------|------|-------|
| Lego Technic Bugatti (42083) | — | Chassis donor; drivetrain stripped and re-motored |
| Arduino Uno R3 | ATmega328P | Motor controller, sonar reader, serial bridge to Pi |
| SunFounder Zeus Motor Shield | Dual L298N H-bridge | Sits on top of Arduino; drives M2 and M3 motor channels |
| 2x TT DC Gear Motor | 3–6 V, ~200 RPM at 5 V | Rear-wheel drive via Zeus shield channels M2 and M3 |

## Steering

| Component | Spec | Notes |
|-----------|------|-------|
| MG996R Servo | 55 g·cm torque, 4.8–7.2 V | Front axle steering, driven by Pi GPIO12 hardware PWM |
| Custom DTS overlay | `pwm-pi5-fix.dtbo` | Enables hardware PWM on GPIO12 (BCM), required on Pi 5 |

### Servo Calibration

| Position | Pulse Width |
|----------|-------------|
| Full Left | 850 µs |
| Center (straight) | 1500 µs |
| Full Right | 2050 µs |
| Period | 20 000 µs (50 Hz) |

## Sensing

| Component | Spec | Notes |
|-----------|------|-------|
| HC-SR04 Ultrasonic Sensor | 2–400 cm range, ±3 mm | Connected to Arduino pin 10; readings sent to Pi over serial as `DIST:<cm>` |
| Logitech C925e Webcam | 1080p, USB 2.0 | Object detection input via V4L2 (`/dev/video0`) at 30 fps |

## Communication / Interface

| Component | Spec | Notes |
|-----------|------|-------|
| USB-A to USB-B cable | — | Pi ↔ Arduino serial at 115200 baud; udev rule aliases to `/dev/arduino` |
