# Autonomous RC Car — Raspberry Pi 5 + ROS2 Jazzy + Behavior Cloning

An indoor autonomous RC car built on a Lego Technic Bugatti (42083) chassis, running a full ROS2 Jazzy software stack on a Raspberry Pi 5. The car can navigate autonomously using behavior cloning, perform object detection at 20 fps on a dedicated Hailo-8L AI accelerator, and self-park using a vision-based OpenCV detector and a learned steering policy.

<!-- Demo video coming soon -->

---

## Overview

This project started as a hardware bring-up exercise and grew into a full autonomous driving pipeline — from raw sensor data all the way to a trained neural network making real-time steering decisions on embedded hardware.

Key capabilities:
- **Behavior cloning**: Drive the car manually with WASD teleop, record sensor/action pairs, train a multi-layer perceptron with scikit-learn, and deploy the policy at 20 Hz without any simulator or cloud compute.
- **Real-time object detection**: YOLOv8s running on the Hailo-8L AI HAT via PCIe — ~20 fps with the Pi 5 CPU free for everything else.
- **Self-parking**: OpenCV black-tape detection feeds a second behavior-cloning policy that steers the car through a four-phase park sequence (searching → aligning → parking → parked).
- **LLM navigation**: Optional Ollama-backed mode queries a local `qwen2.5:1.5b` model every 8 seconds for high-level navigation decisions, with a hard-coded safety state machine overriding any dangerous commands.
- **Layered safety**: Every autonomous mode has an independent sonar-based safety layer that stops, reverses, and re-routes when an obstacle is detected within 35 cm.

---

## Hardware

| Component | Spec | Purpose |
|-----------|------|---------|
| Raspberry Pi 5 | 8 GB RAM | Main compute — runs all ROS2 nodes |
| Hailo-8L AI HAT | 13 TOPS, PCIe Gen2, `/dev/hailo0` | YOLOv8s inference at ~20 fps |
| Logitech C925e Webcam | 1080p, USB 2.0, `/dev/video0` | Camera input for detection and parking |
| Arduino Uno R3 | ATmega328P | Motor controller and sonar reader; serial bridge at 115200 baud |
| SunFounder Zeus Motor Shield | Dual L298N H-bridge | DC motor drive on channels M2 (left) and M3 (right) |
| 2x TT DC Gear Motor | 3–6 V | Rear-wheel drive |
| MG996R Servo | 55 g·cm torque | Front-axle steering via GPIO12 hardware PWM |
| HC-SR04 Ultrasonic Sensor | 2–400 cm, ±3 mm | Forward obstacle distance; wired to Arduino pin 10 |
| Lego Technic Bugatti (42083) | — | Chassis donor |

Full wiring details are in `hardware/wiring.md`. Parts sourcing is in `hardware/parts_list.md`.

---

## Software Architecture

```
                        ┌─────────────────────────────────────────────────────┐
                        │                  ROS2 Jazzy Topics                  │
                        └─────────────────────────────────────────────────────┘

 [CameraNode]                   /camera/image_raw
  (Logitech C925e)  ──────────────────────────────► [ParkingDetectorNode]
                                                            │
                                                            ▼
                                                     /parking_spot ──► [ParkingPolicyNode]
                                                                               │
 [SerialNode]       ──────► /distance                                          │
  (Arduino/HC-SR04)              │                                             │
                                 ├──────────────────► [PolicyNode]             │
                                 ├──────────────────► [ExplorerNode]           │
                                 └──────────────────► [LLMNavigatorNode]       │
                                                                               │
 [DetectionNode]    ──────► /detections                                        │
  (YOLOv8/Hailo-8L)              │                                             │
                                 ├──────────────────► [PolicyNode]             │
                                 ├──────────────────► [ExplorerNode]           │
                                 └──────────────────► [LLMNavigatorNode]       │
                                                                               │
 [TeleopNode]       ──────► /car_command ◄─────────── [PolicyNode] ◄──────────┘
  (WASD keyboard)                │               ◄─── [ExplorerNode]
                                 │               ◄─── [LLMNavigatorNode]
                                 │               ◄─── [ParkingPolicyNode]
                                 │               ◄─── [SafetyNode]
                                 ▼
                         [SerialNode]  ──── serial ──► Arduino ──► Motors (M2/M3)

 [TeleopNode]       ──────► /steering_angle ◄──────── [PolicyNode]
                                 │               ◄─── [ExplorerNode]
                                 │               ◄─── [LLMNavigatorNode]
                                 │               ◄─── [ParkingPolicyNode]
                                 ▼
                         [ServoNode]  ──── sysfs PWM ──► MG996R Servo
```

---

## ROS2 Nodes

| Node | Source file | Purpose | Subscribes | Publishes |
|------|-------------|---------|------------|-----------|
| `serial_node` | `serial_node.py` | Serial bridge to Arduino | `/car_command` | `/distance`, `/arduino_response` |
| `servo_node` | `servo_node.py` | Hardware PWM steering | `/steering_angle` | — |
| `camera_node` | `camera_node.py` | Camera publisher (teleop/streaming mode) | — | `/camera/image_raw` |
| `teleop_node` | `teleop_node.py` | WASD keyboard control | — | `/car_command`, `/steering_angle` |
| `safety_node` | `safety_node.py` | Hard sonar emergency stop | `/distance` | `/car_command` |
| `detection_node` | `detection_node.py` | YOLOv8s on Hailo-8L | — | `/detections` |
| `explorer_node` | `explorer_node.py` | Rule-based autonomous navigation | `/distance`, `/detections`, `/explorer_cmd` | `/car_command`, `/steering_angle` |
| `llm_navigator_node` | `llm_navigator_node.py` | Ollama LLM navigation with safety FSM | `/distance`, `/detections`, `/llm_cmd` | `/car_command`, `/steering_angle`, `/llm_status` |
| `data_collector_node` | `data_collector_node.py` | Record teleop sessions to CSV | `/distance`, `/detections`, `/car_command`, `/steering_angle`, `/collector_cmd` | — |
| `policy_node` | `policy_node.py` | Deploy behavior cloning model at 20 Hz | `/distance`, `/detections`, `/policy_cmd` | `/car_command`, `/steering_angle`, `/policy_status` |
| `parking_detector_node` | `parking_detector_node.py` | OpenCV black-tape spot detection | `/camera/image_raw` | `/parking_spot`, `/parking_debug` |
| `parking_collector_node` | `parking_collector_node.py` | Record parking demos to CSV | `/parking_spot`, `/distance`, `/car_command`, `/steering_angle`, `/parking_collector_cmd` | — |
| `parking_policy_node` | `parking_policy_node.py` | Deploy parking behavior cloning model | `/parking_spot`, `/distance`, `/parking_cmd` | `/car_command`, `/steering_angle`, `/parking_status` |

---

## Behavior Cloning Pipeline

Behavior cloning is the core autonomous driving approach. The idea is simple: record what a human driver does, then train a neural network to imitate it.

### 1. Data Collection

Launch the sensor stack and the data collector, then drive the car manually:

```bash
ros2 run autonomous_car serial_node &
ros2 run autonomous_car detection_node &
ros2 run autonomous_car data_collector_node &
ros2 run autonomous_car teleop_node

# In another terminal, start/stop recording
ros2 topic pub --once /collector_cmd std_msgs/msg/String "data: 'START'"
# ... drive around ...
ros2 topic pub --once /collector_cmd std_msgs/msg/String "data: 'STOP'"
```

Each session is saved to `~/ros2_ws/training_data/session_<timestamp>.csv`.

**Feature vector** (8 inputs recorded at 20 Hz):

| Feature | Description |
|---------|-------------|
| `sonar_cm` | Filtered distance reading from HC-SR04 |
| `sonar_trend` | -1 (approaching wall), 0 (stable), +1 (opening up) |
| `obj_left` | YOLO object count in left third of frame (capped at 5) |
| `obj_center` | YOLO object count in center third of frame |
| `obj_right` | YOLO object count in right third of frame |
| `person_present` | 1 if a person is detected with confidence > 0.5 |
| `speed` | Motor PWM value output by teleop (label) |
| `steering` | Servo angle in degrees output by teleop (label) |

### 2. Training

```bash
cd ~/ros2_ws/src/autonomous_car/training
python3 train_policy.py
```

The script:
- Loads all `session_*.csv` files from `~/ros2_ws/training_data/`
- Normalizes features (sonar to [0, 1], object counts to [0, 1], trend to [0, 1])
- Trains a `(64, 64, 32)` ReLU MLP with Adam optimizer using scikit-learn's `MLPRegressor`
- Uses early stopping (patience 20 iterations, 10% validation split)
- Reports validation MAE for speed (out of 255) and steering (out of 90°)
- Saves model + scaler to `~/ros2_ws/trained_policy.pkl`

### 3. Lateral / Longitudinal Split

A key design decision: **the trained model only controls steering**. Speed is handled by a separate rule-based longitudinal controller:

```
distance > 120 cm  →  cruise at PWM 160
distance > 80 cm   →  steady at PWM 130
35 < distance < 80 →  linear ramp 130 → 0
distance < 35 cm   →  safety stop (state machine: stop → reverse → turn)
person in center   →  force speed = 0
```

This split makes the system more robust: even if the learned policy has a distribution shift, the longitudinal controller prevents the car from crashing into walls at full speed.

### 4. Kalman Filter on Sonar

The HC-SR04 sonar has occasional spikes. `policy_node` runs a simple 1-D Kalman filter before feeding sonar readings to the policy and the longitudinal controller:

```
Process noise Q = 2.0   (distance can change quickly)
Measurement noise R = 15.0  (sonar spec ~1-3cm, but spikes happen)
```

A reading of `DIST:0` from the Arduino means out-of-range (clear path), not a 0 cm obstacle. The Kalman filter holds the last valid estimate when a `0` arrives.

### 5. EMA Steering Smoothing

Raw policy output is passed through an exponential moving average before commanding the servo:

```
smoothed = 0.25 * raw + 0.75 * previous_smoothed
```

This eliminates high-frequency jitter from frame-to-frame model variation without introducing excessive lag.

### 6. Deployment

```bash
ros2 run autonomous_car serial_node &
ros2 run autonomous_car servo_node &
ros2 run autonomous_car detection_node &
ros2 run autonomous_car policy_node

# Start the policy
ros2 topic pub --once /policy_cmd std_msgs/msg/String "data: 'START'"
```

---

## Self-Parking

The parking system uses a second behavior cloning pipeline focused entirely on navigating the car into a marked parking spot (black electrical tape rectangle on a wooden floor).

### 1. Spot Detection — `parking_detector_node`

The detector uses classical computer vision, no ML:

1. Crop the bottom 65% of the frame (floor only, ignore ceiling/walls)
2. Convert to grayscale and Gaussian blur (7x7 kernel)
3. Threshold at pixel value 80 — black tape becomes white mask
4. Morphological close (3x3, 2 iterations) to fill gaps
5. Find external contours; filter by area (800–200000 px²)
6. Merge all valid contours into a single `minAreaRect`
7. Publish normalized spot features: `{visible, cx, cy, area, angle}`

`cx` is normalized to [-1, 1] (negative = spot left of center).
`area` is normalized to [0, 1] relative to the cropped frame.

### 2. Data Collection — `parking_collector_node`

Similar to the main data collector, but records parking-specific features. Run 30-50 demos: start outside the spot, drive in, park, stop.

```bash
ros2 topic pub --once /parking_collector_cmd std_msgs/msg/String "data: 'START'"
# drive into spot
ros2 topic pub --once /parking_collector_cmd std_msgs/msg/String "data: 'STOP'"
# repeat
ros2 topic pub --once /parking_collector_cmd std_msgs/msg/String "data: 'SAVE'"
```

### 3. Training — `train_parking.py`

```bash
cd ~/ros2_ws/src/autonomous_car/training
python3 train_parking.py
```

Trains a steering-only MLP — speed is always rule-based (80–100 PWM, controlled by phase).
Input features: `[spot_visible, spot_cx, spot_cy, spot_area, sonar_norm, sonar_trend_norm]`
Output: normalized steering angle.

### 4. Phase-Based Policy — `parking_policy_node`

The node runs a four-phase state machine:

| Phase | Condition to enter | Action |
|-------|--------------------|--------|
| `searching` | Default / spot not visible | Creep forward at PWM 100, no steering |
| `aligning` | Spot visible | Run learned policy to steer toward cx = 0 |
| `parking` | `abs(cx) < 0.15` and `area > 0.05` | Run learned policy at slightly higher speed |
| `parked` | `area >= 0.35` (spot fills 35% of frame) | Stop, shutdown |

Hard sonar stop (25 cm threshold) overrides all phases.

---

## Setup Instructions

### Hardware Requirements

- Raspberry Pi 5 (4 GB or 8 GB)
- Hailo-8L AI HAT (optional — remove `detection_node` from launch if not present)
- Arduino Uno R3 with SunFounder Zeus shield
- MG996R servo with custom PWM overlay (see `hardware/wiring.md`)
- HC-SR04 sonar wired to Arduino
- Logitech C925e or compatible USB webcam

### 1. Install ROS2 Jazzy

Follow the official instructions for Ubuntu 24.04:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 2. Install HailoRT 4.20.0

Download the HailoRT `.deb` packages for ARM64 from the Hailo Developer Zone and install:

```bash
sudo dpkg -i hailort_4.20.0_arm64.deb
sudo dpkg -i hailort-pcie-driver_4.20.0_arm64.deb
pip install hailort-4.20.0-cp312-cp312-linux_aarch64.whl
```

Verify the HAT is detected:

```bash
ls /dev/hailo0
hailortcli fw-control identify
```

### 3. Install Python Dependencies

```bash
pip install scikit-learn numpy opencv-python pyserial requests
```

### 4. Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/dchavan2192/autonomous-bugatti.git autonomous_car
cd ~/ros2_ws
colcon build --packages-select autonomous_car
source install/setup.bash
```

### 5. Hardware Setup

**udev rule for Arduino** — create `/etc/udev/rules.d/99-arduino.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Hardware PWM overlay for servo** — see `hardware/wiring.md` for the full procedure using `pwm-pi5-fix.dts`.

### 6. Download YOLOv8s HEF Model

The compiled Hailo model file is not included in the repo (binary, ~20 MB).
Download `yolov8s.hef` from the Hailo Model Zoo and place it at:

```
/home/<user>/yolov8s.hef
```

Or recompile from ONNX using the Hailo Dataflow Compiler.

### 7. Run the Base Stack

```bash
# Terminal 1 — launch serial bridge + servo
ros2 launch autonomous_car drive.launch.py

# Terminal 2 — teleop
ros2 run autonomous_car teleop_node
```

### 8. Run Autonomous Modes

**Behavior cloning policy** (requires trained model):

```bash
ros2 run autonomous_car detection_node &
ros2 run autonomous_car policy_node &
ros2 topic pub --once /policy_cmd std_msgs/msg/String "data: 'START'"
```

**Explorer (rule-based)**:

```bash
ros2 run autonomous_car explorer_node &
ros2 topic pub --once /explorer_cmd std_msgs/msg/String "data: 'START'"
```

**LLM navigation** (requires Ollama with `qwen2.5:1.5b`):

```bash
ollama pull qwen2.5:1.5b
ros2 run autonomous_car llm_navigator_node &
ros2 topic pub --once /llm_cmd std_msgs/msg/String "data: 'START'"
```

**Self-parking** (requires trained parking model):

```bash
ros2 run autonomous_car camera_node &
ros2 run autonomous_car parking_detector_node &
ros2 run autonomous_car parking_policy_node &
ros2 topic pub --once /parking_cmd std_msgs/msg/String "data: 'START'"
```

---

## Training Your Own Model

### Collect Driving Data

1. Launch the sensor stack and data collector.
2. Drive the car with WASD teleop for at least 5–10 minutes across varied conditions — open areas, near walls, with objects present.
3. Aim for balanced data: roughly equal forward/stop/reverse and left/straight/right samples.

```bash
ros2 run autonomous_car serial_node &
ros2 run autonomous_car detection_node &
ros2 run autonomous_car data_collector_node &
ros2 run autonomous_car teleop_node

# Start/stop recording in another terminal
ros2 topic pub --once /collector_cmd std_msgs/msg/String "data: 'START'"
ros2 topic pub --once /collector_cmd std_msgs/msg/String "data: 'STOP'"
```

### Train the Policy

```bash
cd ~/ros2_ws/src/autonomous_car/training
python3 train_policy.py
```

Training takes under 60 seconds on the Pi 5. Target validation MAE: speed < 20 (out of 255), steering < 8° (out of 90°).

### Collect Parking Data

Do 30–50 demos: start 1–2 meters from the spot, drive toward it, align, and pull in fully.

```bash
ros2 run autonomous_car camera_node &
ros2 run autonomous_car parking_detector_node &
ros2 run autonomous_car parking_collector_node &
ros2 run autonomous_car teleop_node

ros2 topic pub --once /parking_collector_cmd std_msgs/msg/String "data: 'START'"
# drive demo
ros2 topic pub --once /parking_collector_cmd std_msgs/msg/String "data: 'STOP'"
# repeat 30-50 times, then:
ros2 topic pub --once /parking_collector_cmd std_msgs/msg/String "data: 'SAVE'"
```

### Train the Parking Model

```bash
cd ~/ros2_ws/src/autonomous_car/training
python3 train_parking.py
```

---

## Roadmap

- [ ] IMU integration (ICM20948) for odometry and heading estimation
- [ ] Larger training dataset (current: ~1 session; target: 10+ diverse sessions)
- [ ] Demo video of full autonomous lap and self-parking
- [ ] Packaging: single-command launch for each autonomous mode
- [ ] Waypoint navigation using sonar + IMU dead reckoning
- [ ] ROS2 bag recording for offline dataset inspection

---

## License

MIT License — see `LICENSE` for details.
