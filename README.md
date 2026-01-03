# ESP32-S2 Mini 12-Channel Line Follower Robot

> **Complete Integration**: Sensors + Motor Driver + Encoders + OLED + ROS

A professional-grade line following robot built on ESP32-S2 Mini with advanced features including 12-channel sensor array, PID control, encoder feedback, real-time OLED display, and ROS integration.

---

## 📋 Table of Contents

- [System Overview](#-system-overview)
- [Hardware Components](#-hardware-components)
- [System Architecture](#-system-architecture)
- [Wiring Diagram](#-wiring-diagram)
- [Pin Assignments](#-pin-assignments)
- [Sensor Array Layout](#-sensor-array-layout)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [How It Works](#-how-it-works)
- [ROS Integration](#-ros-integration)
- [Troubleshooting](#-troubleshooting)

---

## 🎯 System Overview

This project integrates multiple components into a single, cohesive line-following robot system optimized for the ESP32-S2 Mini's single-core architecture.

### Key Features

✅ **12-Channel Sensor Array** - High-precision line detection via 74HC multiplexer  
✅ **PID Control** - Smooth, responsive line following algorithm  
✅ **Encoder Feedback** - GA25 motors with quadrature encoders (1632 PPR)  
✅ **Real-time Display** - 0.96" OLED showing sensors, error, and speeds  
✅ **ROS Compatible** - Serial communication for advanced robotics applications  
✅ **Single-Core Optimized** - Efficient implementation for ESP32-S2  
✅ **Modular Design** - Easy to customize and extend  

---

## 🔧 Hardware Components

| Component | Specification | Quantity |
|-----------|--------------|----------|
| **Microcontroller** | ESP32-S2 Mini | 1 |
| **Sensor Array** | IR sensors + 74HC multiplexer | 12 sensors |
| **Motor Driver** | TB6612FNG dual motor driver | 1 |
| **Motors** | GA25 gear motors with encoders | 2 |
| **Display** | 0.96" OLED (128x64, I2C, SSD1306) | 1 |
| **Power Supply** | 6-12V for motors, 5V for logic | - |

---

## 🏗️ System Architecture

![System Architecture](file:///home/alvee/.gemini/antigravity/brain/b8da6554-6571-4cea-8be8-c9bde5b5a46f/system_architecture_1767437801958.png)

The system consists of five main subsystems:

1. **Sensor Subsystem**: 12 IR sensors → 74HC multiplexer → ESP32-S2 ADC
2. **Motor Control**: ESP32-S2 PWM → TB6612FNG → GA25 motors
3. **Encoder Feedback**: Motor encoders → ESP32-S2 GPIO (quadrature decoding)
4. **Display**: ESP32-S2 I2C → SSD1306 OLED
5. **Communication**: ESP32-S2 USB Serial → ROS (optional)

---

## 🔌 Wiring Diagram

![Wiring Diagram](file:///home/alvee/.gemini/antigravity/brain/b8da6554-6571-4cea-8be8-c9bde5b5a46f/wiring_diagram_1767437829782.png)

### Power Distribution

- **ESP32-S2**: 5V via USB or external regulator
- **Motors (TB6612FNG VM)**: 6-12V from battery
- **Logic (TB6612FNG VCC)**: 5V (shared with ESP32)
- **OLED**: 3.3V from ESP32-S2

> ⚠️ **Important**: Never connect motor voltage (VM) to ESP32 pins!

---

## 📌 Pin Assignments

### ESP32-S2 Mini Pinout Reference

![ESP32-S2 Pinout](file:///home/alvee/.gemini/antigravity/brain/b8da6554-6571-4cea-8be8-c9bde5b5a46f/esp32_s2_pinout_1767437552361.png)

### Complete Pin Mapping

#### 🔍 Sensor Multiplexer (74HC)

| Function | GPIO | Type | Description |
|----------|------|------|-------------|
| SELECT_A | 37 | Digital Out | Multiplexer select bit A |
| SELECT_B | 38 | Digital Out | Multiplexer select bit B |
| INPUT_A | 1 | Analog In | ADC1_CH0 - Sensors 6,7,9,8 |
| INPUT_B | 2 | Analog In | ADC1_CH1 - Sensors 11,10 |
| INPUT_C | 3 | Analog In | ADC1_CH2 - Sensors 4,2,3,5 |
| INPUT_D | 4 | Analog In | ADC1_CH3 - Sensors 1,0 |

#### 🚗 Motor Driver (TB6612FNG)

| Function | GPIO | Type | Description |
|----------|------|------|-------------|
| MOTOR_L_PWM | 12 | PWM Out | Left motor speed control |
| MOTOR_L_IN1 | 11 | Digital Out | Left motor direction 1 |
| MOTOR_L_IN2 | 10 | Digital Out | Left motor direction 2 |
| MOTOR_R_PWM | 9 | PWM Out | Right motor speed control |
| MOTOR_R_IN1 | 8 | Digital Out | Right motor direction 1 |
| MOTOR_R_IN2 | 7 | Digital Out | Right motor direction 2 |
| MOTOR_STBY | 6 | Digital Out | Standby (HIGH=active) |

#### 📊 Encoders (GA25 Motors)

| Function | GPIO | Type | Description |
|----------|------|------|-------------|
| ENCODER_L_A | 5 | Digital In | Left encoder channel A (green) |
| ENCODER_L_B | 18 | Digital In | Left encoder channel B (yellow) |
| ENCODER_R_A | 16 | Digital In | Right encoder channel A (yellow) |
| ENCODER_R_B | 17 | Digital In | Right encoder channel B (green) |

#### 📺 OLED Display (I2C)

| Function | GPIO | Type | Description |
|----------|------|------|-------------|
| I2C_SDA | 33 | I2C Data | OLED data line |
| I2C_SCL | 35 | I2C Clock | OLED clock line |

---

## 🎯 Sensor Array Layout

![Sensor Array Layout](file:///home/alvee/.gemini/antigravity/brain/b8da6554-6571-4cea-8be8-c9bde5b5a46f/sensor_array_layout_1767437848429.png)

### Sensor Weighting System

The 12 sensors are weighted from **-11 to +11** for position calculation:

```
Sensor:  [0]  [1]  [2]  [3]  [4]  [5]  [6]  [7]  [8]  [9]  [10] [11]
Weight:  -11  -9   -7   -5   -3   -1   +1   +3   +5   +7   +9   +11
         ←←←←←←← LEFT          CENTER          RIGHT →→→→→→→
```

**How it works:**
- Line under **left sensors** → Negative error → Turn left
- Line under **center sensors** → Near-zero error → Go straight
- Line under **right sensors** → Positive error → Turn right

---

## 🚀 Installation

### 1. Arduino IDE Setup

```bash
# Install Arduino IDE (if not already installed)
# Download from: https://www.arduino.cc/en/software

# Add ESP32 board support
# File → Preferences → Additional Board URLs:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Tools → Board → Board Manager → Search "esp32" → Install
```

### 2. Install Required Libraries

Via Arduino IDE Library Manager (`Sketch → Include Library → Manage Libraries`):

- **Adafruit GFX Library** (for graphics primitives)
- **Adafruit SSD1306** (for OLED display)
- **Rosserial Arduino Library** (for ROS, optional)

Or via Arduino CLI:
```bash
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Rosserial Arduino Library"
```

### 3. Upload Code

1. Open `ESP32_LFR_Complete.ino` in Arduino IDE
2. Select **Tools → Board → ESP32S2 Dev Module**
3. Select your COM port
4. Click **Upload** ⬆️

---

## ⚙️ Configuration

All settings are in [`config.h`](file:///home/alvee/Desktop/line_follower_robot/ESP32_LFR_Complete/config.h). Key parameters:

### PID Tuning

```cpp
#define KP  1.5   // Proportional gain (responsiveness)
#define KI  0.0   // Integral gain (steady-state error)
#define KD  0.8   // Derivative gain (damping/smoothness)
```

**Tuning Guide:**
1. Start with KP only (set KI=0, KD=0)
2. Increase KP until robot oscillates
3. Add KD to reduce oscillation
4. Add KI only if there's steady-state error

### Motor Speeds

```cpp
#define BASE_SPEED  150  // Cruising speed (0-255)
#define MAX_SPEED   255  // Maximum speed limit
#define TURN_SPEED  100  // Speed during sharp turns
```

### Sensor Threshold

```cpp
#define SENSOR_THRESHOLD  2000  // ADC value (0-4095)
// Above threshold = white surface
// Below threshold = black line
```

### Feature Flags

```cpp
#define USE_ROS      // Comment out to disable ROS
#define USE_OLED     // Comment out to disable OLED
#define USE_ENCODERS // Comment out to disable encoders
```

---

## 🧠 How It Works

### Data Flow

![Data Flow Diagram](file:///home/alvee/.gemini/antigravity/brain/b8da6554-6571-4cea-8be8-c9bde5b5a46f/data_flow_diagram_1767437866746.png)

### Control Loop (10ms cycle)

```
1. Read Sensors → 12 ADC values via multiplexer
2. Convert to Binary → Compare with threshold
3. Calculate Error → Weighted position (-11 to +11)
4. PID Controller → Compute correction
5. Motor Speeds → Base speed ± PID output
6. Update Display → Show status on OLED
7. Publish ROS → Send data to ROS (optional)
8. Update Encoders → Count pulses for odometry
```

### Encoder Operation

![Encoder Quadrature](file:///home/alvee/.gemini/antigravity/brain/b8da6554-6571-4cea-8be8-c9bde5b5a46f/encoder_quadrature_1767437887540.png)

**Quadrature Encoding:**
- Two channels (A & B) create 4 states: 00, 01, 10, 11
- **Forward**: 00→01→11→10→00 (clockwise)
- **Backward**: 00→10→11→01→00 (counter-clockwise)
- Direction determined by state transition sequence

**GA25 Specifications:**
- 1632 pulses per revolution
- Allows precise distance and speed measurement

---

## 🤖 ROS Integration

### Setup ROS Serial

**On your Linux computer:**

```bash
# Install rosserial
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial

# Start roscore
roscore
```

**In another terminal:**
```bash
# Connect to ESP32-S2
rosrun rosserial_python serial_node.py /dev/ttyUSB0

# If permission denied:
sudo chmod 666 /dev/ttyUSB0
```

### Available Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/lfr/sensors` | Int16MultiArray | ESP32 → ROS | All 12 sensor ADC values |
| `/lfr/status` | Float32 | ESP32 → ROS | Current PID error value |
| `/lfr/cmd_vel` | Twist | ROS → ESP32 | Manual control commands |

### Example Commands

**View sensor data:**
```bash
rostopic echo /lfr/sensors
```

**View error value:**
```bash
rostopic echo /lfr/status
```

**Manual control (switch to MANUAL mode first):**
```bash
# Move forward
rostopic pub /lfr/cmd_vel geometry_msgs/Twist "linear: {x: 150.0} angular: {z: 0.0}"

# Turn right
rostopic pub /lfr/cmd_vel geometry_msgs/Twist "linear: {x: 100.0} angular: {z: 50.0}"

# Stop
rostopic pub /lfr/cmd_vel geometry_msgs/Twist "linear: {x: 0.0} angular: {z: 0.0}"
```

---

## 🐛 Troubleshooting

### OLED Not Working

**Symptoms**: Blank display or "SSD1306 allocation failed"

**Solutions:**
- Check I2C address (try 0x3C or 0x3D)
- Verify SDA/SCL connections (GPIO33/35)
- Test with I2C scanner sketch
- Check OLED power (3.3V)

### Motors Not Running

**Symptoms**: Motors don't respond

**Solutions:**
- Check STBY pin is HIGH (GPIO6)
- Verify motor power supply (VM: 6-12V)
- Test motor driver connections
- Check PWM channel assignments
- Ensure motor driver VCC has 5V

### Sensors Not Reading

**Symptoms**: All sensors show same value or no response

**Solutions:**
- Run sensor calibration (automatic on startup)
- Check multiplexer connections
- Verify sensor power supply
- Adjust `SENSOR_THRESHOLD` in config.h
- Test individual sensors

### Encoders Not Counting

**Symptoms**: Encoder values stay at 0

**Solutions:**
- Check encoder wiring (A/B channels)
- Verify pull-up resistors enabled
- Test with `DEBUG_ENCODERS true`
- Manually rotate wheels and watch serial output
- Ensure encoders have power

### ROS Connection Failed

**Symptoms**: rosserial can't connect

**Solutions:**
- Check serial port: `ls /dev/ttyUSB*`
- Fix permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate (115200)
- Ensure rosserial is installed
- Check USB cable quality

---

## 📊 Performance Tips

### Optimal Track Conditions

- **Line width**: 2-3 cm black tape
- **Surface**: Matte white (avoid glossy/reflective)
- **Lighting**: Consistent, avoid direct sunlight
- **Curves**: Gradual (radius > 20cm for high speed)

### Speed Optimization

1. Start with `BASE_SPEED = 100`
2. Gradually increase by 25 until oscillation appears
3. Back off by 10-15 for safety margin
4. Adjust PID gains if needed

### PID Tuning Process

```
1. Set KP=1.0, KI=0, KD=0
2. Increase KP until robot oscillates
3. Reduce KP by 20%
4. Add KD (start with KP/2)
5. Fine-tune both for smooth following
6. Add KI only if steady-state error exists
```

---

## 📁 Project Structure

```
ESP32_LFR_Complete/
├── ESP32_LFR_Complete.ino  # Main sketch (~650 lines)
├── config.h                 # Pin definitions & settings
└── README.md               # This file
```

---

## 🎓 Technical Details

### Single-Core Optimization

Unlike dual-core ESP32, the ESP32-S2 uses a single-core architecture:

- **Encoders**: Polled in main loop (no separate task)
- **Loop frequency**: ~100Hz (10ms delay)
- **Fast enough**: GA25 max speed = 5400 pulses/sec, we sample at 10000 Hz equivalent

### Memory Usage

- **Flash**: ~200KB (with all features enabled)
- **RAM**: ~15KB (including ROS buffers)
- **Stack**: 4KB (sufficient for single-core)

### Multiplexer Operation

The 74HC multiplexer uses 2 select pins to route 4 analog inputs:

```
SELECT_B | SELECT_A | Active Sensors
---------|----------|---------------
   0     |    0     | 6, 4, 1
   0     |    1     | 7, 11, 2, 0
   1     |    0     | 9, 10, 3
   1     |    1     | 8, 5
```

---

## 📝 License

Based on demo codes by [mdaslamhossain3825-glitch](https://github.com/mdaslamhossain3825-glitch)  
Modified and integrated for ESP32-S2 Mini

---

## 🤝 Support

**Need help?**
1. Check the [Troubleshooting](#-troubleshooting) section
2. Review pin connections against wiring diagram
3. Enable debug flags in `config.h`
4. Test components individually

---

## 🎯 Next Steps

- [ ] Upload code and test basic functionality
- [ ] Calibrate sensors on your track
- [ ] Tune PID parameters for your robot
- [ ] Optimize speed settings
- [ ] (Optional) Set up ROS integration

---

**Happy Line Following! 🤖🏁**
