/*
 * ESP32-S2 Mini 12-Channel Line Follower Robot - Configuration File
 * 
 * Board: ESP32-S2 Mini (SINGLE CORE)
 * This file contains all pin definitions, constants, and tunable parameters
 * for the complete LFR system integration.
 * 
 * Pin Selection Notes:
 * - Avoid GPIO0, GPIO45, GPIO46 (strapping pins)
 * - ADC1 channels preferred (GPIO1-GPIO10) for analog inputs
 * - I2C default: SDA=GPIO33, SCL=GPIO35 (but can use any GPIO)
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
#define ESP32_S2_MINI     // Single-core ESP32-S2 Mini board

// ============================================================================
// FEATURE FLAGS
// ============================================================================
#define USE_ROS           // Comment out to disable ROS integration
#define USE_OLED          // Comment out to disable OLED display
#define USE_ENCODERS      // Comment out to disable encoder feedback

// ============================================================================
// SENSOR PINS (74HC Multiplexer)
// ============================================================================
#define SELECT_A          37    // Multiplexer select bit A
#define SELECT_B          38    // Multiplexer select bit B
#define INPUT_A           1     // Analog input A (ADC1_CH0)
#define INPUT_B           2     // Analog input B (ADC1_CH1)
#define INPUT_C           3     // Analog input C (ADC1_CH2)
#define INPUT_D           4     // Analog input D (ADC1_CH3)

#define SENSOR_COUNT      12    // Total number of sensors

// ============================================================================
// MOTOR DRIVER PINS (TB6612FNG)
// ============================================================================
// Left Motor (Motor A)
#define MOTOR_L_PWM       12    // PWM pin for left motor speed
#define MOTOR_L_IN1       11    // Direction pin 1
#define MOTOR_L_IN2       10    // Direction pin 2

// Right Motor (Motor B)
#define MOTOR_R_PWM       9     // PWM pin for right motor speed
#define MOTOR_R_IN1       8     // Direction pin 1
#define MOTOR_R_IN2       7     // Direction pin 2

// Standby Pin
#define MOTOR_STBY        6     // Standby pin (HIGH = active, LOW = standby)

// PWM Configuration
#define PWM_FREQ          1000  // PWM frequency in Hz
#define PWM_RESOLUTION    8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_L     0     // PWM channel for left motor
#define PWM_CHANNEL_R     1     // PWM channel for right motor

// ============================================================================
// ENCODER PINS (GA25 Motors)
// ============================================================================
#define ENCODER_L_A       5     // Left encoder channel A (green wire)
#define ENCODER_L_B       18    // Left encoder channel B (yellow wire)
#define ENCODER_R_A       16    // Right encoder channel A (yellow wire)
#define ENCODER_R_B       17    // Right encoder channel B (green wire)

// Encoder Specifications
#define PULSES_PER_REV    1632  // GA25 motor: 1632 pulses per revolution
#define WHEEL_DIAMETER    65.0  // Wheel diameter in mm
#define WHEEL_BASE        150.0 // Distance between wheels in mm

// ============================================================================
// OLED DISPLAY (I2C)
// ============================================================================
#define OLED_WIDTH        128   // OLED display width in pixels
#define OLED_HEIGHT       64    // OLED display height in pixels
#define OLED_RESET        -1    // Reset pin (-1 if sharing Arduino reset)
#define OLED_ADDRESS      0x3C  // I2C address (0x3C or 0x3D)

// I2C Pins (ESP32-S2 default)
#define I2C_SDA           33    // I2C Data
#define I2C_SCL           35    // I2C Clock

// ============================================================================
// BUTTON PINS (Menu Navigation)
// ============================================================================
#define BUTTON_UP         34    // Navigate up / Increase value
#define BUTTON_DOWN       36    // Navigate down / Decrease value
#define BUTTON_SELECT     39    // Enter menu / Confirm selection (FIXED: was 37, conflicted with SELECT_A)

// Button Configuration
#define BUTTON_DEBOUNCE   50    // Debounce delay in ms
#define LONG_PRESS_TIME   1000  // Long press duration in ms

// ============================================================================
// MENU SYSTEM CONFIGURATION
// ============================================================================
#define MENU_TIMEOUT      30000 // Exit menu after 30s inactivity (ms)
#define EEPROM_NAMESPACE  "LFR_Settings"
#define SETTINGS_VERSION  1     // Increment when settings structure changes

// ============================================================================
// SENSOR CALIBRATION
// ============================================================================
#define SENSOR_THRESHOLD  2000  // ADC threshold (0-4095): above = white, below = black
#define CALIBRATION_SAMPLES 100 // Number of samples for calibration

// ============================================================================
// PID CONTROLLER PARAMETERS
// ============================================================================
// PID Gains (tune these for your robot)
#define KP                1.5   // Proportional gain
#define KI                0.0   // Integral gain (disabled by default)
#define KD                0.8   // Derivative gain

// Adaptive PID (gains adjust based on error magnitude)
#define USE_ADAPTIVE_PID  true  // Enable adaptive PID tuning
#define KP_AGGRESSIVE     2.0   // Aggressive P gain for large errors
#define KD_AGGRESSIVE     1.2   // Aggressive D gain for large errors
#define ADAPTIVE_THRESHOLD 3.0  // Error threshold to switch to aggressive

// Derivative Filtering (reduce noise sensitivity)
#define DERIVATIVE_FILTER 0.7   // Low-pass filter constant (0-1, higher = more filtering)

// PID Limits
#define PID_MAX           255   // Maximum PID output
#define PID_MIN           -255  // Minimum PID output
#define INTEGRAL_MAX      100   // Integral windup limit

// ============================================================================
// MOTOR SPEED SETTINGS
// ============================================================================
#define BASE_SPEED        150   // Base motor speed (0-255)
#define MAX_SPEED         255   // Maximum motor speed
#define MIN_SPEED         0     // Minimum motor speed
#define TURN_SPEED        120   // Speed during sharp turns
#define PIVOT_SPEED       100   // Speed during pivot turns (encoder-based)

// ============================================================================
// LINE FOLLOWING BEHAVIOR
// ============================================================================
#define SENSOR_WEIGHTS    {-11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11}  // Position weights
#define LOST_LINE_TIMEOUT 1000  // Time in ms before considering line lost
#define STOP_ON_LOST      false // Stop motors when line is lost

// ============================================================================
// TURN DETECTION SETTINGS
// ============================================================================
// Turn Detection Thresholds
#define TURN_DETECT_THRESHOLD  3    // Minimum consecutive sensors to detect turn
#define LEFT_SENSOR_START      0    // Leftmost sensor index
#define LEFT_SENSOR_END        2    // Left turn detection range end
#define RIGHT_SENSOR_START     9    // Right turn detection range start
#define RIGHT_SENSOR_END       11   // Rightmost sensor index
#define CENTER_SENSOR_START    4    // Center region start
#define CENTER_SENSOR_END      7    // Center region end

// Turn Detection Behavior
#define ENABLE_TURN_DETECTION  true // Enable automatic turn detection
#define TURN_COOLDOWN         500   // Cooldown time in ms between turns

// Junction Handling (when multiple turn options detected)
enum TurnType {
  TURN_NONE = 0,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_T_JUNCTION,      // Both left and right available
  TURN_CROSSROAD,       // All directions available
  TURN_SHARP_LEFT,      // Very sharp left turn
  TURN_SHARP_RIGHT      // Very sharp right turn
};

// Default behaviors for junctions
#define T_JUNCTION_DEFAULT  TURN_RIGHT    // Default when T-junction detected
#define CROSSROAD_DEFAULT   TURN_NONE     // Default when crossroad detected (go straight)

// Junction Action Planning (configurable via menu)
enum JunctionAction {
  JUNCTION_LEFT = 0,
  JUNCTION_STRAIGHT = 1,
  JUNCTION_RIGHT = 2
};

// Path Planning Configuration
#define MAX_JUNCTIONS        10   // Maximum junctions in path plan

// Turn Movement Calibration
#define FORWARD_BEFORE_TURN  50   // mm to move forward before turn
#define FORWARD_AFTER_TURN   30   // mm to move forward after turn

// ============================================================================
// INVERTED LINE SECTION DETECTION
// ============================================================================
#define ENABLE_INVERSION    true  // Enable inverted line detection
#define INVERSION_THRESHOLD 8     // Min sensors flipped to detect inversion
#define INVERSION_CONFIRM   3     // Consecutive readings to confirm inversion
#define INVERSION_HYSTERESIS 2    // Readings needed to switch back

// ============================================================================
// ENCODER-BASED TURNING PARAMETERS
// ============================================================================
#define ENABLE_ENCODER_TURNS true  // Use encoders for precise turns

// Turn Calculations
#define TURN_ANGLE_90       90.0   // Standard 90 degree turn
#define TURN_ANGLE_45       45.0   // 45 degree turn
#define TURN_TOLERANCE      5.0    // Acceptable error in degrees

// Calculate pulses per degree of rotation
// Formula: (WHEEL_BASE * PI * PULSES_PER_REV) / (WHEEL_DIAMETER * PI * 360)
#define PULSES_PER_DEGREE   ((WHEEL_BASE * PULSES_PER_REV) / (WHEEL_DIAMETER * 360.0))

// Turn Execution
#define TURN_TIMEOUT        3000   // Maximum time for turn in ms
#define TURN_SETTLE_TIME    100    // Time to settle after turn in ms

// ============================================================================
// ROS CONFIGURATION
// ============================================================================
#ifdef USE_ROS
  #define ROS_BAUD_RATE   115200
  #define ROS_TOPIC_SENSORS   "/lfr/sensors"
  #define ROS_TOPIC_ODOM      "/lfr/odom"
  #define ROS_TOPIC_CMD_VEL   "/lfr/cmd_vel"
  #define ROS_TOPIC_STATUS    "/lfr/status"
  #define ROS_PUBLISH_RATE    50  // Publish rate in Hz
#endif

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define LOOP_DELAY        10    // Main loop delay in ms
#define DISPLAY_UPDATE    100   // OLED update interval in ms
#define SERIAL_BAUD       115200 // Serial communication baud rate

// ============================================================================
// OPERATING MODES
// ============================================================================
enum OperatingMode {
  MODE_IDLE,          // Robot stopped
  MODE_LINE_FOLLOW,   // Autonomous line following
  MODE_MANUAL,        // Manual control via ROS
  MODE_CALIBRATE,     // Sensor calibration mode
  MODE_MENU           // Settings menu mode
};

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define DEBUG_SENSORS     false // Print sensor values to serial
#define DEBUG_PID         false // Print PID values to serial
#define DEBUG_ENCODERS    false // Print encoder values to serial
#define DEBUG_MOTORS      false // Print motor speeds to serial

#endif // CONFIG_H
