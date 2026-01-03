/*
 * ESP32-S2 Mini 12-Channel Line Follower Robot
 * Complete Integration: Sensors + TB6612FNG + Encoders + OLED + ROS
 * 
 * Author: Based on demo codes from mdaslamhossain3825
 * Board: ESP32-S2 Mini (Single Core)
 * 
 * Features:
 * - 12-channel IR sensor reading via 74HC multiplexer
 * - TB6612FNG dual motor driver control
 * - GA25 motor encoder feedback
 * - 0.96" OLED display (128x64)
 * - ROS serial communication
 * - PID-based line following
 */

#include "config.h"

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#ifdef USE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

#ifdef USE_ROS
  #include <ros.h>
  #include <std_msgs/Int16MultiArray.h>
  #include <std_msgs/Float32.h>
  #include <geometry_msgs/Twist.h>
#endif

// EEPROM for settings storage
#include <Preferences.h>

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Sensor Data
int sensorADC[SENSOR_COUNT];
bool sensorBinary[SENSOR_COUNT];
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];

// Encoder Data
volatile long leftPulses = 0;
volatile long rightPulses = 0;
volatile uint8_t lastStateL = 0;
volatile uint8_t lastStateR = 0;

// PID Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float filteredDerivative = 0;  // Filtered derivative for noise reduction
float pidOutput = 0;

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;

// Operating Mode
OperatingMode currentMode = MODE_IDLE;

// Turn Detection
TurnType currentTurn = TURN_NONE;
unsigned long lastTurnTime = 0;
bool turnInProgress = false;

// Inverted Line Detection
bool invertedMode = false;
int inversionCounter = 0;
int normalCounter = 0;

// Encoder Turn Targets
long leftTargetPulses = 0;
long rightTargetPulses = 0;

// Timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastLineDetected = 0;

// Line Following Robustness
unsigned long lineAge = 0;
bool lineSearchActive = false;
int searchDirection = 1;
float lastValidError = 0;
float lastDerivative = 0;

// Path Planning Robustness
struct PathExecutionState {
  int junctionsExecuted;
  int junctionsMissed;
  unsigned long lastJunctionTime;
  float avgConfidence;
};
PathExecutionState pathState = {0, 0, 0, 0.0};
float junctionConfidence = 0.0;

// ============================================================================
// SETTINGS STRUCTURE & MENU SYSTEM
// ============================================================================

// Settings structure for EEPROM storage
struct Settings {
  uint8_t version;              // Settings version
  
  // PID Parameters
  float baseSpeed;
  float kp;
  float kd;
  
  // Turn Calibration
  float pulsesPerDegree;        // Encoder pulses per degree of turn
  int forwardBeforeTurn;        // mm to move forward before turn
  int forwardAfterTurn;         // mm to move forward after turn
  
  // Path Planning (sequential junction actions)
  uint8_t numJunctions;                      // Total junctions in plan (0-10)
  JunctionAction pathPlan[MAX_JUNCTIONS];    // Sequence of actions
};

Settings settings;
Preferences preferences;

// Menu State
enum MenuScreen {
  MENU_MAIN = 0,
  MENU_PID_SETTINGS,
  MENU_TURN_CALIBRATION,
  MENU_JUNCTION_PLANNING,
  MENU_SENSOR_CALIBRATION
};

MenuScreen currentMenu = MENU_MAIN;
int menuIndex = 0;
int subMenuIndex = 0;
bool inSubMenu = false;
unsigned long lastMenuActivity = 0;

// Path Planning State
int currentJunction = 0;       // Current junction number (0-based)
bool pathPlanEnabled = true;   // Enable/disable path planning
int editingJunction = 0;       // Junction being edited in menu

// Button State
struct ButtonState {
  bool pressed;
  bool lastState;
  unsigned long lastDebounce;
  unsigned long pressTime;
};

ButtonState btnUp, btnDown, btnSelect;

#ifdef USE_OLED
  Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
#endif

#ifdef USE_ROS
  ros::NodeHandle nh;
  std_msgs::Int16MultiArray sensor_msg;
  std_msgs::Float32 error_msg;
  
  void cmdVelCallback(const geometry_msgs::Twist& msg) {
    // Manual control via ROS
    if (currentMode == MODE_MANUAL) {
      float linear = msg.linear.x;
      float angular = msg.angular.z;
      
      leftSpeed = constrain(linear - angular, -MAX_SPEED, MAX_SPEED);
      rightSpeed = constrain(linear + angular, -MAX_SPEED, MAX_SPEED);
      
      setMotorSpeeds(leftSpeed, rightSpeed);
    }
  }
  
  ros::Publisher sensor_pub(ROS_TOPIC_SENSORS, &sensor_msg);
  ros::Publisher error_pub(ROS_TOPIC_STATUS, &error_msg);
  ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub(ROS_TOPIC_CMD_VEL, cmdVelCallback);
#endif

// ============================================================================
// BUTTON & SETTINGS FUNCTIONS
// ============================================================================

void initButtons() {
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  
  // Initialize button states
  btnUp.lastState = HIGH;
  btnDown.lastState = HIGH;
  btnSelect.lastState = HIGH;
}

bool readButton(ButtonState &btn, int pin) {
  bool currentState = digitalRead(pin);
  bool buttonPressed = false;
  
  // Check if state changed
  if (currentState != btn.lastState) {
    btn.lastDebounce = millis();
  }
  
  // Debounce
  if ((millis() - btn.lastDebounce) > BUTTON_DEBOUNCE) {
    if (currentState != btn.pressed) {
      btn.pressed = currentState;
      
      // Button was just pressed (LOW = pressed with pullup)
      if (btn.pressed == LOW) {
        btn.pressTime = millis();
        buttonPressed = true;
      }
    }
  }
  
  btn.lastState = currentState;
  return buttonPressed;
}

void loadSettings() {
  preferences.begin(EEPROM_NAMESPACE, false);
  
  // Check if settings exist and version matches
  uint8_t storedVersion = preferences.getUChar("version", 0);
  
  if (storedVersion == SETTINGS_VERSION) {
    // Load saved settings with validation
    settings.version = storedVersion;
    
    // Validate and constrain PID parameters
    settings.baseSpeed = constrain(preferences.getFloat("baseSpeed", BASE_SPEED), 50, 255);
    settings.kp = constrain(preferences.getFloat("kp", KP), 0, 10);
    settings.kd = constrain(preferences.getFloat("kd", KD), 0, 10);
    settings.pulsesPerDegree = constrain(preferences.getFloat("ppd", PULSES_PER_DEGREE), 0.1, 50);
    settings.forwardBeforeTurn = constrain(preferences.getInt("fwdBefore", FORWARD_BEFORE_TURN), 0, 500);
    settings.forwardAfterTurn = constrain(preferences.getInt("fwdAfter", FORWARD_AFTER_TURN), 0, 500);
    
    // Load and validate path plan
    settings.numJunctions = constrain(preferences.getUChar("numJunc", 0), 0, MAX_JUNCTIONS);
    for (int i = 0; i < MAX_JUNCTIONS; i++) {
      char key[16];
      sprintf(key, "junc%d", i);
      uint8_t action = preferences.getUChar(key, JUNCTION_STRAIGHT);
      // Validate action is within enum range
      if (action > JUNCTION_RIGHT) action = JUNCTION_STRAIGHT;
      settings.pathPlan[i] = (JunctionAction)action;
    }
    
    Serial.println("Settings loaded from EEPROM (validated)");
  } else {
    // Use defaults
    settings.version = SETTINGS_VERSION;
    settings.baseSpeed = BASE_SPEED;
    settings.kp = KP;
    settings.kd = KD;
    settings.pulsesPerDegree = PULSES_PER_DEGREE;
    settings.forwardBeforeTurn = FORWARD_BEFORE_TURN;
    settings.forwardAfterTurn = FORWARD_AFTER_TURN;
    
    // Initialize path plan
    settings.numJunctions = 0;
    for (int i = 0; i < MAX_JUNCTIONS; i++) {
      settings.pathPlan[i] = JUNCTION_STRAIGHT;
    }
    
    Serial.println("Using default settings");
    saveSettings(); // Save defaults
  }
  
  preferences.end();
}

void saveSettings() {
  preferences.begin(EEPROM_NAMESPACE, false);
  
  preferences.putUChar("version", settings.version);
  preferences.putFloat("baseSpeed", settings.baseSpeed);
  preferences.putFloat("kp", settings.kp);
  preferences.putFloat("kd", settings.kd);
  preferences.putFloat("ppd", settings.pulsesPerDegree);
  preferences.putInt("fwdBefore", settings.forwardBeforeTurn);
  preferences.putInt("fwdAfter", settings.forwardAfterTurn);
  
  // Save path plan
  preferences.putUChar("numJunc", settings.numJunctions);
  for (int i = 0; i < MAX_JUNCTIONS; i++) {
    char key[16];
    sprintf(key, "junc%d", i);
    preferences.putUChar(key, settings.pathPlan[i]);
  }
  
  preferences.end();
  
  Serial.println("Settings saved to EEPROM");
  
  #ifdef USE_OLED
    display.clearDisplay();
    display.setCursor(30, 28);
    display.setTextSize(2);
    display.println("SAVED!");
    display.display();
    delay(1000);
  #endif
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Initialize buttons for menu
  initButtons();
  
  // Load settings from EEPROM
  loadSettings();
  
  // Initialize sensor pins
  pinMode(SELECT_A, OUTPUT);
  pinMode(SELECT_B, OUTPUT);
  pinMode(INPUT_A, INPUT);
  pinMode(INPUT_B, INPUT);
  pinMode(INPUT_C, INPUT);
  pinMode(INPUT_D, INPUT);
  
  // Initialize motor driver pins
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_PWM, PWM_CHANNEL_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CHANNEL_R);
  
  // Enable motor driver
  digitalWrite(MOTOR_STBY, HIGH);
  
  #ifdef USE_ENCODERS
    // Initialize encoder pins
    pinMode(ENCODER_L_A, INPUT_PULLUP);
    pinMode(ENCODER_L_B, INPUT_PULLUP);
    pinMode(ENCODER_R_A, INPUT_PULLUP);
    pinMode(ENCODER_R_B, INPUT_PULLUP);
    
    // Initialize encoder states
    lastStateL = (digitalRead(ENCODER_L_A) << 1) | digitalRead(ENCODER_L_B);
    lastStateR = (digitalRead(ENCODER_R_A) << 1) | digitalRead(ENCODER_R_B);
  #endif
  
  #ifdef USE_OLED
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
    } else {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("ESP32-S2 LFR"));
      display.println(F("12-Channel"));
      display.println(F("Initializing..."));
      display.display();
      delay(2000);
    }
  #endif
  
  #ifdef USE_ROS
    nh.initNode();
    nh.advertise(sensor_pub);
    nh.advertise(error_pub);
    nh.subscribe(cmd_vel_sub);
    
    sensor_msg.data_length = SENSOR_COUNT;
    sensor_msg.data = (int16_t*)malloc(sizeof(int16_t) * SENSOR_COUNT);
  #endif
  
  // Initialize sensor calibration arrays
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
  
  // Calibrate sensors
  calibrateSensors();
  
  // Validate path plan
  validatePathPlan();
  
  // Initialize path execution state
  pathState.junctionsExecuted = 0;
  pathState.junctionsMissed = 0;
  pathState.lastJunctionTime = 0;
  pathState.avgConfidence = 0.0;
  
  // Start in line following mode
  currentMode = MODE_LINE_FOLLOW;
  
  Serial.println("Setup complete!");
}

// ============================================================================
// PATH PLANNING VALIDATION
// ============================================================================

bool validatePathPlan() {
  #if !PATH_VALIDATION
    return true;
  #endif
  
  if (settings.numJunctions == 0) {
    Serial.println("Path Plan: Empty (no junctions programmed)");
    return false;
  }
  
  if (settings.numJunctions > MAX_JUNCTIONS) {
    Serial.print("Path Plan: ERROR - numJunctions (");
    Serial.print(settings.numJunctions);
    Serial.print(") exceeds MAX_JUNCTIONS (");
    Serial.print(MAX_JUNCTIONS);
    Serial.println(")");
    settings.numJunctions = MAX_JUNCTIONS;
    return false;
  }
  
  Serial.print("Path Plan: Valid - ");
  Serial.print(settings.numJunctions);
  Serial.println(" junctions programmed");
  
  return true;
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Check for SELECT button to enter menu
  if (currentMode != MODE_MENU) {
    if (readButton(btnSelect, BUTTON_SELECT)) {
      currentMode = MODE_MENU;
      lastMenuActivity = millis();
      menuIndex = 0;
      inSubMenu = false;
      #ifdef USE_OLED
        showMainMenu();
      #endif
      return;
    }
  }
  
  // Read sensors
  readSensors();
  
  #ifdef USE_ENCODERS
    // Update encoders (single-core, so we do it in main loop)
    updateEncoders();
  #endif
  
  // Check for line inversion transitions
  checkLineInversion();
  
  // Process based on mode
  switch (currentMode) {
    case MODE_MENU:
      #ifdef USE_OLED
        handleMenuNavigation();
      #endif
      break;
      
    case MODE_LINE_FOLLOW:
      // Check for turns (with cooldown to avoid repeated detection)
      if (!turnInProgress && (millis() - lastTurnTime > TURN_COOLDOWN)) {
        currentTurn = detectTurn();
        
        if (currentTurn != TURN_NONE) {
          if (DEBUG_SENSORS) {
            Serial.print("Turn detected: ");
            Serial.println(currentTurn);
          }
          executeTurn(currentTurn);
        }
      }
      
      // Follow line
      if (!turnInProgress) {
        followLine();
      }
      break;
      
    case MODE_MANUAL:
      // Motor control handled by ROS callback
      break;
      
    case MODE_IDLE:
      stopMotors();
      break;
      
    case MODE_CALIBRATE:
      calibrateSensors();
      currentMode = MODE_IDLE;
      break;
  }
  
  // Update display
  #ifdef USE_OLED
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE) {
      updateDisplay();
      lastDisplayUpdate = millis();
    }
  #endif
  
  // Publish ROS data
  #ifdef USE_ROS
    publishROS();
    nh.spinOnce();
  #endif
  
  // Debug output
  if (DEBUG_SENSORS) {
    printSensorData();
  }
  
  delay(LOOP_DELAY);
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

void readSensors() {
  // Read all 12 sensors using multiplexer
  // Configuration 00
  digitalWrite(SELECT_A, LOW);
  digitalWrite(SELECT_B, LOW);
  sensorADC[6] = analogRead(INPUT_A);
  sensorADC[4] = analogRead(INPUT_C);
  sensorADC[1] = analogRead(INPUT_D);
  
  // Configuration 01
  digitalWrite(SELECT_A, LOW);
  digitalWrite(SELECT_B, HIGH);
  sensorADC[7] = analogRead(INPUT_A);
  sensorADC[11] = analogRead(INPUT_B);
  sensorADC[2] = analogRead(INPUT_C);
  sensorADC[0] = analogRead(INPUT_D);
  
  // Configuration 10
  digitalWrite(SELECT_A, HIGH);
  digitalWrite(SELECT_B, LOW);
  sensorADC[9] = analogRead(INPUT_A);
  sensorADC[10] = analogRead(INPUT_B);
  sensorADC[3] = analogRead(INPUT_C);
  
  // Configuration 11
  digitalWrite(SELECT_A, HIGH);
  digitalWrite(SELECT_B, HIGH);
  sensorADC[8] = analogRead(INPUT_A);
  sensorADC[5] = analogRead(INPUT_C);
  
  // Convert to binary based on threshold
  for (int i = 0; i < SENSOR_COUNT; i++) {
    bool detectedLine = (sensorADC[i] < SENSOR_THRESHOLD);
    
    // If in inverted mode, flip the detection logic
    if (invertedMode) {
      sensorBinary[i] = !detectedLine;  // White line on black surface
    } else {
      sensorBinary[i] = detectedLine;   // Black line on white surface
    }
  }
}

void calibrateSensors() {
  #ifdef USE_OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Calibrating..."));
    display.println(F("Move robot over"));
    display.println(F("line & surface"));
    display.display();
  #endif
  
  Serial.println("Calibrating sensors...");
  Serial.println("Move robot over line and surface for 5 seconds");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    readSensors();
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (sensorADC[i] < sensorMin[i]) sensorMin[i] = sensorADC[i];
      if (sensorADC[i] > sensorMax[i]) sensorMax[i] = sensorADC[i];
    }
    
    delay(10);
  }
  
  Serial.println("Calibration complete!");
  
  #ifdef USE_OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Calibration"));
    display.println(F("Complete!"));
    display.display();
    delay(1000);
  #endif
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setMotorSpeeds(int left, int right) {
  // Left motor
  if (left > 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    ledcWrite(PWM_CHANNEL_L, abs(left));
  } else if (left < 0) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_L, abs(left));
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    ledcWrite(PWM_CHANNEL_L, 0);
  }
  
  // Right motor
  if (right > 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(PWM_CHANNEL_R, abs(right));
  } else if (right < 0) {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_R, abs(right));
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(PWM_CHANNEL_R, 0);
  }
  
  leftSpeed = left;
  rightSpeed = right;
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

// ============================================================================
// LINE FOLLOWING FUNCTIONS
// ============================================================================

void followLine() {
  // Calculate weighted position error
  int weights[] = SENSOR_WEIGHTS;
  int weightedSum = 0;
  int activeCount = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorBinary[i]) {
      weightedSum += weights[i];
      activeCount++;
    }
  }
  
  // Check if line is detected
  if (activeCount > 0) {
    error = weightedSum / (float)activeCount;
    lastLineDetected = millis();
    lastValidError = error;
    lineAge = 0;
    lineSearchActive = false;
  } else {
    // Line lost - graduated recovery strategy
    lineAge = millis() - lastLineDetected;
    
    if (lineAge < LINE_LOSS_SEARCH_START) {
      // Phase 1: Continue with last error (0-200ms)
      error = lastValidError;
    } else if (lineAge < LINE_LOSS_SEARCH_WIDE) {
      // Phase 2: Small oscillating search (200-500ms)
      if (!lineSearchActive) {
        lineSearchActive = true;
        searchDirection = (lastValidError > 0) ? 1 : -1;
      }
      error = lastValidError + (searchDirection * 3.0 * sin(millis() / 100.0));
    } else if (lineAge < LINE_LOSS_TIMEOUT) {
      // Phase 3: Wide sweep search (500-1000ms)
      error = lastValidError + (searchDirection * 8.0 * sin(millis() / 150.0));
    } else {
      // Phase 4: Stop if configured (>1000ms)
      if (STOP_ON_LOST) {
        stopMotors();
        return;
      }
      error = lastValidError;  // Keep trying last known direction
    }
  }
  
  // Detect line edges for aggressive correction
  bool atLeftEdge = (sensorBinary[0] || sensorBinary[1]) && 
                    !(sensorBinary[10] || sensorBinary[11]);
  bool atRightEdge = (sensorBinary[10] || sensorBinary[11]) && 
                     !(sensorBinary[0] || sensorBinary[1]);
  float edgeBoost = (atLeftEdge || atRightEdge) ? LINE_EDGE_BOOST : 1.0;
  
  // Adaptive PID gains based on error magnitude
  float currentKp = settings.kp;
  float currentKd = settings.kd;
  
  #if USE_ADAPTIVE_PID
    if (abs(error) > ADAPTIVE_THRESHOLD) {
      currentKp = KP_AGGRESSIVE;
      currentKd = KD_AGGRESSIVE;
    }
  #endif
  
  // Apply edge boost to gains
  currentKp *= edgeBoost;
  currentKd *= edgeBoost;
  
  // Integral calculation with enhanced anti-windup
  // Only integrate when error is reasonable (not lost)
  if (activeCount > 0 && abs(error) < 10) {
    integral += error;
    // Different limits for curves vs straights
    float integralLimit = (abs(error) > SHARP_CURVE_THRESHOLD) ? 
                         INTEGRAL_MAX * 0.5 : INTEGRAL_MAX;
    integral = constrain(integral, -integralLimit, integralLimit);
  } else {
    // Faster decay when line lost or error large
    integral = integral * 0.7;
  }
  
  // Detect error direction change (crossed line center)
  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
    if (abs(error - lastError) > 8.0) {
      // Significant crossover - reset integral
      integral = 0;
    }
  }
  
  // Derivative calculation with spike filtering
  derivative = error - lastError;
  
  // Filter derivative spikes (likely sensor noise)
  if (abs(derivative - lastDerivative) > DERIVATIVE_SPIKE_FILTER) {
    // Smooth out the spike
    derivative = lastDerivative * 0.7 + derivative * 0.3;
  }
  lastDerivative = derivative;
  
  // Apply low-pass filter to derivative
  filteredDerivative = (DERIVATIVE_FILTER * filteredDerivative) + 
                       ((1.0 - DERIVATIVE_FILTER) * derivative);
  
  // PID calculation
  pidOutput = (currentKp * error) + 
              (KI * integral) + 
              (currentKd * filteredDerivative);
  pidOutput = constrain(pidOutput, PID_MIN, PID_MAX);
  
  // Adaptive speed control based on error (curve/straight detection)
  float adaptiveSpeed = settings.baseSpeed;
  if (activeCount > 0) {  // Only adjust speed when line detected
    if (abs(error) > SHARP_CURVE_THRESHOLD) {
      // Sharp curve detected - slow down
      adaptiveSpeed *= SPEED_CURVE_FACTOR;
    } else if (abs(error) < STRAIGHT_THRESHOLD && abs(derivative) < 1.0) {
      // Straight section - speed up
      adaptiveSpeed *= SPEED_STRAIGHT_FACTOR;
    }
  } else if (lineSearchActive) {
    // Slow speed during search
    adaptiveSpeed = SEARCH_OSCILLATE_SPEED;
  }
  
  // Calculate motor speeds with adaptive base speed
  leftSpeed = constrain(adaptiveSpeed - pidOutput, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(adaptiveSpeed + pidOutput, MIN_SPEED, MAX_SPEED);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  lastError = error;
  
  if (DEBUG_PID) {
    Serial.print("Error: "); Serial.print(error);
    Serial.print(" | I: "); Serial.print(integral);
    Serial.print(" | D: "); Serial.print(filteredDerivative);
    Serial.print(" | PID: "); Serial.print(pidOutput);
    Serial.print(" | Speed: "); Serial.print(adaptiveSpeed, 0);
    if (lineSearchActive) Serial.print(" [SEARCH]");
    Serial.print(" | L: "); Serial.print(leftSpeed);
    Serial.print(" | R: "); Serial.println(rightSpeed);
  }
}

// ============================================================================
// TURN DETECTION FUNCTIONS
// ============================================================================

TurnType detectTurn() {
  #if !ENABLE_TURN_DETECTION
    junctionConfidence = 0.0;
    return TURN_NONE;
  #endif
  
  // Count active sensors in different regions
  int leftCount = 0;
  int rightCount = 0;
  int centerCount = 0;
  int totalCount = 0;
  
  // Left region (sensors 0-2)
  for (int i = LEFT_SENSOR_START; i <= LEFT_SENSOR_END; i++) {
    if (sensorBinary[i]) {
      leftCount++;
      totalCount++;
    }
  }
  
  // Right region (sensors 9-11)
  for (int i = RIGHT_SENSOR_START; i <= RIGHT_SENSOR_END; i++) {
    if (sensorBinary[i]) {
      rightCount++;
      totalCount++
;
    }
  }
  
  // Center region (sensors 4-7)
  for (int i = CENTER_SENSOR_START; i <= CENTER_SENSOR_END; i++) {
    if (sensorBinary[i]) {
      centerCount++;
      totalCount++;
    }
  }
  
  // Initialize confidence
  junctionConfidence = 0.0;
  TurnType detectedTurn = TURN_NONE;
  
  // Detect turn patterns with confidence scoring
  // Crossroad: All regions active
  if (leftCount >= TURN_DETECT_THRESHOLD && 
      rightCount >= TURN_DETECT_THRESHOLD && 
      centerCount >= 2) {
    detectedTurn = TURN_CROSSROAD;
    // High confidence if all zones well-represented
    junctionConfidence = 0.6 + (totalCount * CONFIDENCE_SENSOR_WEIGHT);
    junctionConfidence = constrain(junctionConfidence, 0, 1.0);
  }
  // T-Junction: Both left and right active
  else if (leftCount >= TURN_DETECT_THRESHOLD && 
      rightCount >= TURN_DETECT_THRESHOLD) {
    detectedTurn = TURN_T_JUNCTION;
    // Confidence based on left+right balance
    float balance = 1.0 - abs(leftCount - rightCount) / 3.0;
    junctionConfidence = constrain(0.6 + balance * 0.3, 0, 1.0);
  }
  // 90° Left turn: Left sensors heavily active, right sensors minimal
  else if (leftCount >= TURN_DETECT_THRESHOLD && rightCount < 2) {
    detectedTurn = TURN_LEFT;
    junctionConfidence = 0.5 + (leftCount * CONFIDENCE_SENSOR_WEIGHT);
    junctionConfidence = constrain(junctionConfidence, 0, 1.0);
  }
  // 90° Right turn: Right sensors heavily active, left sensors minimal
  else if (rightCount >= TURN_DETECT_THRESHOLD && leftCount < 2) {
    detectedTurn = TURN_RIGHT;
    junctionConfidence = 0.5 + (rightCount * CONFIDENCE_SENSOR_WEIGHT);
    junctionConfidence = constrain(junctionConfidence, 0, 1.0);
  }
  // Sharp left turn: Only leftmost sensors
  else if (leftCount >= 2 && totalCount < 5 && rightCount == 0) {
    detectedTurn = TURN_SHARP_LEFT;
    junctionConfidence = 0.6 + (leftCount * CONFIDENCE_SENSOR_WEIGHT);
  }
  // Sharp right turn: Only rightmost sensors
  else if (rightCount >= 2 && totalCount < 5 && leftCount == 0) {
    detectedTurn = TURN_SHARP_RIGHT;
    junctionConfidence = 0.6 + (rightCount * CONFIDENCE_SENSOR_WEIGHT);
  }
  
  #if DEBUG_SENSORS
    if (detectedTurn != TURN_NONE) {
      Serial.print("Turn detected: ");
      Serial.print(detectedTurn);
      Serial.print(" Confidence: ");
      Serial.println(junctionConfidence, 2);
    }
  #endif
  
  return detectedTurn;
}

void executeTurn(TurnType turnType) {
  if (turnType == TURN_NONE) return;
  
  turnInProgress = true;
  
  // Check if this is a junction (T or crossroad)
  if (turnType == TURN_T_JUNCTION || turnType == TURN_CROSSROAD) {
    // Confidence check for junctions
    #if PATH_VALIDATION
      if (junctionConfidence < JUNCTION_CONFIDENCE_MIN) {
        #if DEBUG_SENSORS
          Serial.print("Junction rejected - low confidence: ");
          Serial.println(junctionConfidence, 2);
        #endif
        turnInProgress = false;
        return;
      }
    #endif
    
    // Check for missed junctions
    #if MISSED_JUNCTION_RECOVERY
      unsigned long timeSinceLastJunction = millis() - pathState.lastJunctionTime;
      if (pathState.lastJunctionTime > 0 && 
          timeSinceLastJunction > MAX_JUNCTION_INTERVAL &&
          currentJunction < settings.numJunctions) {
        // Junction likely missed - auto skip
        #if DEBUG_SENSORS
          Serial.print("Missed junction detected. Skipping junction ");
          Serial.println(currentJunction + 1);
        #endif
        pathState.junctionsMissed++;
        currentJunction++;
      }
    #endif
    
    if (pathPlanEnabled && currentJunction < settings.numJunctions) {
      // Validate array bounds
      if (currentJunction >= MAX_JUNCTIONS) {
        Serial.println("ERROR: currentJunction >= MAX_JUNCTIONS");
        turnInProgress = false;
        return;
      }
      
      // Use programmed action from path plan
      JunctionAction plannedAction = settings.pathPlan[currentJunction];
      
      #if DEBUG_SENSORS
        Serial.print("Junction ");
        Serial.print(currentJunction + 1);
        Serial.print(" -> ");
      #endif
      
      // Convert to turn type
      if (plannedAction == JUNCTION_LEFT) {
        turnType = TURN_LEFT;
        #if DEBUG_SENSORS
          Serial.println("LEFT");
        #endif
      } else if (plannedAction == JUNCTION_RIGHT) {
        turnType = TURN_RIGHT;
        #if DEBUG_SENSORS
          Serial.println("RIGHT");
        #endif
      } else {
        // JUNCTION_STRAIGHT - don't turn
        #if DEBUG_SENSORS
          Serial.println("STRAIGHT");
        #endif
        turnInProgress = false;
        currentJunction++;
        // Update path state
        pathState.junctionsExecuted++;
        pathState.lastJunctionTime = millis();
        pathState.avgConfidence = (pathState.avgConfidence + junctionConfidence) / 2.0;
        return;
      }
      
      currentJunction++;  // Move to next junction
      // Update path state
      pathState.junctionsExecuted++;
      pathState.lastJunctionTime = millis();
      pathState.avgConfidence = (pathState.avgConfidence + junctionConfidence) / 2.0;
    } else {
      // No plan or past end of plan: go straight by default
      #if DEBUG_SENSORS
        if (currentJunction >= settings.numJunctions) {
          Serial.println("Past plan end - going straight");
        }
      #endif
      turnInProgress = false;
      return;
    }
  }
  
  // Execute the turn (regular turns or programmed junction turns)
  #if ENABLE_ENCODER_TURNS
    executeTurnWithEncoders(turnType);
  #else
    executeTurnTimeBased(turnType);
  #endif
  
  // Settle time after turn
  delay(TURN_SETTLE_TIME);
  
  // Reset PID state
  integral = 0;
  lastError = 0;
  filteredDerivative = 0;
  
  turnInProgress = false;
  lastTurnTime = millis();
}

// Encoder-based precise turning
void executeTurnWithEncoders(TurnType turnType) {
  float targetAngle = 0;
  
  // Determine turn angle based on type
  switch (turnType) {
    case TURN_LEFT:
    case TURN_SHARP_LEFT:
      targetAngle = -TURN_ANGLE_90;  // Negative for left
      break;
      
    case TURN_RIGHT:
    case TURN_SHARP_RIGHT:
      targetAngle = TURN_ANGLE_90;   // Positive for right
      break;
      
    default:
      return;
  }
  
  // Calculate required encoder pulses using calibrated value
  long targetPulses = (long)(abs(targetAngle) * settings.pulsesPerDegree);
  
  // Reset encoder counters
  leftPulses = 0;
  rightPulses = 0;
  
  // Perform pivot turn
  if (targetAngle < 0) {
    // Left turn: left wheel backward, right wheel forward
    setMotorSpeeds(-PIVOT_SPEED, PIVOT_SPEED);
  } else {
    // Right turn: left wheel forward, right wheel backward
    setMotorSpeeds(PIVOT_SPEED, -PIVOT_SPEED);
  }
  
  // Wait until target reached or timeout
  unsigned long startTime = millis();
  while (millis() - startTime < TURN_TIMEOUT) {
    updateEncoders();
    
    long avgPulses = (abs(leftPulses) + abs(rightPulses)) / 2;
    
    if (avgPulses >= targetPulses) {
      break;
    }
    
    delay(5);
  }
  
  // Stop motors
  stopMotors();
  delay(50);
  
  if (DEBUG_ENCODERS) {
    Serial.print("Turn complete: L=");
    Serial.print(leftPulses);
    Serial.print(" R=");
    Serial.print(rightPulses);
    Serial.print(" Target=");
    Serial.println(targetPulses);
  }
}

// Time-based turning (fallback if encoders disabled)
void executeTurnTimeBased(TurnType turnType) {
  int turnDuration = 500;  // Default 500ms turn
  
  switch (turnType) {
    case TURN_LEFT:
    case TURN_SHARP_LEFT:
      setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
      break;
      
    case TURN_RIGHT:
    case TURN_SHARP_RIGHT:
      setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
      break;
      
    case TURN_T_JUNCTION:
      if (T_JUNCTION_DEFAULT == TURN_LEFT) {
        setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
      } else if (T_JUNCTION_DEFAULT == TURN_RIGHT) {
        setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
      } else {
        return;
      }
      break;
      
    case TURN_CROSSROAD:
      if (CROSSROAD_DEFAULT == TURN_LEFT) {
        setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
      } else if (CROSSROAD_DEFAULT == TURN_RIGHT) {
        setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
      } else {
        return;
      }
      break;
      
    default:
      return;
  }
  
  delay(turnDuration);
  stopMotors();
}

// ============================================================================
// INVERTED LINE DETECTION FUNCTIONS
// ============================================================================

void checkLineInversion() {
  #if !ENABLE_INVERSION
    return;
  #endif
  
  // Count how many sensors see opposite of expected
  int flippedCount = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    bool rawDetection = (sensorADC[i] < SENSOR_THRESHOLD);
    bool expectedInCurrent = invertedMode ? !rawDetection : rawDetection;
    
    if (expectedInCurrent != sensorBinary[i]) {
      flippedCount++;
    }
  }
  
  // Detect transition to inverted mode
  if (!invertedMode && flippedCount >= INVERSION_THRESHOLD) {
    inversionCounter++;
    normalCounter = 0;
    
    if (inversionCounter >= INVERSION_CONFIRM) {
      invertedMode = true;
      inversionCounter = 0;
      
      Serial.println(">>> INVERTED MODE: Following white line");
      
      #ifdef USE_OLED
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(2);
        display.println("INVERTED");
        display.println("  MODE");
        display.display();
        delay(500);
      #endif
    }
  }
  // Detect transition back to normal mode
  else if (invertedMode && flippedCount >= INVERSION_THRESHOLD) {
    normalCounter++;
    inversionCounter = 0;
    
    if (normalCounter >= INVERSION_HYSTERESIS) {
      invertedMode = false;
      normalCounter = 0;
      
      Serial.println(">>> NORMAL MODE: Following black line");
      
      #ifdef USE_OLED
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(2);
        display.println(" NORMAL");
        display.println("  MODE");
        display.display();
        delay(500);
      #endif
    }
  }
  // Reset counters if stable
  else {
    if (inversionCounter > 0) inversionCounter--;
    if (normalCounter > 0) normalCounter--;
  }
}

// ============================================================================
// ENCODER FUNCTIONS (Single-core implementation)
// ============================================================================

#ifdef USE_ENCODERS
inline int8_t decodeQuadrature(uint8_t last, uint8_t current) {
  if ((last == 0b00 && current == 0b01) ||
      (last == 0b01 && current == 0b11) ||
      (last == 0b11 && current == 0b10) ||
      (last == 0b10 && current == 0b00)) {
    return +1;
  }
  if ((last == 0b00 && current == 0b10) ||
      (last == 0b10 && current == 0b11) ||
      (last == 0b11 && current == 0b01) ||
      (last == 0b01 && current == 0b00)) {
    return -1;
  }
  return 0;
}

void updateEncoders() {
  // Left encoder
  uint8_t currentStateL = (digitalRead(ENCODER_L_A) << 1) | digitalRead(ENCODER_L_B);
  int8_t dirL = decodeQuadrature(lastStateL, currentStateL);
  leftPulses += dirL;
  lastStateL = currentStateL;
  
  // Right encoder
  uint8_t currentStateR = (digitalRead(ENCODER_R_A) << 1) | digitalRead(ENCODER_R_B);
  int8_t dirR = decodeQuadrature(lastStateR, currentStateR);
  rightPulses += dirR;
  lastStateR = currentStateR;
  
  if (DEBUG_ENCODERS) {
    Serial.print("L: "); Serial.print(leftPulses);
    Serial.print(" | R: "); Serial.println(rightPulses);
  }
}
#endif

// ============================================================================
// MENU SYSTEM FUNCTIONS
// ============================================================================

#ifdef USE_OLED
void showMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("=== MAIN MENU ==="));
  display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  
  const char* menuItems[] = {
    "1.Start Follow",
    "2.PID Settings",
    "3.Turn Calib",
    "4.Path Plan",      // Changed from Junction Plan
    "5.Calibrate"
  };
  
  for (int i = 0; i < 5; i++) {
    display.setCursor(0, 14 + i * 10);
    if (i == menuIndex) {
      display.print(F(">"));
    } else {
      display.print(F(" "));
    }
    display.println(menuItems[i]);
  }
  
  display.display();
}

void showPIDSettings() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("== PID SETTINGS =="));
  display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  
  const char* items[] = {"Speed", "KP", "KD", "SAVE & EXIT"};
  
  for (int i = 0; i < 4; i++) {
    display.setCursor(0, 14 + i * 12);
    if (i == subMenuIndex) display.print(F(">"));
    else display.print(F(" "));
    
    display.print(items[i]);
    display.print(F(": "));
    
    if (i == 0) display.print(settings.baseSpeed, 0);
    else if (i == 1) display.print(settings.kp, 2);
    else if (i == 2) display.print(settings.kd, 2);
  }
  
  display.display();
}

void showTurnCalibration() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("= TURN CALIB ="));
  display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  
  display.setCursor(0, 14);
  if (subMenuIndex == 0) display.print(F(">"));
  else display.print(F(" "));
  display.print(F("90\xF7 PPD:"));
  display.println(settings.pulsesPerDegree, 1);
  
  display.setCursor(0, 26);
  if (subMenuIndex == 1) display.print(F(">"));
  else display.print(F(" "));
  display.print(F("Fwd Before:"));
  display.print(settings.forwardBeforeTurn);
  display.println(F("mm"));
  
  display.setCursor(0, 38);
  if (subMenuIndex == 2) display.print(F(">"));
  else display.print(F(" "));
  display.print(F("Fwd After:"));
  display.print(settings.forwardAfterTurn);
  display.println(F("mm"));
  
  display.setCursor(0, 50);
  if (subMenuIndex == 3) display.print(F(">"));
  else display.print(F(" "));
  display.println(F("SAVE & EXIT"));
  
  display.display();
}

void showPathPlanning() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  if (inSubMenu && editingJunction >= 0) {
    // Editing specific junction
    display.print(F("= JUNCTION "));
    display.print(editingJunction + 1);
    display.println(F(" ="));
    display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
    
    const char* actions[] = {"LEFT", "STRAIGHT", "RIGHT"};
    
    for (int i = 0; i < 3; i++) {
      display.setCursor(0, 16 + i * 12);
      if (i == subMenuIndex) display.print(F(">"));
      else display.print(F(" "));
      display.print(i + 1);
      display.print(F(". "));
      display.println(actions[i]);
    }
    
    display.setCursor(0, 56);
    display.println(F("[SELECT to SET]"));
  } else {
    // Junction list view
    display.println(F("= PATH PLAN ="));
    display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
    
    // Show up to 3 junctions at a time
    int startIdx = max(0, menuIndex - 1);
    int endIdx = min(startIdx + 3, MAX_JUNCTIONS);
    
    const char* actions[] = {"L", "S", "R"};  // Short versions
    
    for (int i = startIdx; i < endIdx; i++) {
      display.setCursor(0, 14 + (i - startIdx) * 10);
      if (i == menuIndex) display.print(F(">"));
      else display.print(F(" "));
      
      display.print(F("J"));
      display.print(i + 1);
      display.print(F(": "));
      display.println(actions[settings.pathPlan[i]]);
    }
    
    // Bottom info and controls (adjusted Y positions to fit 64px screen)
    display.setCursor(0, 42);
    if (menuIndex == MAX_JUNCTIONS) display.print(F(">"));
    else display.print(F(" "));
    display.print(F("Curr: "));
    display.println(currentJunction);
    
    display.setCursor(0, 50);
    if (menuIndex == MAX_JUNCTIONS + 1) display.print(F(">"));
    else display.print(F(" "));
    display.print(F("Total: "));
    display.println(settings.numJunctions);
    
    display.setCursor(0, 58);
    if (menuIndex == MAX_JUNCTIONS + 2) {
      display.println(F(">SAVE"));
    } else {
      display.println(F(" SAVE"));
    }
  }
  
  display.display();
}

void handleMenuNavigation() {
  // Read buttons
  bool upPressed = readButton(btnUp, BUTTON_UP);
  bool downPressed = readButton(btnDown, BUTTON_DOWN);
  bool selectPressed = readButton(btnSelect, BUTTON_SELECT);
  
  if (upPressed || downPressed || selectPressed) {
    lastMenuActivity = millis();
  }
  
  // Check timeout
  if (millis() - lastMenuActivity > MENU_TIMEOUT) {
    currentMode = MODE_LINE_FOLLOW;
    currentMenu = MENU_MAIN;
    menuIndex = 0;
    inSubMenu = false;
    return;
  }
  
  // Main menu navigation
  if (!inSubMenu) {
    if (upPressed && menuIndex > 0) menuIndex--;
    if (downPressed && menuIndex < 4) menuIndex++;
    
    if (selectPressed) {
      if (menuIndex == 0) {
        // Start line following
        currentMode = MODE_LINE_FOLLOW;
        currentMenu = MENU_MAIN;
        menuIndex = 0;
      } else if (menuIndex == 1) {
        currentMenu = MENU_PID_SETTINGS;
        subMenuIndex = 0;
        inSubMenu = true;
      } else if (menuIndex == 2) {
        currentMenu = MENU_TURN_CALIBRATION;
        subMenuIndex = 0;
        inSubMenu = true;
      } else if (menuIndex == 3) {
        currentMenu = MENU_JUNCTION_PLANNING;
        subMenuIndex = 0;
        inSubMenu = true;
      } else if (menuIndex == 4) {
        currentMode = MODE_CALIBRATE;
        currentMenu = MENU_MAIN;
        menuIndex = 0;
      }
    }
  }
  // Submenu navigation
  else {
    if (currentMenu == MENU_PID_SETTINGS) {
      // Navigate between items
      if (upPressed && subMenuIndex > 0) {
        subMenuIndex--;
      }
      if (downPressed && subMenuIndex < 3) {
        subMenuIndex++;
      }
      
      if (selectPressed) {
        if (subMenuIndex == 3) {
          // Save and exit
          saveSettings();
          inSubMenu = false;
          currentMenu = MENU_MAIN;
          subMenuIndex = 0;
        } else {
          // Enter edit mode (could add visual indicator here)
          // For now, UP/DOWN will adjust values directly
        }
      }
      
      // Handle long press for faster value adjustment
      bool longPressUp = (btnUp.pressed == LOW && (millis() - btnUp.pressTime) > LONG_PRESS_TIME);
      bool longPressDown = (btnDown.pressed == LOW && (millis() - btnDown.pressTime) > LONG_PRESS_TIME);
      int increment = (longPressUp || longPressDown) ? 10 : 1;
      
      // Adjust values when holding button (not just on press)
      if (btnUp.pressed == LOW && subMenuIndex < 3) {
        if (subMenuIndex == 0) {
          settings.baseSpeed = constrain(settings.baseSpeed + (increment * 5), 50, 255);
        } else if (subMenuIndex == 1) {
          settings.kp = constrain(settings.kp + 0.1, 0, 10);
        } else if (subMenuIndex == 2) {
          settings.kd = constrain(settings.kd + 0.1, 0, 10);
        }
        delay(100);  // Delay for smooth adjustment
      }
      
      if (btnDown.pressed == LOW && subMenuIndex < 3) {
        if (subMenuIndex == 0) {
          settings.baseSpeed = constrain(settings.baseSpeed - (increment * 5), 50, 255);
        } else if (subMenuIndex == 1) {
          settings.kp = constrain(settings.kp - 0.1, 0, 10);
        } else if (subMenuIndex == 2) {
          settings.kd = constrain(settings.kd - 0.1, 0, 10);
        }
        delay(100);  // Delay for smooth adjustment
      }
    }
    else if (currentMenu == MENU_TURN_CALIBRATION) {
      // Navigate between items
      if (upPressed && subMenuIndex > 0) {
        subMenuIndex--;
      }
      if (downPressed && subMenuIndex < 3) {
        subMenuIndex++;
      }
      
      if (selectPressed) {
        if (subMenuIndex == 3) {
          // Save and exit
          saveSettings();
          inSubMenu = false;
          currentMenu = MENU_MAIN;
          subMenuIndex = 0;
        }
      }
      
      // Handle long press for faster value adjustment
      bool longPressUp = (btnUp.pressed == LOW && (millis() - btnUp.pressTime) > LONG_PRESS_TIME);
      bool longPressDown = (btnDown.pressed == LOW && (millis() - btnDown.pressTime) > LONG_PRESS_TIME);
      int increment = (longPressUp || longPressDown) ? 5 : 1;
      
      // Adjust values when holding button
      if (btnUp.pressed == LOW && subMenuIndex < 3) {
        if (subMenuIndex == 0) {
          settings.pulsesPerDegree += 0.1 * increment;
          settings.pulsesPerDegree = constrain(settings.pulsesPerDegree, 0.1, 50);
        } else if (subMenuIndex == 1) {
          settings.forwardBeforeTurn = constrain(settings.forwardBeforeTurn + (5 * increment), 0, 500);
        } else if (subMenuIndex == 2) {
          settings.forwardAfterTurn = constrain(settings.forwardAfterTurn + (5 * increment), 0, 500);
        }
        delay(100);
      }
      
      if (btnDown.pressed == LOW && subMenuIndex < 3) {
        if (subMenuIndex == 0) {
          settings.pulsesPerDegree -= 0.1 * increment;
          settings.pulsesPerDegree = constrain(settings.pulsesPerDegree, 0.1, 50);
        } else if (subMenuIndex == 1) {
          settings.forwardBeforeTurn = constrain(settings.forwardBeforeTurn - (5 * increment), 0, 500);
        } else if (subMenuIndex == 2) {
          settings.forwardAfterTurn = constrain(settings.forwardAfterTurn - (5 * increment), 0, 500);
        }
        delay(100);
      }
    }
    else if (currentMenu == MENU_JUNCTION_PLANNING) {
      if (inSubMenu && editingJunction >= 0) {
        // Editing specific junction action
        if (upPressed && subMenuIndex > 0) subMenuIndex--;
        if (downPressed && subMenuIndex < 2) subMenuIndex++;
        
        if (selectPressed) {
          // Set junction action
          settings.pathPlan[editingJunction] = (JunctionAction)subMenuIndex;
          
          // Update numJunctions if needed
          if (editingJunction >= settings.numJunctions) {
            settings.numJunctions = editingJunction + 1;
          }
          
          // Return to junction list
          inSubMenu = false;
          editingJunction = -1;
        }
      } else {
        // Junction list navigation - special handling for Curr and Total
        if (menuIndex == MAX_JUNCTIONS) {
          // On "Curr:" - only adjust value, no navigation
          if (upPressed && currentJunction < MAX_JUNCTIONS) {
            currentJunction++;
          } else if (downPressed && currentJunction > 0) {
            currentJunction--;
          }
          
          // SELECT to move to Total
          if (selectPressed) {
            menuIndex = MAX_JUNCTIONS + 1;
          }
        } else if (menuIndex == MAX_JUNCTIONS + 1) {
          // On "Total:" - only adjust value, no navigation
          if (upPressed && settings.numJunctions < MAX_JUNCTIONS) {
            settings.numJunctions++;
          } else if (downPressed && settings.numJunctions > 0) {
            settings.numJunctions--;
          }
          
          // SELECT to move to Save
          if (selectPressed) {
            menuIndex = MAX_JUNCTIONS + 2;
          }
        } else {
          // Normal junction list navigation
          if (upPressed && menuIndex > 0) menuIndex--;
          if (downPressed && menuIndex < MAX_JUNCTIONS + 2) menuIndex++;
          
          if (selectPressed) {
            if (menuIndex < MAX_JUNCTIONS) {
              // Edit junction
              editingJunction = menuIndex;
              subMenuIndex = settings.pathPlan[menuIndex];  // Current action
              inSubMenu = true;
            } else if (menuIndex == MAX_JUNCTIONS + 2) {
              // Save and exit
              saveSettings();
              inSubMenu = false;
              currentMenu = MENU_MAIN;
              menuIndex = 0;
            }
          }
        }
      }
    }
  }
  
  // Update display
  if (currentMenu == MENU_MAIN && !inSubMenu) {
    showMainMenu();
  } else if (currentMenu == MENU_PID_SETTINGS) {
    showPIDSettings();
  } else if (currentMenu == MENU_TURN_CALIBRATION) {
    showTurnCalibration();
  } else if (currentMenu == MENU_JUNCTION_PLANNING) {
    showPathPlanning();
  }
}
#endif

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

#ifdef USE_OLED
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // Title
  display.println(F("12CH LFR"));
  display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  
  // Mode and Path Progress
  display.setCursor(0, 14);
  display.print(F("Mode: "));
  switch (currentMode) {
    case MODE_IDLE: display.println(F("IDLE")); break;
    case MODE_LINE_FOLLOW: 
      display.print(F("AUTO"));
      if (invertedMode) {
        display.print(F(" INV"));
      }
      display.println();
      break;
    case MODE_MANUAL: display.println(F("MANUAL")); break;
    case MODE_CALIBRATE: display.println(F("CALIB")); break;
  }
  
  // Path planning progress (if enabled and planned)
  if (pathPlanEnabled && settings.numJunctions > 0) {
    display.setCursor(70, 14);
    display.print(F("P:"));
    display.print(currentJunction);
    display.print(F("/"));
    display.print(settings.numJunctions);
  }
  
  // Sensor visualization
  display.setCursor(0, 24);
  display.print(F("Sensors:"));
  display.setCursor(0, 34);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorBinary[i]) {
      display.fillRect(i * 10, 34, 8, 8, SSD1306_WHITE);
    } else {
      display.drawRect(i * 10, 34, 8, 8, SSD1306_WHITE);
    }
  }
  
  // Error and speeds
  display.setCursor(0, 46);
  display.print(F("Err:"));
  display.print(error, 1);
  
  // Turn indicator
  if (turnInProgress) {
    display.setCursor(60, 46);
    display.print(F("TURN"));
  }
  
  display.setCursor(0, 56);
  display.print(F("L:"));
  display.print(leftSpeed);
  display.print(F(" R:"));
  display.print(rightSpeed);
  
  display.display();
}
#endif

// ============================================================================
// ROS FUNCTIONS
// ============================================================================

#ifdef USE_ROS
void publishROS() {
  // Publish sensor data
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensor_msg.data[i] = sensorADC[i];
  }
  sensor_pub.publish(&sensor_msg);
  
  // Publish error
  error_msg.data = error;
  error_pub.publish(&error_msg);
}
#endif

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

void printSensorData() {
  Serial.print("Sensors: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorADC[i]);
    Serial.print(" ");
  }
  Serial.print("| Binary: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorBinary[i]);
  }
  Serial.println();
}
