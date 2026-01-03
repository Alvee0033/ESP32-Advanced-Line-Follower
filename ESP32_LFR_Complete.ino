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
    // Load saved settings
    settings.version = storedVersion;
    settings.baseSpeed = preferences.getFloat("baseSpeed", BASE_SPEED);
    settings.kp = preferences.getFloat("kp", KP);
    settings.kd = preferences.getFloat("kd", KD);
    settings.pulsesPerDegree = preferences.getFloat("ppd", PULSES_PER_DEGREE);
    settings.forwardBeforeTurn = preferences.getInt("fwdBefore", FORWARD_BEFORE_TURN);
    settings.forwardAfterTurn = preferences.getInt("fwdAfter", FORWARD_AFTER_TURN);
    
    // Load path plan
    settings.numJunctions = preferences.getUChar("numJunc", 0);
    for (int i = 0; i < MAX_JUNCTIONS; i++) {
      char key[16];
      sprintf(key, "junc%d", i);
      settings.pathPlan[i] = (JunctionAction)preferences.getUChar(key, JUNCTION_STRAIGHT);
    }
    
    Serial.println("Settings loaded from EEPROM");
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
  
  // Start in line following mode
  currentMode = MODE_LINE_FOLLOW;
  
  Serial.println("Setup complete!");
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
  } else {
    // Line lost - use last error direction
    if (millis() - lastLineDetected > LOST_LINE_TIMEOUT) {
      if (STOP_ON_LOST) {
        stopMotors();
        return;
      }
    }
    // Keep last error to continue in the same direction
  }
  
  // Adaptive PID gains based on error magnitude
  float currentKp = settings.kp;
  float currentKd = settings.kd;
  
  #if USE_ADAPTIVE_PID
    if (abs(error) > ADAPTIVE_THRESHOLD) {
      currentKp = KP_AGGRESSIVE;
      currentKd = KD_AGGRESSIVE;
    }
  #endif
  
  // Integral calculation with anti-windup
  // Only integrate when error is reasonable (not lost)
  if (activeCount > 0 && abs(error) < 10) {
    integral += error;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  } else {
    // Reset integral when line is lost or error too large
    integral = integral * 0.8;  // Decay instead of hard reset
  }
  
  // Derivative calculation with low-pass filter
  derivative = error - lastError;
  filteredDerivative = (DERIVATIVE_FILTER * filteredDerivative) + 
                       ((1.0 - DERIVATIVE_FILTER) * derivative);
  
  // PID calculation
  pidOutput = (currentKp * error) + 
              (KI * integral) + 
              (currentKd * filteredDerivative);
  pidOutput = constrain(pidOutput, PID_MIN, PID_MAX);
  
  // Calculate motor speeds with smooth control - use settings.baseSpeed
  leftSpeed = constrain(settings.baseSpeed - pidOutput, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(settings.baseSpeed + pidOutput, MIN_SPEED, MAX_SPEED);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  lastError = error;
  
  if (DEBUG_PID) {
    Serial.print("Error: "); Serial.print(error);
    Serial.print(" | I: "); Serial.print(integral);
    Serial.print(" | D: "); Serial.print(filteredDerivative);
    Serial.print(" | PID: "); Serial.print(pidOutput);
    Serial.print(" | L: "); Serial.print(leftSpeed);
    Serial.print(" | R: "); Serial.println(rightSpeed);
  }
}

// ============================================================================
// TURN DETECTION FUNCTIONS
// ============================================================================

TurnType detectTurn() {
  #if !ENABLE_TURN_DETECTION
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
      totalCount++;
    }
  }
  
  // Center region (sensors 4-7)
  for (int i = CENTER_SENSOR_START; i <= CENTER_SENSOR_END; i++) {
    if (sensorBinary[i]) {
      centerCount++;
      totalCount++;
    }
  }
  
  // Detect turn patterns
  // Crossroad: All regions active
  if (leftCount >= TURN_DETECT_THRESHOLD && 
      rightCount >= TURN_DETECT_THRESHOLD && 
      centerCount >= 2) {
    return TURN_CROSSROAD;
  }
  
  // T-Junction: Both left and right active
  if (leftCount >= TURN_DETECT_THRESHOLD && 
      rightCount >= TURN_DETECT_THRESHOLD) {
    return TURN_T_JUNCTION;
  }
  
  // 90° Left turn: Left sensors heavily active, right sensors minimal
  if (leftCount >= TURN_DETECT_THRESHOLD && rightCount < 2) {
    return TURN_LEFT;
  }
  
  // 90° Right turn: Right sensors heavily active, left sensors minimal
  if (rightCount >= TURN_DETECT_THRESHOLD && leftCount < 2) {
    return TURN_RIGHT;
  }
  
  // Sharp left turn: Only leftmost sensors
  if (leftCount >= 2 && totalCount < 5 && rightCount == 0) {
    return TURN_SHARP_LEFT;
  }
  
  // Sharp right turn: Only rightmost sensors
  if (rightCount >= 2 && totalCount < 5 && leftCount == 0) {
    return TURN_SHARP_RIGHT;
  }
  
  return TURN_NONE;
}

void executeTurn(TurnType turnType) {
  if (turnType == TURN_NONE) return;
  
  turnInProgress = true;
  
  // Check if this is a junction (T or crossroad)
  if (turnType == TURN_T_JUNCTION || turnType == TURN_CROSSROAD) {
    if (pathPlanEnabled && currentJunction < settings.numJunctions) {
      // Use programmed action from path plan
      JunctionAction plannedAction = settings.pathPlan[currentJunction];
      
      Serial.print("Junction ");
      Serial.print(currentJunction + 1);
      Serial.print(" -> ");
      
      // Convert to turn type
      if (plannedAction == JUNCTION_LEFT) {
        turnType = TURN_LEFT;
        Serial.println("LEFT");
      } else if (plannedAction == JUNCTION_RIGHT) {
        turnType = TURN_RIGHT;
        Serial.println("RIGHT");
      } else {
        // JUNCTION_STRAIGHT - don't turn
        Serial.println("STRAIGHT");
        turnInProgress = false;
        currentJunction++;  // Still increment counter
        return;
      }
      
      currentJunction++;  // Move to next junction
    } else {
      // No plan or past end of plan: go straight by default
      if (currentJunction >= settings.numJunctions) {
        Serial.println("Past plan end - going straight");
      }
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
    
    // Bottom info
    display.setCursor(0, 48);
    display.print(F("Total: "));
    display.println(settings.numJunctions);
    
    display.setCursor(0, 56);
    if (menuIndex == MAX_JUNCTIONS) {
      display.println(F(">RESET  SAVE"));
    } else if (menuIndex == MAX_JUNCTIONS + 1) {
      display.println(F(" RESET >SAVE"));
    } else {
      display.println(F(" RESET  SAVE"));
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
      if (upPressed) {
        if (subMenuIndex > 0) subMenuIndex--;
        else {
          // Adjust values
          if (subMenuIndex == 0) settings.baseSpeed = constrain(settings.baseSpeed + 5, 50, 255);
          else if (subMenuIndex == 1) settings.kp = constrain(settings.kp + 0.1, 0, 5);
          else if (subMenuIndex == 2) settings.kd = constrain(settings.kd + 0.1, 0, 3);
        }
      }
      if (downPressed) {
        if (subMenuIndex < 3) {
          // Adjust values
          if (subMenuIndex == 0) settings.baseSpeed = constrain(settings.baseSpeed - 5, 50, 255);
          else if (subMenuIndex == 1) settings.kp = constrain(settings.kp - 0.1, 0, 5);
          else if (subMenuIndex == 2) settings.kd = constrain(settings.kd - 0.1, 0, 3);
        } else if (subMenuIndex < 3) {
          subMenuIndex++;
        }
      }
      if (selectPressed) {
        if (subMenuIndex == 3) {
          saveSettings();
          inSubMenu = false;
          currentMenu = MENU_MAIN;
        } else {
          subMenuIndex++;
        }
      }
    }
    else if (currentMenu == MENU_TURN_CALIBRATION) {
      if (upPressed) {
        if (subMenuIndex > 0) subMenuIndex--;
        else {
          if (subMenuIndex == 0) settings.pulsesPerDegree += 1.0;
          else if (subMenuIndex == 1) settings.forwardBeforeTurn = constrain(settings.forwardBeforeTurn + 5, 0, 200);
          else if (subMenuIndex == 2) settings.forwardAfterTurn = constrain(settings.forwardAfterTurn + 5, 0, 200);
        }
      }
      if (downPressed) {
        if (subMenuIndex < 3) {
          if (subMenuIndex == 0) settings.pulsesPerDegree = constrain(settings.pulsesPerDegree - 1.0, 1, 20);
          else if (subMenuIndex == 1) settings.forwardBeforeTurn = constrain(settings.forwardBeforeTurn - 5, 0, 200);
          else if (subMenuIndex == 2) settings.forwardAfterTurn = constrain(settings.forwardAfterTurn - 5, 0, 200);
        } else if (subMenuIndex < 3) {
          subMenuIndex++;
        }
      }
      if (selectPressed) {
        if (subMenuIndex == 3) {
          saveSettings();
          inSubMenu = false;
          currentMenu = MENU_MAIN;
        } else {
          subMenuIndex++;
        }
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
        // Junction list navigation
        if (upPressed && menuIndex > 0) menuIndex--;
        if (downPressed && menuIndex < MAX_JUNCTIONS + 1) menuIndex++;
        
        if (selectPressed) {
          if (menuIndex < MAX_JUNCTIONS) {
            // Edit junction
            editingJunction = menuIndex;
            subMenuIndex = settings.pathPlan[menuIndex];  // Current action
            inSubMenu = true;
          } else if (menuIndex == MAX_JUNCTIONS) {
            // Reset counter
            currentJunction = 0;
            display.clearDisplay();
            display.setCursor(20, 28);
            display.setTextSize(1);
            display.println("COUNTER RESET!");
            display.display();
            delay(1000);
          } else if (menuIndex == MAX_JUNCTIONS + 1) {
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
