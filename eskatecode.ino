// Code for LucasMakes off road skateboard (which turned into an off road buggy) project.


#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Arduino pin assignments
const int THROTTLE_IN_PIN = 2;
const int STEER_IN_PIN    = 3;
const int KILL_SWITCH_PIN = 4;
const int SPEED_SW_PIN    = 5;   
const int OP_MODE_SW_PIN  = 6;   
const int REVERSE_PIN     = 8;
const int PWM_OUT_PIN     = 9;
const int BRAKE_SERVO_PIN = 10;
const int STEER_SERVO_PIN = 11;

// RC signal values
const int RC_NEUTRAL      = 1507;
const int RC_MIN_VAL      = 976;
const int RC_MAX_VAL      = 2040;
const int RC_DEADBAND     = 23;

const int SW_LOW_MAX      = 1300; 
const int SW_HIGH_MIN     = 1700; 

// throttle values
const int PWM_IDLE_MIN    = 29;    
const int PWM_ENGAGE_MIN  = 40;    
const int PWM_MAX_SAFE    = 124;   
const int PWM_WALK_MAX    = 65;    

// brake and servo
const int BRAKE_SERVO_OFF = 120;     
const int BRAKE_SERVO_MAX = 90;    

const int STEER_SERVO_LEFT   = 45;
const int STEER_SERVO_CENTER = 90;
const int STEER_SERVO_RIGHT  = 135;

Servo brakeServo;
Servo steerServo;

enum OpMode { MODE_RC, MODE_PERSON, MODE_PARK };

void setup() {
  pinMode(THROTTLE_IN_PIN, INPUT);
  pinMode(STEER_IN_PIN, INPUT);
  pinMode(SPEED_SW_PIN, INPUT);
  pinMode(OP_MODE_SW_PIN, INPUT);
  
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  
  pinMode(PWM_OUT_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);

  brakeServo.attach(BRAKE_SERVO_PIN);
  brakeServo.write(BRAKE_SERVO_OFF);

  steerServo.attach(STEER_SERVO_PIN);
  steerServo.write(STEER_SERVO_CENTER);

  digitalWrite(REVERSE_PIN, LOW);
  analogWrite(PWM_OUT_PIN, PWM_IDLE_MIN);

  Serial.begin(115200);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

void loop() {
  bool physicalKillSwitched = (digitalRead(KILL_SWITCH_PIN) == LOW);

  unsigned long rawThrottle = pulseIn(THROTTLE_IN_PIN, HIGH, 25000);
  unsigned long rawSteer    = pulseIn(STEER_IN_PIN, HIGH, 25000);
  unsigned long rawSpeedSw  = pulseIn(SPEED_SW_PIN, HIGH, 25000);
  unsigned long rawOpModeSw = pulseIn(OP_MODE_SW_PIN, HIGH, 25000);

  static unsigned long throttlePulse = RC_NEUTRAL;
  static unsigned long steerPulse    = RC_NEUTRAL;
  static unsigned long speedSwPulse  = RC_NEUTRAL;
  static unsigned long opModeSwPulse = RC_MAX_VAL; 
  static unsigned long lastValidTime = 0;

  bool hasSignal = (rawThrottle > 800 && rawThrottle < 2200);

  if (hasSignal) {
    throttlePulse = rawThrottle;
    if (rawSteer > 800) steerPulse = rawSteer;
    if (rawSpeedSw > 800) speedSwPulse = rawSpeedSw;
    if (rawOpModeSw > 800) opModeSwPulse = rawOpModeSw;
    lastValidTime = millis();
  } 
  
  bool isFailsafe = (millis() - lastValidTime > 250);

  OpMode currentMode;
  if (opModeSwPulse < SW_LOW_MAX) {
    currentMode = MODE_RC;
  } else if (opModeSwPulse > SW_HIGH_MIN) {
    currentMode = MODE_PARK;
  } else {
    currentMode = MODE_PERSON;
  }

  bool isWalkMode = (speedSwPulse < SW_LOW_MAX);
  int activeMaxPwm = isWalkMode ? PWM_WALK_MAX : PWM_MAX_SAFE;

  bool isKilled = false;
  if (currentMode == MODE_PERSON && physicalKillSwitched) {
    isKilled = true;
  }

  int targetThrottlePwm = PWM_IDLE_MIN;
  int targetBrakeAngle  = BRAKE_SERVO_OFF;

  // Core logic
  if (isFailsafe) {
    digitalWrite(REVERSE_PIN, LOW);
    targetThrottlePwm = PWM_IDLE_MIN;
    targetBrakeAngle  = BRAKE_SERVO_MAX; 
    steerServo.write(STEER_SERVO_CENTER);
  } 
  else if (currentMode == MODE_PARK || isKilled) {
    digitalWrite(REVERSE_PIN, LOW);
    targetThrottlePwm = PWM_IDLE_MIN;
    targetBrakeAngle  = BRAKE_SERVO_MAX; 
    
    long boundedSteer = constrain(steerPulse, RC_MIN_VAL, RC_MAX_VAL);
    steerServo.write(map(boundedSteer, RC_MIN_VAL, RC_MAX_VAL, STEER_SERVO_LEFT, STEER_SERVO_RIGHT));
  }
  else {
    // Normal driving
    if (throttlePulse < (RC_NEUTRAL - RC_DEADBAND)) { 
      digitalWrite(REVERSE_PIN, LOW);
      long boundedPulse = constrain(throttlePulse, RC_MIN_VAL, RC_NEUTRAL - RC_DEADBAND); 
      targetThrottlePwm = map(boundedPulse, RC_NEUTRAL - RC_DEADBAND, RC_MIN_VAL, PWM_ENGAGE_MIN, activeMaxPwm); 
      targetBrakeAngle  = BRAKE_SERVO_OFF;
    }
    else if (throttlePulse > (RC_NEUTRAL + RC_DEADBAND)) { 
      digitalWrite(REVERSE_PIN, HIGH);
      targetThrottlePwm = PWM_IDLE_MIN; 
      long boundedPulse = constrain(throttlePulse, RC_NEUTRAL + RC_DEADBAND, RC_MAX_VAL); 
      targetBrakeAngle  = map(boundedPulse, RC_NEUTRAL + RC_DEADBAND, RC_MAX_VAL, BRAKE_SERVO_OFF, BRAKE_SERVO_MAX); 
    }
    else {
      digitalWrite(REVERSE_PIN, LOW);
      targetThrottlePwm = PWM_IDLE_MIN;
      targetBrakeAngle  = BRAKE_SERVO_OFF;
    }

    long boundedSteer = constrain(steerPulse, RC_MIN_VAL, RC_MAX_VAL);
    steerServo.write(map(boundedSteer, RC_MIN_VAL, RC_MAX_VAL, STEER_SERVO_LEFT, STEER_SERVO_RIGHT));
  }

  // Write brake angle 
  brakeServo.write(targetBrakeAngle);

  // Throttle ramping
  static int currentPwm = PWM_IDLE_MIN;
  static unsigned long lastRamp = 0;
  if (millis() - lastRamp >= 10) {
    lastRamp = millis();
    if (currentPwm < targetThrottlePwm) {
      currentPwm = min(currentPwm + 4, targetThrottlePwm);
    } else if (currentPwm > targetThrottlePwm) {
      currentPwm = max(currentPwm - 6, targetThrottlePwm);
    }
    if ((isKilled || isFailsafe || currentMode == MODE_PARK) && currentPwm > PWM_IDLE_MIN) {
      currentPwm = PWM_IDLE_MIN;
    }
  }
  analogWrite(PWM_OUT_PIN, currentPwm);

  // Update OLED display
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 200) {
    lastDisplayUpdate = millis();
    
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    if (currentMode == MODE_RC) display.print("OP: RC");
    else if (currentMode == MODE_PERSON) display.print("OP: PERSON");
    else display.print("OP: PARK");

    display.setCursor(0, 24);
    display.print("SPD: ");
    display.print(isWalkMode ? "WALK" : "MAX");

    display.setCursor(0, 48);
    if (isFailsafe) {
      display.print("! FAILSAFE !");
    } else if (isKilled) {
      display.print("! KILLED !");
    } else {
      display.print("SYS: OK");
    }

    display.display();
  }

  // Serial debug
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint >= 100) {
    lastSerialPrint = millis();
    Serial.print("Raw Kill Pin: ");
    Serial.print(physicalKillSwitched ? "LOW (Pulled)" : "HIGH (Attached)");
    Serial.print(" | System Killed: ");
    Serial.println(isKilled ? "YES" : "NO");
  }
}