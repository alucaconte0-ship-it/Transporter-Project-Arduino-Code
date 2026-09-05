#include "MeMegaPi.h"

// ===== HARDWARE =====
MeMegaPiDCMotor gripperMotor(PORT1A);
MeMegaPiDCMotor liftMotor(PORT3B);
MeMegaPiDCMotor leftMotor(PORT1B);
MeMegaPiDCMotor rightMotor(PORT2B);
MeUltrasonicSensor ultrasonic(PORT_6);

// ===== PARAMETERS =====
// GRIPPER SETTINGS
uint8_t closeSpeed = 130;
uint8_t openSpeed = 130;
const int CLOSE_TIME = 2000;
const int OPEN_TIME = 2000;

// LIFT SETTINGS
uint8_t liftSpeed = 100;
const int LOWER_TIME = 300;
const int RAISE_TIME = 500;

// DRIVE SETTINGS
int driveSpeed = 80; 
int turnSpeed = 155;    //WAS 120 for MCST building //150 works for HAML tables
int approachSpeed = 200; 
int creepingSpeed = 80;

// SCANNING PARAMETERS
const int SCAN_TURN_TIME = 250; //ms
const int SCAN_POSITIONS_FULL_ROTATION = 5;  // turns 90° from testing
const int STABILIZATION_DELAY = 300; 
const int READINGS_PER_POSITION = 5; //reads position with ultrasonic 5 times
const int READING_INTERVAL = 200; //every 200ms

// REPOSITIONING
const int DIAGONAL_TURN_TIME = 500;  
const int MOVE_TO_CENTER_TIME = 8000; //needs testing

// DISTANCE THRESHOLDS
const float INITIAL_GRIP_DISTANCE = 15.0;
const float MIN_VALID_DISTANCE = 2.0;
const float TARGET_DETECT_MIN = 10.0; //target must be at lease 10cm away
const float TARGET_DETECT_MAX = 100.0; //maximum 100cm away 
const float FINAL_APPROACH_DISTANCE = 4.0; //distance at which robot stops and palces block
const float CREEPING_DISTANCE = 15.0; //distance at which robot starts creeping

// ===== STATES =====
enum RobotState {
  INITIAL_GRIP,
  LIFT_AND_WAIT,
  INCREMENTAL_SCAN,
  REPOSITION_DIAGONAL,
  MOVE_TO_CENTER,
  CENTER_360_SCAN,
  APPROACH_TARGET,
  CREEP,
  PLACE_BLOCKS,
  RETREAT,
  MISSION_COMPLETE
};

RobotState currentState = INITIAL_GRIP;
float currentDistance = 0;
int   scanPosition    = 0;
bool  targetFound     = false;

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  delay(2000);
}

// ===== MAIN LOOP =====
void loop() {
  float rawDistance = ultrasonic.distanceCm();
  if (rawDistance > 0 && rawDistance < 400)
    currentDistance = rawDistance;

  switch (currentState) {
    case INITIAL_GRIP:        initialGrip();          break;
    case LIFT_AND_WAIT:       liftAndWait();           break;
    case INCREMENTAL_SCAN:    incrementalScan();       break;
    case REPOSITION_DIAGONAL: repositionDiagonal();    break;
    case MOVE_TO_CENTER:      moveToCenter();          break;
    case CENTER_360_SCAN:     center360Scan();         break;
    case APPROACH_TARGET:     approachTarget();        break;
    case CREEP:               creepTowardsTarget();    break;
    case PLACE_BLOCKS:        placeBlocks();           break;
    case RETREAT:             retreatFromTarget();     break;
    case MISSION_COMPLETE:    missionComplete();       break;
  }

  delay(50);
}

// ===== STATE FUNCTIONS =====

void initialGrip() {
  static bool checked = false;

  if (!checked) {
    Serial.print("dist: ");
    Serial.println(currentDistance);

    if (currentDistance <= INITIAL_GRIP_DISTANCE && currentDistance >= MIN_VALID_DISTANCE
        || currentDistance < MIN_VALID_DISTANCE) {
      lowerGripper();
      closeGripper();
      currentState = LIFT_AND_WAIT;
    } else {
      Serial.println("no blocks, retrying");
      delay(2000);
      return;
    }
    checked = true;
  }
}

void liftAndWait() {
  static bool lifted = false;

  if (!lifted) {
    raiseGripper();
    delay(2000);
    currentState  = INCREMENTAL_SCAN;
    scanPosition  = 0;
    targetFound   = false;
    lifted        = true;
  }
}

void incrementalScan() {
  static bool  isScanning   = false;
  static int   readingCount = 0;
  static float totalDistance = 0;
  static int   validReadings = 0;

  if (!isScanning && scanPosition > 0) {
    turnLeft(turnSpeed);
    delay(SCAN_TURN_TIME);
    stopMotors();
    delay(STABILIZATION_DELAY);
  }

  if (!isScanning) {
    Serial.print("scan pos ");
    Serial.println(scanPosition);
    isScanning    = true;
    readingCount  = 0;
    totalDistance = 0;
    validReadings = 0;
  }

  if (readingCount < READINGS_PER_POSITION) {
    delay(READING_INTERVAL);
    if (currentDistance >= TARGET_DETECT_MIN && currentDistance <= TARGET_DETECT_MAX) {
      totalDistance += currentDistance;
      validReadings++;
    }
    readingCount++;
    return;
  }

  isScanning = false;

  if (validReadings >= 3) {
    Serial.println("target found");
    targetFound  = true;
    currentState = APPROACH_TARGET;
    return;
  }

  scanPosition++;

  if (scanPosition >= SCAN_POSITIONS_FULL_ROTATION) {
    Serial.println("scan done, no target - repositioning");
    currentState = REPOSITION_DIAGONAL;
  }
}

void repositionDiagonal() {
  static bool repositioned = false;

  if (!repositioned) {
    turnRight(turnSpeed);
    delay(DIAGONAL_TURN_TIME);
    stopMotors();
    delay(500);
    currentState  = MOVE_TO_CENTER;
    repositioned  = true;
  }
}

void moveToCenter() {
  static bool moved = false;

  if (!moved) {
    moveForward(driveSpeed);
    delay(MOVE_TO_CENTER_TIME);
    stopMotors();
    delay(500);
    currentState = CENTER_360_SCAN;
    scanPosition = 0;
    moved        = true;
  }
}

void center360Scan() {
  static bool  isScanning   = false;
  static int   readingCount = 0;
  static float totalDistance = 0;
  static int   validReadings = 0;

  if (!isScanning && scanPosition > 0) {
    turnLeft(turnSpeed);
    delay(SCAN_TURN_TIME);
    stopMotors();
    delay(STABILIZATION_DELAY);
  }

  if (!isScanning) {
    Serial.print("center scan pos ");
    Serial.println(scanPosition);
    isScanning    = true;
    readingCount  = 0;
    totalDistance = 0;
    validReadings = 0;
  }

  if (readingCount < READINGS_PER_POSITION) {
    delay(READING_INTERVAL);
    if (currentDistance >= TARGET_DETECT_MIN && currentDistance <= TARGET_DETECT_MAX) {
      totalDistance += currentDistance;
      validReadings++;
    }
    readingCount++;
    return;
  }

  isScanning = false;

  if (validReadings >= 3) {
    Serial.print("target at ");
    Serial.println(totalDistance / validReadings);
    currentState = APPROACH_TARGET;
    return;
  }

  scanPosition++;
  if (scanPosition >= SCAN_POSITIONS_FULL_ROTATION) {
    Serial.println("360 done, no target - restarting");
    scanPosition = 0;
  }
}

void approachTarget() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    Serial.print("dist: ");
    Serial.println(currentDistance);
    lastPrint = millis();
  }

  if (currentDistance <= CREEPING_DISTANCE && currentDistance >= MIN_VALID_DISTANCE) {
    currentState = CREEP;
    return;
  }

  if (currentDistance > TARGET_DETECT_MAX) {
    Serial.println("target lost");
    stopMotors();
    delay(500);
    currentState = CENTER_360_SCAN;
    scanPosition = 0;
    return;
  }

  moveForward(approachSpeed);
}

void creepTowardsTarget() {
  if (currentDistance <= FINAL_APPROACH_DISTANCE && currentDistance >= MIN_VALID_DISTANCE) {
    stopMotors();
    delay(500);
    currentState = PLACE_BLOCKS;
    return;
  }
  moveForward(creepingSpeed);
}

void placeBlocks() {
  lowerGripper();
  delay(500);
  openGripper();
  delay(500);
  raiseGripperEnd();
  currentState = RETREAT;
}

void retreatFromTarget() {
  moveBackward(driveSpeed);
  delay(1000);
  stopMotors();
  currentState = MISSION_COMPLETE;
}

void missionComplete() {
  static bool printed = false;
  if (!printed) {
    Serial.println("done");
    printed = true;
  }
  stopMotors();
}

// ===== MOVEMENT =====

void moveForward(int speed) {
  leftMotor.run(speed);
  rightMotor.run(-speed);
}

void moveBackward(int speed) {
  leftMotor.run(-speed);
  rightMotor.run(speed);
}

void turnRight(int speed) {
  leftMotor.run(speed);
  rightMotor.run(speed);
}

void turnLeft(int speed) {
  leftMotor.run(-speed);
  rightMotor.run(-speed);
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

// ===== GRIPPER =====

void closeGripper() {
  gripperMotor.run(closeSpeed);
  delay(CLOSE_TIME);
  gripperMotor.stop();
}

void openGripper() {
  gripperMotor.run(-openSpeed);
  delay(OPEN_TIME);
  gripperMotor.stop();
}

// ===== LIFT =====

void lowerGripper() {
  liftMotor.run(-liftSpeed);
  delay(LOWER_TIME);
  liftMotor.stop();
}

void raiseGripper() {
  liftMotor.run(liftSpeed);
  delay(RAISE_TIME);
  liftMotor.stop();
}

void raiseGripperEnd() {
  liftMotor.run(liftSpeed);
  delay(150);
  liftMotor.stop();
}
