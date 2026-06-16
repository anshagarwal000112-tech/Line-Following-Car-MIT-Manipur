#include <L298N.h>

#define IN1    21
#define IN2    22
#define ENA    23
#define IN3    25
#define IN4    33
#define ENB    32
#define STBY   19

#define S1     26
#define S2     27
#define S3     14
#define S4     4
#define S5     13

L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

float Kp = 15.0;
float Ki = 0.0;
float Kd = 55.0;

int baseSpeed = 255;
int maxSpeed = 255;
int minMotorSpeed = 60;

unsigned long prevLoopTime = 0;
const unsigned long loopInterval = 5000;
float derivativeFilterAlpha = 0.9;
float filteredDerivative = 0.0;

int rawPattern = 0b11111;
int stablePattern = 0b11111;
int debounceCount = 0;
const int DEBOUNCE_LIMIT = 3;

enum RobotState { NORMAL_PID, LEFT_90, RIGHT_90, CROSS_JUNC, LOST };
RobotState state = NORMAL_PID;

float error = 0;
float prevError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  pinMode(S1, INPUT); pinMode(S2, INPUT);
  pinMode(S3, INPUT); pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  
  motor_drive(0, 0);
  
  Serial.println("Place on line to launch...");
  
  while(readRawSensors() == 0b11111) { 
    delay(50); 
  }
  
  Serial.println("Line locked! Starting in 2s...");
  delay(2000); 
  
  prevLoopTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime - prevLoopTime < loopInterval) {
    return; 
  }
  prevLoopTime = currentTime;

  rawPattern = readRawSensors();
  updateDebounce();
  
  switch(state) {
    case NORMAL_PID: handleNormalPID(); break;
    case LEFT_90:    handleLeft90(); break;
    case RIGHT_90:   handleRight90(); break;
    case CROSS_JUNC: handleCrossJunction(); break;
    case LOST:       handleLineLost(); break;
  }
}

int readRawSensors() {
  return (digitalRead(S1) << 4) | (digitalRead(S2) << 3) | 
         (digitalRead(S3) << 2) | (digitalRead(S4) << 1) | digitalRead(S5);
}

void updateDebounce() {
  bool isSpecialCase = (rawPattern == 0b00000 || rawPattern == 0b00011 || 
                        rawPattern == 0b00001 || rawPattern == 0b11000 || 
                        rawPattern == 0b10000 || rawPattern == 0b11111);

  if (isSpecialCase) {
    if (rawPattern == stablePattern) debounceCount++;
    else { stablePattern = rawPattern; debounceCount = 1; }
  } else {
    stablePattern = rawPattern;
    debounceCount = 0;
  }
}

void handleNormalPID() {
  if (debounceCount >= DEBOUNCE_LIMIT) {
    switch(stablePattern) {
      case 0b00000: state = CROSS_JUNC; return;
      case 0b00011: 
      case 0b00001: state = LEFT_90; return;
      case 0b11000: 
      case 0b10000: state = RIGHT_90; return;
      case 0b11111: state = LOST; return;
    }
  }

  float newError = calculateError(stablePattern);
  
  if (abs(newError - error) > 2.0) {
    error = newError * 0.5; 
    prevError = error;      
  } else {
    error = newError;
  }
  
  applyPID();
  prevError = error;
}

void handleLeft90() {
  integral = 0; filteredDerivative = 0; error = -5.0;
  motor_drive(-140, 170);
  if (stablePattern != 0b00011 && stablePattern != 0b00001) {
    state = NORMAL_PID; prevError = -5.0; 
  }
}

void handleRight90() {
  integral = 0; filteredDerivative = 0; error = 5.0;
  motor_drive(170, -140);
  if (stablePattern != 0b11000 && stablePattern != 0b10000) {
    state = NORMAL_PID; prevError = 5.0; 
  }
}

void handleCrossJunction() {
  integral = 0; filteredDerivative = 0; error = 0;
  motor_drive(baseSpeed, baseSpeed);
  if (stablePattern != 0b00000) { state = NORMAL_PID; prevError = 0; }
}

void handleLineLost() {
  integral = 0; filteredDerivative = 0;
  if (prevError <= 0) {
    motor_drive(-baseSpeed * 0.6, baseSpeed); error = -6.0;
  } else {
    motor_drive(baseSpeed, -baseSpeed * 0.6); error = 6.0;
  }
  if (stablePattern != 0b11111) { state = NORMAL_PID; }
}

float calculateError(int pattern) {
  switch(pattern) {
    case 0b11011: return 0.0;           
    case 0b10011: return -0.7;        
    case 0b11001: return 0.7;         
    case 0b10111: return -1.5;        
    case 0b11101: return 1.5;         
    case 0b00111: return -2.5;        
    case 0b11100: return 2.5;         
    case 0b01111: return -3.5;        
    case 0b11110: return 3.5;         
    default: return prevError; 
  }
}

void applyPID() {
  integral += error;
  if(integral > 30) integral = 30;
  if(integral < -30) integral = -30;
  if(abs(error) < 0.5) integral *= 0.95;
  
  float rawDerivative = error - prevError;
  filteredDerivative = (derivativeFilterAlpha * rawDerivative) + ((1.0 - derivativeFilterAlpha) * filteredDerivative);
  float correction = (Kp * error) + (Ki * integral) + (Kd * filteredDerivative);
  
  float absErr = abs(error);
  float speedFactor = (absErr <= 0.8) ? 1.0 : (absErr <= 1.5) ? 0.95 : (absErr <= 2.5) ? 0.88 : 0.82;   
  
  int effectiveSpeed = baseSpeed * speedFactor;
  int leftSpeed = constrain(effectiveSpeed + correction, 0, maxSpeed);
  int rightSpeed = constrain(effectiveSpeed - correction, 0, maxSpeed);
  
  if(leftSpeed > 0 && leftSpeed < minMotorSpeed) leftSpeed = minMotorSpeed;
  if(rightSpeed > 0 && rightSpeed < minMotorSpeed) rightSpeed = minMotorSpeed;
  
  motor_drive(leftSpeed, rightSpeed);
}

void motor_drive(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    motor1.setSpeed(constrain(leftSpeed, 0, 255));
    motor1.forward();
  } else if (leftSpeed < 0) {
    motor1.setSpeed(constrain(abs(leftSpeed), 0, 255));
    motor1.backward();
  } else {
    motor1.setSpeed(0);
    motor1.stop();
  }

  if (rightSpeed > 0) {
    motor2.setSpeed(constrain(rightSpeed, 0, 255));
    motor2.forward();
  } else if (rightSpeed < 0) {
    motor2.setSpeed(constrain(abs(rightSpeed), 0, 255));
    motor2.backward();
  } else {
    motor2.setSpeed(0);
    motor2.stop();
  }
}
