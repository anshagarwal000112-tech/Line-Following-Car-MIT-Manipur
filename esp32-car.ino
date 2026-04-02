/*
 * ============================================================
 * ESP32 PID Line Following Robot - Fixed for Arduino Core v3.0+
 * ============================================================
 * Track: Black line on white background
 * Sensors: 0 = Black (line), 1 = White (no line)
 * ============================================================
 */

// ==================== PIN DEFINITIONS ====================
// Motor Driver (L298N)
#define IN1    21    // Left Motor Direction A
#define IN2    22    // Left Motor Direction B  
#define IN3    25    // Right Motor Direction A
#define IN4    33    // Right Motor Direction B
#define ENA    23    // Left Motor PWM
#define ENB    32    // Right Motor PWM
#define STBY   19    // Motor Driver Enable (HIGH = ON)

// IR Sensors (S1=Leftmost -> S5=Rightmost)
#define S1     26
#define S2     27
#define S3     14
#define S4     4
#define S5     13

// ==================== PWM SETTINGS (ESP32 Core 3.0+ Syntax) ====================
#define PWM_FREQ       5000
#define PWM_BITS       8

// ==================== PID TUNING PARAMETERS ====================
float Kp = 18.0;      
float Ki = 0.002;     
float Kd = 25.0;      

// ==================== SPEED SETTINGS ====================
int baseSpeed = 255;          
int maxSpeed = 255;           
int sharpTurnSpeed = 255;     

// ==================== GLOBAL VARIABLES ====================
float error = 0;
float prevError = 0;
float integral = 0;
bool turnMode = false;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  // Sensor pins
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  
  // PWM setup for ESP32 Core 3.0+ (New Syntax)
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
  
  
  Serial.println("========================================");
  Serial.println("   ESP32 PID Line Follower Ready (v3)");
  Serial.println("========================================");
  Serial.println("Place robot on line to start...");
  
  // Wait for line detection
  while(digitalRead(S1) && digitalRead(S2) && digitalRead(S3) && 
        digitalRead(S4) && digitalRead(S5)) {
    delay(50);
  }
  
  Serial.println("Line detected! Starting in 2s...");
  delay(2000);
}

// ==================== MAIN LOOP ====================
void loop() {
  // Read sensors: 0=Black(line), 1=White(no line)
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);
  
  // Create pattern: S1=bit4, S2=bit3, S3=bit2, S4=bit1, S5=bit0
  int pattern = (s1 << 4) | (s2 << 3) | (s3 << 2) | (s4 << 1) | s5;
  
  // ==================== PATTERN HANDLING ====================
  switch(pattern) {
    
    // ---- CROSS JUNCTION (all sensors on line) ----
    case 0b00000:
      handleCrossJunction();
      break;
      
    // ---- LEFT 90 DEGREE TURN (3-4 sensors on left side) ----
    case 0b00011:  // S1+S2+S3 on line
    case 0b00001:  // S1+S2+S3+S4 on line  
      handleLeft90();
      break;
      
    // ---- RIGHT 90 DEGREE TURN (3-4 sensors on right side) ----
    case 0b11000:  // S3+S4+S5 on line
    case 0b10000:  // S2+S3+S4+S5 on line
      handleRight90();
      break;
      
    // ---- LINE LOST ----
    case 0b11111:
      handleLineLost();
      break;
      
    // ---- NORMAL PID LINE FOLLOWING ----
    default:
      turnMode = false;
      error = calculateError(pattern);
      applyPID();
      break;
  }
  
  // Debug output
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint >= 100) {
    Serial.print("S:");
    Serial.print(s1);Serial.print(s2);Serial.print(s3);
    Serial.print(s4);Serial.print(s5);
    Serial.print(" Err:"); Serial.print(error,1);
    Serial.print(" "); Serial.println(turnMode ? "TURN" : "PID");
    lastPrint = millis();
  }
  
  prevError = error;
  delay(0.002);
}

// ==================== ERROR CALCULATION ====================
float calculateError(int pattern) {
  switch(pattern) {
    case 0b11011: return 0;           // Only S3
    case 0b10011: return -0.7;        // S2+S3
    case 0b11001: return 0.7;         // S3+S4
    case 0b10111: return -1.5;        // Only S2
    case 0b11101: return 1.5;         // Only S4
    case 0b00111: return -2.5;        // S1+S2
    case 0b11100: return 2.5;         // S4+S5
    case 0b01111: return -3.5;        // Only S1
    case 0b11110: return 3.5;         // Only S5
    default: return prevError;        // Fallback
  }
}

// ==================== PID CONTROL ====================
void applyPID() {
  integral += error;
  if(integral > 30) integral = 30;
  if(integral < -30) integral = -30;
  
  if(abs(error) < 0.5) integral *= 0.95;
  
  float derivative = error - prevError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Adaptive speed: slow down for curves automatically
  float absErr = abs(error);
  float speedFactor;
  
  if(absErr <= 0.8)      speedFactor = 1.0;    
  else if(absErr <= 1.5) speedFactor = 0.90;   
  else if(absErr <= 2.5) speedFactor = 0.78;   
  else                   speedFactor = 0.65;   
  
  int effectiveSpeed = baseSpeed * speedFactor;
  
  int leftSpeed = effectiveSpeed + correction;
  int rightSpeed = effectiveSpeed - correction;
  
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);
  
  if(leftSpeed > 0 && leftSpeed < 55) leftSpeed = 55;
  if(rightSpeed > 0 && rightSpeed < 55) rightSpeed = 55;
  
  setMotor(IN1, IN2, ENA, leftSpeed);
  setMotor(IN3, IN4, ENB, rightSpeed);
}

// ==================== LEFT 90 DEGREE TURN ====================
void handleLeft90() {
  turnMode = true;
  integral = 0; 
  
  setMotor(IN1, IN2, ENA, -sharpTurnSpeed);
  setMotor(IN3, IN4, ENB, sharpTurnSpeed + 30);
  
  error = -5;  
}

// ==================== RIGHT 90 DEGREE TURN ====================
void handleRight90() {
  turnMode = true;
  integral = 0;  
  
  setMotor(IN1, IN2, ENA, sharpTurnSpeed + 30);
  setMotor(IN3, IN4, ENB, -sharpTurnSpeed);
  
  error = 5;  
}

// ==================== CROSS JUNCTION ====================
void handleCrossJunction() {
  setMotor(IN1, IN2, ENA, baseSpeed);
  setMotor(IN3, IN4, ENB, baseSpeed);
  error = 0;
  integral = 0;
}

// ==================== LINE LOST ====================
void handleLineLost() {
  integral = 0;
  
  if(prevError <= 0) {
    setMotor(IN1, IN2, ENA, -100);
    setMotor(IN3, IN4, ENB, 110);
    error = -6;
  } else {
    setMotor(IN1, IN2, ENA, 110);
    setMotor(IN3, IN4, ENB, -100);
    error = 6;
  }
}

// ==================== SET MOTOR SPEED ====================
void setMotor(int in1Pin, int in2Pin, int pwmPin, int speed) {
  if(speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmPin, constrain(speed, 0, 255));
  } 
  else if(speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmPin, constrain(-speed, 0, 255));
  } 
  else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmPin, 0);
  }
}

// ==================== STOP MOTORS ====================
// Removed by Chinglen
