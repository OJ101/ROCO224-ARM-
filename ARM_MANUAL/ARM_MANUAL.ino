#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN 150
#define SERVO_MAX 600
#define STEP_SIZE 5     // Degrees to move per key press
#define TOTAL_SERVOS 6

int servoAngles[TOTAL_SERVOS] = {90, 90, 90, 90, 90, 65};  // Default start angles

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); // Standard 50Hz for servos
  Serial.println("Servo Controller Ready. Use keys QW ER TY AS DF GH.");
  // Move servos to initial position
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    moveServo(i, servoAngles[i]);
  }
}

void loop() {
  if (Serial.available()) {
    char key = Serial.read();
    handleKey(key);
  }
}

void moveServo(int servoID, int angle) {
  angle = constrain(angle, 0, 180);
  servoAngles[servoID] = angle;
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servoID, 0, pulse);

  Serial.print("Servo ");
  Serial.print(servoID);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println(" degrees");
}
//detects which keys are pressed
void handleKey(char key) {
  switch (key) {
    case 'Q': moveServo(0, servoAngles[0] + STEP_SIZE); break;
    case 'W': moveServo(0, servoAngles[0] - STEP_SIZE); break;
    case 'E': moveServo(1, servoAngles[1] + STEP_SIZE); break;
    case 'R': moveServo(1, servoAngles[1] - STEP_SIZE); break;
    case 'T': moveServo(2, servoAngles[2] + STEP_SIZE); break;
    case 'Y': moveServo(2, servoAngles[2] - STEP_SIZE); break;
    case 'A': moveServo(3, servoAngles[3] + STEP_SIZE); break;
    case 'S': moveServo(3, servoAngles[3] - STEP_SIZE); break;
    case 'D': moveServo(4, servoAngles[4] + STEP_SIZE); break;
    case 'F': moveServo(4, servoAngles[4] - STEP_SIZE); break;
    case 'G': moveServo(5, servoAngles[5] + STEP_SIZE); break;
    case 'H': moveServo(5, servoAngles[5] - STEP_SIZE); break;
    default:
      Serial.println("Invalid key. Use QW ER TY AS DF GH.");
      break;
  }
}
