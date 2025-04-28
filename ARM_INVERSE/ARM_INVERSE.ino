#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define PI 3.1415926535


#define PWM_ADDR 0x40  //address for pwm driver

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR);  

// 
const float l0 = 5.58;   // Base to shoulder height (cm)
const float l1 = 11.625;   // Shoulder to elbow length (cm)
const float l2 = 22.0;  // Elbow to gripper center length (cm)

const int SERVO_MIN_PULSE = 600;   // 0°
const int SERVO_MAX_PULSE = 2400;  // 180°

const int servo_channels[6] = {0, 1, 2, 3, 4, 5};

float home_x = 0, home_y = 0, home_z = 0; //home

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  pwm.begin();  
  pwm.setPWMFreq(50);  // Set the PWM frequency to 50Hz 

  for (int i = 0; i < 6; i++) {
    setServoAngle(i, 90);  // Initialize all servos to 90°
  }

  Serial.println("Enter coordinates as 'x,y,z' (e.g., '0,0,27.475')");
  Serial.println("To set the home position, enter 'home'");
  Serial.println("Then move the arm to the position you want to be the 'home' (0,0,0) and press Enter.");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "home") {
      // manually set home (0,0,0)
      Serial.println("Manually position the arm at 0,0,0. Press Enter when done.");
      while (!Serial.available()) {
        delay(100);  
      }
      // 
      home_x = 0;  // Set the home position 
      home_y = 0;
      home_z = 0;
      Serial.println("Home position set to (0,0,0). Now enter coordinates.");
    } else {
      // Process coordinates for new home position
      float x, y, z;
      int comma1 = input.indexOf(',');
      int comma2 = input.indexOf(',', comma1 + 1);
      if (comma1 > 0 && comma2 > comma1) {
        x = input.substring(0, comma1).toFloat() + home_x; // Add home offset
        y = input.substring(comma1 + 1, comma2).toFloat() + home_y; 
        z = input.substring(comma2 + 1).toFloat() + home_z; 

        // Calculate rand theta0 
        float r = sqrt(x * x + y * y);
        float theta0 = (r > 0) ? atan2(y, x) : 0;  

        // Calculate p and q based on the x, y, z coordinates
        float p = r;
        float q = z - l0;  
        float d = sqrt(p * p + q * q);

        Serial.println("Debug: r=" + String(r) + ", q=" + String(q) + ", d=" + String(d));

        if (d > l1 + l2 || d < fabs(l1 - l2)) {
          Serial.println("Position unreachable");
        } else {
          // Calculate theta2 (elbow angle) using the law of cosines
          float cos_theta2 = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2);
          if (cos_theta2 < -1 || cos_theta2 > 1) {
            Serial.println("Position unreachable");
          } else {
            float theta2 = acos(cos_theta2);  // Elbow angle (0° to 180°)
            float alpha = atan2(q, p);
            float beta = atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
            float theta1 = alpha + beta;
            float theta1_adjusted = theta1 - PI / 2;  
            float theta2_adjusted = theta2 - PI / 2;  

            float theta0_deg = theta0 * 180 / PI;  // Base rotation angle (J0)
            float theta1_deg = theta1_adjusted * 180 / PI;  
            float theta2_deg = theta2_adjusted * 180 / PI;  

            // Servo mappings
            float servo0_angle = 90 + theta0_deg;  // J0: -90° to 90° → 0° to 180°
            float servo1_angle = theta1_deg;       // J1: 0° to 90° → 0° to 90°
            float servo2_angle = 90 + theta2_deg;  // J2: 0° to 90° → 90° to 180°
            float servo3_angle = 90;
            float servo4_angle = 90;
            float servo5_angle = 90;

            servo0_angle = constrain(servo0_angle, 0, 180);
            servo1_angle = constrain(servo1_angle, 0, 90);
            servo2_angle = constrain(servo2_angle, 90, 180);

            setServoAngle(0, servo0_angle);
            setServoAngle(1, servo1_angle);
            setServoAngle(2, servo2_angle);
            setServoAngle(3, servo3_angle);
            setServoAngle(4, servo4_angle);
            setServoAngle(5, servo5_angle);

            Serial.println("Moved to (" + String(x) + ", " + String(y) + ", " + String(z) + ")");
            Serial.println("Angles: J0=" + String(theta0_deg) + ", J1=" + String(theta1_deg) + ", J2=" + String(theta2_deg));
            Serial.println("Servo: J0=" + String(servo0_angle) + ", J1=" + String(servo1_angle) + ", J2=" + String(servo2_angle));
          }
        }
      } else {
        Serial.println("Invalid input. Use 'x,y,z'");
      }
    }
  }
}

void setServoAngle(int servo_num, float angle) {
  int channel = servo_channels[servo_num];
  int pulse_width = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  int off = (pulse_width * 4096) / 20000;
  pwm.setPWM(channel, 0, off);
}

