#include <Arduino.h>

// ---------------- Motor Pins ----------------
#define M1INA 2
#define M1INB 4
#define M1PWM 9
#define M1EN  6

#define M2INA 7
#define M2INB 8
#define M2PWM 10
#define M2EN  12

// ---------------- Encoder Pins ----------------
#define LEFT_ENC_A 3
#define LEFT_ENC_B 5
#define RIGHT_ENC_A 11
#define RIGHT_ENC_B 13

#define ENCODER_CPR 6256    // 17 PPR x 92 gearbox x 4
#define LOOP_INTERVAL 10    // PID loop interval in ms
#define MAX_PWM 255

// ---------------- PID Constants (tune as needed) ----------------
float Kp = 0.5;
float Ki = 0.02;
float Kd = 0.1;

// ---------------- Encoder Variables ----------------
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long last_left_ticks = 0;
long last_right_ticks = 0;

// ---------------- PID State ----------------
float left_target = 0;     // target velocity in ticks/sec
float right_target = 0;
float left_integral = 0;
float right_integral = 0;
float left_prev_error = 0;
float right_prev_error = 0;

int left_pwm = 0;
int right_pwm = 0;

// ---------------- Encoder ISRs ----------------
void leftEncoderISR() {
  bool A = digitalRead(LEFT_ENC_A);
  bool B = digitalRead(LEFT_ENC_B);
  if (A == B) left_ticks++; else left_ticks--;
}

void rightEncoderISR() {
  bool A = digitalRead(RIGHT_ENC_A);
  bool B = digitalRead(RIGHT_ENC_B);
  if (A == B) right_ticks++; else right_ticks--;
}

// ---------------- Motor Control ----------------
void setLeftMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed >= 0) { digitalWrite(M1INA,HIGH); digitalWrite(M1INB,LOW); analogWrite(M1PWM,speed);}
  else { digitalWrite(M1INA,LOW); digitalWrite(M1INB,HIGH); analogWrite(M1PWM,-speed);}
}

void setRightMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed >= 0) { digitalWrite(M2INA,HIGH); digitalWrite(M2INB,LOW); analogWrite(M2PWM,speed);}
  else { digitalWrite(M2INA,LOW); digitalWrite(M2INB,HIGH); analogWrite(M2PWM,-speed);}
}

// ---------------- Serial Command ----------------
#define CMD_BUFFER 32
char cmd[CMD_BUFFER];
uint8_t cmd_index = 0;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(57600);  // Must match ROS plugin baud rate

  // Motor pins
  pinMode(M1INA, OUTPUT); pinMode(M1INB, OUTPUT); pinMode(M1PWM, OUTPUT); pinMode(M1EN, OUTPUT); digitalWrite(M1EN,HIGH);
  pinMode(M2INA, OUTPUT); pinMode(M2INB, OUTPUT); pinMode(M2PWM, OUTPUT); pinMode(M2EN, OUTPUT); digitalWrite(M2EN,HIGH);

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
}

// ---------------- Main Loop ----------------
unsigned long last_loop = 0;
void loop() {
  // Serial handling
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') {
      cmd[cmd_index] = '\0';
      processCommand(cmd);
      cmd_index = 0;
    } else if (cmd_index < CMD_BUFFER-1) {
      cmd[cmd_index++] = c;
    }
  }

  // PID loop
  unsigned long now = millis();
  if (now - last_loop >= LOOP_INTERVAL) {
    last_loop = now;
    float dt = LOOP_INTERVAL / 1000.0;

    // Compute actual wheel velocities (ticks/sec)
    long left_delta  = left_ticks - last_left_ticks;
    long right_delta = right_ticks - last_right_ticks;
    float left_vel  = left_delta / dt;
    float right_vel = right_delta / dt;
    last_left_ticks = left_ticks;
    last_right_ticks = right_ticks;

    // -------- PID Left Wheel --------
    float error = left_target - left_vel;
    left_integral += error * dt;
    float derivative = (error - left_prev_error)/dt;
    left_pwm += Kp*error + Ki*left_integral + Kd*derivative;
    left_prev_error = error;

    // -------- PID Right Wheel --------
    error = right_target - right_vel;
    right_integral += error * dt;
    derivative = (error - right_prev_error)/dt;
    right_pwm += Kp*error + Ki*right_integral + Kd*derivative;
    right_prev_error = error;

    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);
  }
}

// ---------------- Process Serial Command ----------------
void processCommand(char* s) {
  if (s[0] == 'm') {
    int l=0,r=0;
    sscanf(s,"m %d %d",&l,&r);
    // Convert plugin PWM command to target ticks/sec
    left_target  = l * 20.0;  // tune this scale factor
    right_target = r * 20.0;
  } else if (s[0] == 'e') {
    Serial.print(left_ticks);
    Serial.print(" ");
    Serial.println(right_ticks);
  } else if (s[0] == 'u') {
    // Parse PID tuning
    float p,d,i,o;
    sscanf(s,"u %f:%f:%f:%f",&p,&d,&i,&o);
    Kp=p; Kd=d; Ki=i;
  } else if (s[0] == '\0') {
    // heartbeat
  }
}
