#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>


// ============================================================
// Arduino Uno R3 + Pololu Dual VNH5019 Shield + Hall Encoders
//
// Notes for Uno R3:
// - The VNH5019 shield uses the default shield pins automatically
//   when stacked onto the Uno.
// - Uno R3 only has normal external interrupts on D2 and D3.
// - D2 is already used by the shield (M1INA), so we keep:
//      LEFT encoder A  on D3  -> external interrupt
//      RIGHT encoder A on D11 -> pin-change interrupt
// ============================================================

// ---------------- Shield default motor pins ----------------
#define M1INA 2
#define M1INB 4
#define M1PWM 9
#define M1EN  6   // EN/DIAG (fault line from shield)

#define M2INA 7
#define M2INB 8
#define M2PWM 10
#define M2EN  12  // EN/DIAG (fault line from shield)

// ---------------- Encoder pins ----------------
#define LEFT_ENC_A   3   // INT1 on Uno
#define LEFT_ENC_B   5
#define RIGHT_ENC_A 11   // PCINT3 on Uno (Port B)
#define RIGHT_ENC_B 13

#define ENCODER_CPR   6256L   // Keep synced with your ROS config / real decoding mode
#define LOOP_INTERVAL 33UL    // ms, chosen to match ros2_control loop_rate ~= 30 Hz
#define MAX_PWM       255
#define TARGET_SCALE  (1000.0f / (float)LOOP_INTERVAL)

// ---------------- PID constants ----------------
float Kp = 0.15f;
float Ki = 0.0f;
float Kd = 0.0f;

// ---------------- Encoder state ----------------
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long last_left_ticks = 0;
long last_right_ticks = 0;

// ---------------- PID state ----------------
float left_target = 0.0f;   // target velocity in ticks/sec
float right_target = 0.0f;

float left_integral = 0.0f;
float right_integral = 0.0f;
float left_prev_error = 0.0f;
float right_prev_error = 0.0f;

int left_pwm = 0;
int right_pwm = 0;

// ---------------- Serial command buffer ----------------
#define CMD_BUFFER 32
char cmd[CMD_BUFFER];
uint8_t cmd_index = 0;
bool last_char_was_newline = false;

unsigned long last_loop = 0;

// ---------------- Encoder ISRs ----------------
void leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  if (a == b) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  if (a == b) {
    right_ticks--;
  } else {
    right_ticks++;
  }
}

// Pin-change interrupt vector for D8..D13 on Uno/Nano (PORTB)
ISR(PCINT0_vect) {
  rightEncoderISR();
}

// ---------------- Helpers ----------------
void setupRightEncoderPinChangeInterrupt() {
  // D11 on Uno = PB3 = PCINT3
  PCICR |= _BV(PCIE0);     // enable pin-change interrupt group for PORTB
  PCMSK0 |= _BV(PCINT3);   // enable interrupt source for D11 only
}

void readEncoderTicksAtomic(long &left, long &right) {
  noInterrupts();
  left = left_ticks;
  right = right_ticks;
  interrupts();
}

void resetPidState() {
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_prev_error = 0.0f;
  right_prev_error = 0.0f;
  left_pwm = 0;
  right_pwm = 0;
}

// ---------------- Motor control ----------------
void setLeftMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed == 0) {
    digitalWrite(M1INA, LOW);
    digitalWrite(M1INB, LOW);
    analogWrite(M1PWM, 0);
  } else if (speed > 0) {
    digitalWrite(M1INA, HIGH);
    digitalWrite(M1INB, LOW);
    analogWrite(M1PWM, speed);
  } else {
    digitalWrite(M1INA, LOW);
    digitalWrite(M1INB, HIGH);
    analogWrite(M1PWM, -speed);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed == 0) {
    digitalWrite(M2INA, LOW);
    digitalWrite(M2INB, LOW);
    analogWrite(M2PWM, 0);
  } else if (speed > 0) {
    digitalWrite(M2INA, LOW);
    digitalWrite(M2INB, HIGH);
    analogWrite(M2PWM, speed);
  } else {
    digitalWrite(M2INA, HIGH);
    digitalWrite(M2INB, LOW);
    analogWrite(M2PWM, -speed);
  }
}

// ---------------- Serial command handling ----------------
bool parseMotorCommand(char *s, int &l, int &r) {
  if (s[0] != 'm' || s[1] != ' ') {
    return false;
  }

  char *p = s + 2;
  char *end = NULL;

  long lv = strtol(p, &end, 10);
  if (end == p) {
    return false;
  }

  while (*end == ' ') {
    end++;
  }

  long rv = strtol(end, &p, 10);
  if (p == end) {
    return false;
  }

  l = (int)lv;
  r = (int)rv;
  return true;
}

bool parsePidCommand(char *s, float &p_out, float &d_out, float &i_out, float &o_out) {
  if (s[0] != 'u' || s[1] != ' ') {
    return false;
  }

  char *p = s + 2;
  char *end = NULL;

  // On AVR/Uno, strtof is often unavailable. strtod works, and on Uno
  // double is the same size as float, so this is a safe replacement.
  p_out = (float)strtod(p, &end);
  if (end == p || *end != ':') {
    return false;
  }
  p = end + 1;

  d_out = (float)strtod(p, &end);
  if (end == p || *end != ':') {
    return false;
  }
  p = end + 1;

  i_out = (float)strtod(p, &end);
  if (end == p || *end != ':') {
    return false;
  }
  p = end + 1;

  o_out = (float)strtod(p, &end);
  return end != p;
}

void processCommand(char *s) {
  if (s[0] == 'm') {
    int l = 0;
    int r = 0;
    if (parseMotorCommand(s, l, r)) {
      // Keep this protocol compatible with your RPi side.
      // diffdrive_arduino sends motor targets in encoder counts per control loop.
      // Convert that to counts/sec to compare against measured encoder velocity.
      left_target = l * TARGET_SCALE;
      right_target = r * TARGET_SCALE;

      if (l == 0 && r == 0) {
        resetPidState();
        setLeftMotor(0);
        setRightMotor(0);
      }
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
  } else if (s[0] == 'e') {
    long left_now = 0;
    long right_now = 0;
    readEncoderTicksAtomic(left_now, right_now);
    Serial.print(left_now);
    Serial.print(' ');
    Serial.println(right_now);
  } else if (s[0] == 'u') {
    float p = 0.0f, d = 0.0f, i = 0.0f, o = 0.0f;
    if (parsePidCommand(s, p, d, i, o)) {
      Kp = p;
      Kd = d;
      Ki = i;
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
  } else if (s[0] == 'r') {
    noInterrupts();
    left_ticks = 0;
    right_ticks = 0;
    interrupts();
    last_left_ticks = 0;
    last_right_ticks = 0;
    resetPidState();
    Serial.println("OK");
  } else if (s[0] == 'f') {
    // Optional fault query
    Serial.print("M1_fault=");
    Serial.print(digitalRead(M1EN) == LOW ? 1 : 0);
    Serial.print(" M2_fault=");
    Serial.println(digitalRead(M2EN) == LOW ? 1 : 0);
  } else if (s[0] == '\0') {
    Serial.println("OK");
  } else {
    Serial.println("ERR");
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (!last_char_was_newline || cmd_index > 0) {
        cmd[cmd_index] = '\0';
        processCommand(cmd);
        cmd_index = 0;
      }
      last_char_was_newline = true;
    } else if (cmd_index < CMD_BUFFER - 1) {
      last_char_was_newline = false;
      cmd[cmd_index++] = c;
    } else {
      // overflow protection: drop and reset the line
      last_char_was_newline = false;
      cmd_index = 0;
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(57600);

  // Shield control pins
  pinMode(M1INA, OUTPUT);
  pinMode(M1INB, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M1EN, INPUT);   // Matches Pololu library behavior for EN/DIAG lines

  pinMode(M2INA, OUTPUT);
  pinMode(M2INB, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M2EN, INPUT);   // Matches Pololu library behavior for EN/DIAG lines

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Left encoder uses Uno external interrupt pin D3
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);

  // Right encoder uses pin-change interrupt on D11
  setupRightEncoderPinChangeInterrupt();

  setLeftMotor(0);
  setRightMotor(0);
  last_loop = millis();
}

// ---------------- Main loop ----------------
void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - last_loop >= LOOP_INTERVAL) {
    float dt = (now - last_loop) / 1000.0f;
    last_loop = now;

    long left_now = 0;
    long right_now = 0;
    readEncoderTicksAtomic(left_now, right_now);

    long left_delta = left_now - last_left_ticks;
    long right_delta = right_now - last_right_ticks;
    last_left_ticks = left_now;
    last_right_ticks = right_now;

    float left_vel = left_delta / dt;
    float right_vel = right_delta / dt;

    // Left PID
    float left_error = left_target - left_vel;
    left_integral += left_error * dt;
    float left_derivative = (left_error - left_prev_error) / dt;
    left_pwm = (int)(Kp * left_error + Ki * left_integral + Kd * left_derivative);
    left_pwm = constrain(left_pwm, -MAX_PWM, MAX_PWM);
    left_prev_error = left_error;

    // Right PID
    float right_error = right_target - right_vel;
    right_integral += right_error * dt;
    float right_derivative = (right_error - right_prev_error) / dt;
    right_pwm = (int)(Kp * right_error + Ki * right_integral + Kd * right_derivative);
    right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);
    right_prev_error = right_error;

    // Clean stop when targets are zero
    if (left_target == 0.0f && right_target == 0.0f) {
      left_pwm = 0;
      right_pwm = 0;
    }

    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);
  }
}
