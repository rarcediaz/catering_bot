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

#define ENCODER_CPR   3136L   // Keep synced with your ROS config / real decoding mode
#define LOOP_INTERVAL 33UL    // ms, chosen to match ros2_control loop_rate ~= 30 Hz
#define COMMAND_TIMEOUT_MS 200UL
#define MAX_PWM       255
#define LEFT_MOTOR_SIGN  -1
#define RIGHT_MOTOR_SIGN -1
#define LEFT_ENCODER_SIGN  -1
#define RIGHT_ENCODER_SIGN -1
#define LEFT_BREAKAWAY_PWM   55
#define RIGHT_BREAKAWAY_PWM  55
#define LEFT_STARTUP_BOOST_PWM  80
#define RIGHT_STARTUP_BOOST_PWM 80
#define STARTUP_BOOST_MS 120UL
#define MOVING_VEL_THRESHOLD 120.0f
#define ALLOW_RUNTIME_PID_UPDATES 0
#define TARGET_SCALE  (1000.0f / (float)LOOP_INTERVAL)

// ---------------- PID constants ----------------
float Kp = 0.10f;
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
unsigned long last_command_ms = 0;
unsigned long left_startup_boost_until_ms = 0;
unsigned long right_startup_boost_until_ms = 0;

// ---------------- Encoder ISRs ----------------
void leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  int delta = (a == b) ? 1 : -1;
  left_ticks += LEFT_ENCODER_SIGN * delta;
}

void rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  int delta = (a == b) ? -1 : 1;
  right_ticks += RIGHT_ENCODER_SIGN * delta;
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

void armStartupBoost(float previous_target, float new_target, unsigned long now, unsigned long &boost_until_ms) {
  bool target_was_zero = fabs(previous_target) < 1e-3f;
  bool target_is_zero = fabs(new_target) < 1e-3f;
  bool direction_changed = !target_was_zero && !target_is_zero &&
    ((previous_target > 0.0f && new_target < 0.0f) || (previous_target < 0.0f && new_target > 0.0f));

  if (target_is_zero) {
    boost_until_ms = 0;
    return;
  }

  if (target_was_zero || direction_changed) {
    boost_until_ms = now + STARTUP_BOOST_MS;
  }
}

int applyDriveAssist(
  int pwm,
  float target,
  float measured_vel,
  unsigned long now,
  unsigned long boost_until_ms,
  int breakaway_pwm,
  int startup_boost_pwm)
{
  if (target == 0.0f || pwm == 0) {
    return pwm;
  }

  // Once the wheel is already rolling, let the PID output control it directly.
  if (fabs(measured_vel) >= MOVING_VEL_THRESHOLD) {
    return pwm;
  }

  int magnitude = abs(pwm);
  int min_pwm = (now <= boost_until_ms) ? startup_boost_pwm : breakaway_pwm;
  if (magnitude >= min_pwm) {
    return pwm;
  }

  return pwm > 0 ? min_pwm : -min_pwm;
}

int suppressReverseCorrection(int pwm, float target) {
  if (target > 0.0f && pwm < 0) {
    return 0;
  }
  if (target < 0.0f && pwm > 0) {
    return 0;
  }
  return pwm;
}

void resetPidState() {
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_prev_error = 0.0f;
  right_prev_error = 0.0f;
  left_pwm = 0;
  right_pwm = 0;
}

void stopMotion() {
  left_target = 0.0f;
  right_target = 0.0f;
  left_startup_boost_until_ms = 0;
  right_startup_boost_until_ms = 0;
  resetPidState();
  setLeftMotor(0);
  setRightMotor(0);
}

// ---------------- Motor control ----------------
void setLeftMotor(int speed) {
  speed *= LEFT_MOTOR_SIGN;
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
  speed *= RIGHT_MOTOR_SIGN;
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
      float new_left_target = l * TARGET_SCALE;
      float new_right_target = r * TARGET_SCALE;
      unsigned long now = millis();

      // Keep this protocol compatible with your RPi side.
      // diffdrive_arduino sends motor targets in encoder counts per control loop.
      // Convert that to counts/sec to compare against measured encoder velocity.
      armStartupBoost(left_target, new_left_target, now, left_startup_boost_until_ms);
      armStartupBoost(right_target, new_right_target, now, right_startup_boost_until_ms);
      left_target = new_left_target;
      right_target = new_right_target;
      last_command_ms = now;

      if (l == 0 && r == 0) {
        stopMotion();
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
#if ALLOW_RUNTIME_PID_UPDATES
      Kp = p;
      Kd = d;
      Ki = i;
#endif
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
    stopMotion();
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

  stopMotion();
  last_loop = millis();
  last_command_ms = last_loop;
  left_startup_boost_until_ms = 0;
  right_startup_boost_until_ms = 0;
}

// ---------------- Main loop ----------------
void loop() {
  handleSerial();

  unsigned long now = millis();
  if ((left_target != 0.0f || right_target != 0.0f) && (now - last_command_ms > COMMAND_TIMEOUT_MS)) {
    // Stop the robot if the host stops refreshing motor commands.
    stopMotion();
  }
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
    left_pwm = suppressReverseCorrection(left_pwm, left_target);
    left_prev_error = left_error;

    // Right PID
    float right_error = right_target - right_vel;
    right_integral += right_error * dt;
    float right_derivative = (right_error - right_prev_error) / dt;
    right_pwm = (int)(Kp * right_error + Ki * right_integral + Kd * right_derivative);
    right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);
    right_pwm = suppressReverseCorrection(right_pwm, right_target);
    right_prev_error = right_error;

    // Clean stop when targets are zero
    if (left_target == 0.0f && right_target == 0.0f) {
      left_pwm = 0;
      right_pwm = 0;
    }

    // Weighted robot + caster scrub needs extra help only while breaking static friction.
    left_pwm = applyDriveAssist(
      left_pwm, left_target, left_vel, now, left_startup_boost_until_ms,
      LEFT_BREAKAWAY_PWM, LEFT_STARTUP_BOOST_PWM);
    right_pwm = applyDriveAssist(
      right_pwm, right_target, right_vel, now, right_startup_boost_until_ms,
      RIGHT_BREAKAWAY_PWM, RIGHT_STARTUP_BOOST_PWM);

    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);
  }
}