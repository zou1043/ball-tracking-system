#include <ESP32Servo.h>
#include <ESP32Encoder.h>

// 1. Pin definitions
const int S1_PIN = 11;
const int S2_PIN = 12;

// Motor A (left wheel) - keep the working wiring
const int AIN1 = 7;
const int AIN2 = 6;
const int PWMA = 15;
const int E1A = 4;
const int E1B = 5;

// Motor B (right wheel) - keep the working wiring
const int BIN1 = 17;
const int BIN2 = 16;
const int PWMB = 18;
const int E2A = 14;
const int E2B = 13;

// 2. Encoder direction - keep the working polarity
const int ENC_A_DIR = -1;
const int ENC_B_DIR = 1;

// 3. Motion control parameters
const int PID_INTERVAL = 20;

float Kp = 0.12;
float Ki = 0.03;
float Kd = 0.00;

const int RAMP_STEP = 400;

// Startup assist
const int MIN_START_PWM_A = 60;
const int MIN_START_PWM_B = 65;
const long START_ASSIST_PPS_THRESHOLD = 20;

// 4. Data structure
struct MotorPID {
  long target_pps;
  long ramp_pps;
  long current_pps;
  long last_count;
  float integral;
  float last_error;
  int pwm_output;
};

MotorPID motorA = {0, 0, 0, 0, 0, 0, 0};
MotorPID motorB = {0, 0, 0, 0, 0, 0, 0};

unsigned long lastCmdTime = 0;
Servo s1, s2;
ESP32Encoder encA, encB;

// 5. Low-level motor control
void setMotorPWM(char motor, int pwm) {
  int p1 = (motor == 'a') ? AIN1 : BIN1;
  int p2 = (motor == 'a') ? AIN2 : BIN2;
  int pwm_pin = (motor == 'a') ? PWMA : PWMB;

  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    analogWrite(pwm_pin, pwm);
  } else if (pwm < 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    analogWrite(pwm_pin, -pwm);
  } else {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    analogWrite(pwm_pin, 0);
  }
}

int applyStartAssist(char motor, int pwm, long target_pps, long current_pps) {
  if (target_pps == 0 || pwm == 0) {
    return pwm;
  }

  if (abs(current_pps) > START_ASSIST_PPS_THRESHOLD) {
    return pwm;
  }

  int min_start_pwm = (motor == 'a') ? MIN_START_PWM_A : MIN_START_PWM_B;
  if (abs(pwm) >= min_start_pwm) {
    return pwm;
  }

  return (pwm > 0) ? min_start_pwm : -min_start_pwm;
}

// 6. PID core logic
void calculatePID() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  if (now - lastTime < PID_INTERVAL) {
    return;
  }

  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  auto applyRamp = [](MotorPID &m) {
    if (m.ramp_pps < m.target_pps) {
      m.ramp_pps = min(m.ramp_pps + RAMP_STEP, m.target_pps);
    } else if (m.ramp_pps > m.target_pps) {
      m.ramp_pps = max(m.ramp_pps - RAMP_STEP, m.target_pps);
    }
  };

  applyRamp(motorA);
  applyRamp(motorB);

  long countA = encA.getCount() * ENC_A_DIR;
  long countB = encB.getCount() * ENC_B_DIR;
  motorA.current_pps = (countA - motorA.last_count) / dt;
  motorB.current_pps = (countB - motorB.last_count) / dt;
  motorA.last_count = countA;
  motorB.last_count = countB;

  auto compute = [&](MotorPID &m) {
    float error = m.ramp_pps - m.current_pps;
    m.integral = constrain(m.integral + error * dt, -10000.0f, 10000.0f);
    float derivative = (error - m.last_error) / dt;
    m.pwm_output = (Kp * error) + (Ki * m.integral) + (Kd * derivative);
    m.last_error = error;

    if (m.target_pps == 0) {
      m.pwm_output = 0;
      m.integral = 0;
      m.last_error = 0;
      m.ramp_pps = 0;
    }
  };

  compute(motorA);
  compute(motorB);

  motorA.pwm_output = applyStartAssist('a', motorA.pwm_output, motorA.target_pps, motorA.current_pps);
  motorB.pwm_output = applyStartAssist('b', motorB.pwm_output, motorB.target_pps, motorB.current_pps);

  setMotorPWM('a', motorA.pwm_output);
  setMotorPWM('b', motorB.pwm_output);
}

void setup() {
  Serial.begin(115200);

  s1.attach(S1_PIN, 500, 2500);
  s2.attach(S2_PIN, 500, 2500);
  s1.write(100);
  s2.write(150);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  encA.attachHalfQuad(E1A, E1B);
  encB.attachHalfQuad(E2A, E2B);
  encA.clearCount();
  encB.clearCount();

  lastCmdTime = millis();
  Serial.println("System Ready. Fast response version.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    char type, id_char;
    int id_int;
    long val;

    if (sscanf(input.c_str(), "%c %c %ld", &type, &id_char, &val) == 3 && type == 'm') {
      if (id_char == 'a') {
        motorA.target_pps = val;
      } else if (id_char == 'b') {
        motorB.target_pps = val;
      }
      lastCmdTime = millis();
    } else if (sscanf(input.c_str(), "%c %d %ld", &type, &id_int, &val) == 3 && type == 's') {
      if (id_int == 1) {
        s1.write(val);
      } else if (id_int == 2) {
        s2.write(val);
      }
    }
  }

  if (millis() - lastCmdTime > 600000 && (motorA.target_pps != 0 || motorB.target_pps != 0)) {
    motorA.target_pps = 0;
    motorB.target_pps = 0;
  }

  calculatePID();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.printf(
      "TargetA:%ld RampA:%ld ActualA:%ld TargetB:%ld RampB:%ld ActualB:%ld\n",
      motorA.target_pps, motorA.ramp_pps, motorA.current_pps,
      motorB.target_pps, motorB.ramp_pps, motorB.current_pps
    );
    lastPrint = millis();
  }
}
