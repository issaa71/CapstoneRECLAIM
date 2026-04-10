// =====================================================
// TEENSY ARM CONTROLLER — RECLAIM
// -----------------------------------------------------
// Based on Shady Siam's final prototype firmware.
// Safety features: motor pin safety,
// servo pin init, detached startup with ARM/DISARM.
//
// Features
//   - 6 DOF hobby-servo arm control
//   - Serial command interface (case-insensitive)
//   - Quintic minimum-jerk interpolation at 50 Hz
//   - Named pose table stored in RAM
//   - Teach mode with NUDGE / SAVE / DELETE
//   - ARM/DISARM for safe detached startup
//   - HOLD sentinel (-999) for per-joint "keep current" in poses
//   - GRIP command for gripper-only control
//   - PICKUP command for full pick-and-sort sequence
//   - DRIVE command for differential drive motors (independent of ARM/DISARM)
//   - TICKS / RESET_TICKS for encoder feedback
//   - ESTOP for emergency stop (motors + arm)
//   - Motor safety timeout: auto-stop if no DRIVE in 500ms
//
// Serial commands
//   HELP
//   PRINT       -> print joint angles + µs
//   READUS      -> print raw µs values (for ROS2 bridge)
//   LIST
//   STOP
//   ARM         -> attach all servos at startup pose
//   DISARM      -> detach all servos (go limp)
//
//   SET j1 j2 j3 j4 j5 j6 [T ms]
//     -> joint-space degrees, use -999 for HOLD
//
//   SETUS us1 us2 us3 us4 us5 us6 [T ms]
//     -> microseconds directly, use -1 for HOLD
//     -> used by ROS2 teensy_bridge for MoveIt2 control
//
//   GOTO j1 j2 j3 j4 j5 j6
//     -> immediate write, no trajectory
//
//   POSE name
//   POSE name T ms
//
//   SAVE name
//   DELETE name
//
//   NUDGE joint_index delta_deg
//     -> example: NUDGE 1 2.0
//
//   GRIP angle [T ms]
//     -> move only gripper (J6) to angle
//
//   RUNBIN n
//     -> home -> binN_pre -> binN_drop -> binN_pre -> home
//
//   PICKUP n angle [T ms]
//     -> full pick-and-sort cycle to bin n with grip angle
//
// Notes
//   - Joint space is in degrees, J1..J6
//   - SAVE is RAM-only; saved poses are lost after reset/reflash
//   - Servos start DISARMED. Send ARM after positioning arm by hand.
//   - Poses with HOLD (-999) on a joint preserve that joint's current angle
// =====================================================

#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <string.h>

// ---------------- USER CONFIG ----------------
static const int N = 6;

// J1..J6 on Teensy pins 10..15
static const int SERVO_PINS[N] = {
  10, 11, 12, 13, 14, 15  // Updated March 18 (J6 confirmed pin 15)
};

// Wheel motor driver pins — must be driven LOW on boot
// to prevent wheels from spinning when arm firmware is loaded.
//   Left motor:  MD10C       — PWM=2, DIR=3
//   Right motor: BTS7960     — RPWM=22, LPWM=23
// BTS7960 uses two PWM pins (RPWM for forward, LPWM for reverse)
// instead of MD10C's single PWM + DIR scheme.
// Pin 20 does NOT support PWM on Teensy 4.1, so LPWM moved to pin 23.
// BTS7960 R_EN and L_EN must be tied to 3.3V (always enabled).
static const int MOTOR_PINS[] = {2, 3, 22, 23};  // All motor control pins
static const int NUM_MOTOR_PINS = 4;

// Right motor driver type flag
static const bool RIGHT_IS_BTS7960 = true;  // false = MD10C, true = BTS7960

// --- Encoder pins ---
static const int L_ENC_A = 6;
static const int L_ENC_B = 7;
static const int R_ENC_A = 19;
static const int R_ENC_B = 18;

// --- Encoder state ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- Encoder ISRs (from dual_motor_test.cpp, verified working) ---
void leftISR_A() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B))
    leftTicks++;
  else
    leftTicks--;
}

void leftISR_B() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B))
    leftTicks--;
  else
    leftTicks++;
}

void rightISR_A() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B))
    rightTicks--;
  else
    rightTicks++;
}

void rightISR_B() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B))
    rightTicks++;
  else
    rightTicks--;
}

// --- Motor drive helpers ---
static long readLeftTicks() {
  noInterrupts();
  long t = leftTicks;
  interrupts();
  return t;
}

static long readRightTicks() {
  noInterrupts();
  long t = rightTicks;
  interrupts();
  return t;
}

static void resetTicks() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

// Drive both motors. PWM is signed: -255..255. Negative = reverse.
// Direction polarity from dual_motor_test.cpp:
//   Left (MD10C):   DIR LOW  = forward, DIR HIGH = reverse
//   Right (BTS7960): RPWM = forward, LPWM = reverse
//   Right (MD10C):   DIR HIGH = forward, DIR LOW  = reverse
static void driveMotors(int leftPWM, int rightPWM) {
  // Left motor — MD10C (PWM=2, DIR=3)
  if (leftPWM >= 0) {
    digitalWrite(3, LOW);
    analogWrite(2, min(leftPWM, 255));
  } else {
    digitalWrite(3, HIGH);
    analogWrite(2, min(-leftPWM, 255));
  }

  // Right motor — pin 22 and pin 23 (BTS7960) or pin 22 and pin 20 (MD10C)
  if (RIGHT_IS_BTS7960) {
    // BTS7960: pin22=RPWM, pin23=LPWM
    if (rightPWM >= 0) {
      analogWrite(22, min(rightPWM, 255)); // RPWM forward
      analogWrite(23, 0);                  // LPWM off
    } else {
      analogWrite(22, 0);                   // RPWM off
      analogWrite(23, min(-rightPWM, 255)); // LPWM reverse
    }
  } else {
    // MD10C: PWM=22, DIR=20
    if (rightPWM >= 0) {
      digitalWrite(20, HIGH);
      analogWrite(22, min(rightPWM, 255));
    } else {
      digitalWrite(20, LOW);
      analogWrite(22, min(-rightPWM, 255));
    }
  }
}

// Motor safety timeout: stop if no DRIVE/SETVEL command in 500ms
static uint32_t lastDriveMs = 0;
static bool driveActive = false;

// ═══════════════════════════════════════════════════════════════
// PI VELOCITY CONTROLLER — per-wheel closed-loop speed control
// ═══════════════════════════════════════════════════════════════
//
// Plant model (from step response):  G(s) = K / (τs + 1)
// Controller:  u = Kf·v_target + Kp·error + Ki·∫error·dt
//
// REPLACE these constants with values from step_response.py:
//   K  = DC gain (m/s per unit PWM)
//   τ  = time constant (seconds)
//   Then compute: Kf = 1/K, and Kp/Ki from pole placement.

// --- Plant parameters (from step response test 2026-03-21, calibrated wheel_radius=0.048m) ---
// Left motor:  G(s) = 0.001073 / (0.0503s + 1)
static const float PLANT_K_L   = 0.001170f;  // DC gain: (m/s) per PWM unit (scaled from 0.001073 × 0.05232/0.048)
static const float PLANT_TAU_L = 0.0503f;    // time constant (seconds)
// Right motor: G(s) = 0.000956 / (0.0463s + 1)
static const float PLANT_K_R   = 0.001042f;  // (scaled from 0.000956 × 0.05232/0.048)
static const float PLANT_TAU_R = 0.0463f;

// --- Controller design parameters ---
static const float PI_ZETA = 0.7f;           // damping ratio (slightly underdamped)
static const float PI_WN_MULT = 1.5f;        // ωn = PI_WN_MULT / τ (reduced — increase if too sluggish)

// --- Computed PI gains (left wheel) ---
// ωn = 6/τ, Kp = (2ζωnτ - 1) / K, Ki = ωn²τ / K, Kf = 1/K
static float pi_Kp_L = (2.0f * PI_ZETA * (PI_WN_MULT / PLANT_TAU_L) * PLANT_TAU_L - 1.0f) / PLANT_K_L;
static float pi_Ki_L = ((PI_WN_MULT / PLANT_TAU_L) * (PI_WN_MULT / PLANT_TAU_L) * PLANT_TAU_L) / PLANT_K_L;
static float pi_Kf_L = 1.0f / PLANT_K_L;

// --- Computed PI gains (right wheel) ---
static float pi_Kp_R = (2.0f * PI_ZETA * (PI_WN_MULT / PLANT_TAU_R) * PLANT_TAU_R - 1.0f) / PLANT_K_R;
static float pi_Ki_R = ((PI_WN_MULT / PLANT_TAU_R) * (PI_WN_MULT / PLANT_TAU_R) * PLANT_TAU_R) / PLANT_K_R;
static float pi_Kf_R = 1.0f / PLANT_K_R;

// --- PI state ---
static float pi_integral_L = 0.0f;
static float pi_integral_R = 0.0f;
static float pi_filtered_vel_L = 0.0f;    // low-pass filtered velocity
static float pi_filtered_vel_R = 0.0f;
static float pi_target_vel_L = 0.0f;      // commanded target velocity (m/s)
static float pi_target_vel_R = 0.0f;
static float pi_ramped_vel_L = 0.0f;      // ramped target (actual setpoint for PI)
static float pi_ramped_vel_R = 0.0f;
static bool  pi_active = false;            // true when SETVEL is being used
static long  pi_prev_ticks_L = 0;
static long  pi_prev_ticks_R = 0;
static uint32_t pi_prev_time_us = 0;       // microsecond timer for precise dt

// --- PI tuning constants ---
static const float PI_DT_MS = 10.0f;       // control loop period (ms) = 100Hz
static const float PI_FILTER_ALPHA = 0.3f;  // velocity low-pass filter (0-1, lower=smoother)
static const float PI_MAX_VEL = 0.3f;       // max allowed target velocity (m/s)
static const float PI_RAMP_RATE = 0.5f;     // max acceleration (m/s²) — 0→0.1 in 200ms
static const int   PI_STATIC_FRICTION_PWM = 15;  // minimum PWM to overcome motor stiction

// --- Wheel geometry (must match teensy_bridge.yaml) ---
static const float WHEEL_RADIUS = 0.05232f;          // meters (calibrated: 0.048 × 54.5/50.0, iteration 5)
static const float TICKS_PER_REV = 7560.0f;
static const float METERS_PER_TICK = 2.0f * 3.14159265f * WHEEL_RADIUS / TICKS_PER_REV;

static void piControllerUpdate() {
  // Read current ticks (atomic)
  long ticks_L = readLeftTicks();
  long ticks_R = readRightTicks();

  // Compute dt from micros() for precision
  uint32_t now_us = micros();
  float dt = (float)(now_us - pi_prev_time_us) / 1000000.0f;
  pi_prev_time_us = now_us;

  // Guard against bad dt (first call, or overflow)
  if (dt <= 0.0f || dt > 0.1f) {
    pi_prev_ticks_L = ticks_L;
    pi_prev_ticks_R = ticks_R;
    return;
  }

  // --- Ramp rate limiter (smooth acceleration/deceleration) ---
  float max_delta = PI_RAMP_RATE * dt;  // max velocity change per tick
  if (pi_ramped_vel_L < pi_target_vel_L)
    pi_ramped_vel_L = min(pi_ramped_vel_L + max_delta, pi_target_vel_L);
  else if (pi_ramped_vel_L > pi_target_vel_L)
    pi_ramped_vel_L = max(pi_ramped_vel_L - max_delta, pi_target_vel_L);

  if (pi_ramped_vel_R < pi_target_vel_R)
    pi_ramped_vel_R = min(pi_ramped_vel_R + max_delta, pi_target_vel_R);
  else if (pi_ramped_vel_R > pi_target_vel_R)
    pi_ramped_vel_R = max(pi_ramped_vel_R - max_delta, pi_target_vel_R);

  // --- Compute measured velocity (m/s) ---
  // Encoders count negative for forward motion, so negate to get positive = forward
  float raw_vel_L = -(float)(ticks_L - pi_prev_ticks_L) * METERS_PER_TICK / dt;
  float raw_vel_R = -(float)(ticks_R - pi_prev_ticks_R) * METERS_PER_TICK / dt;
  pi_prev_ticks_L = ticks_L;
  pi_prev_ticks_R = ticks_R;

  // --- Low-pass filter (exponential moving average) ---
  pi_filtered_vel_L = PI_FILTER_ALPHA * raw_vel_L + (1.0f - PI_FILTER_ALPHA) * pi_filtered_vel_L;
  pi_filtered_vel_R = PI_FILTER_ALPHA * raw_vel_R + (1.0f - PI_FILTER_ALPHA) * pi_filtered_vel_R;

  // --- PI controller for LEFT wheel ---
  float error_L = pi_ramped_vel_L - pi_filtered_vel_L;
  float output_L = pi_Kf_L * pi_ramped_vel_L + pi_Kp_L * error_L + pi_Ki_L * pi_integral_L;

  // Anti-windup: only accumulate integral if output is not saturated
  if (output_L > -255.0f && output_L < 255.0f) {
    pi_integral_L += error_L * dt;
  }

  // Clamp output + static friction compensation (deadband kick)
  int pwm_L = (int)constrain(output_L, -255.0f, 255.0f);
  if (pwm_L > 0 && pwm_L < PI_STATIC_FRICTION_PWM) pwm_L = PI_STATIC_FRICTION_PWM;
  if (pwm_L < 0 && pwm_L > -PI_STATIC_FRICTION_PWM) pwm_L = -PI_STATIC_FRICTION_PWM;
  if (abs(pi_ramped_vel_L) < 0.001f) pwm_L = 0;  // don't kick when target is zero

  // --- PI controller for RIGHT wheel ---
  float error_R = pi_ramped_vel_R - pi_filtered_vel_R;
  float output_R = pi_Kf_R * pi_ramped_vel_R + pi_Kp_R * error_R + pi_Ki_R * pi_integral_R;

  // Anti-windup
  if (output_R > -255.0f && output_R < 255.0f) {
    pi_integral_R += error_R * dt;
  }

  // Clamp output + static friction compensation
  int pwm_R = (int)constrain(output_R, -255.0f, 255.0f);
  if (pwm_R > 0 && pwm_R < PI_STATIC_FRICTION_PWM) pwm_R = PI_STATIC_FRICTION_PWM;
  if (pwm_R < 0 && pwm_R > -PI_STATIC_FRICTION_PWM) pwm_R = -PI_STATIC_FRICTION_PWM;
  if (abs(pi_ramped_vel_R) < 0.001f) pwm_R = 0;  // don't kick when target is zero

  // --- Drive motors ---
  driveMotors(pwm_L, pwm_R);

  // Keep safety timeout alive while PI is running
  lastDriveMs = millis();
  driveActive = true;
}

static void piReset() {
  pi_integral_L = 0.0f;
  pi_integral_R = 0.0f;
  pi_filtered_vel_L = 0.0f;
  pi_filtered_vel_R = 0.0f;
  pi_ramped_vel_L = 0.0f;
  pi_ramped_vel_R = 0.0f;
  pi_target_vel_L = 0.0f;
  pi_target_vel_R = 0.0f;
  pi_prev_ticks_L = readLeftTicks();
  pi_prev_ticks_R = readRightTicks();
  pi_prev_time_us = micros();
  pi_active = false;
}

// Calibrated safe pulse limits for each servo (updated March 18)
// J1 Base:         400-2600 (DS3218, full electronic range)
// J2 Shoulder:     700-2200 (DS3218, calibrated via servo_test Mar 10)
// J3 Elbow:        400-2600 (DS3218, full electronic range)
// J4 Wrist Pitch:  500-2500 (DS3218 270°, physical limits 1400-2400µs)
// J5 Wrist Rotate: 400-2600 (MG996R, physical limits 675-2600µs)
// J6 Gripper:      1400-1755 (DS3218 + LewanSoul claw, 1400=open 1755=closed)
static int SERVO_MIN_US[N] = {
  400, 700, 400, 500, 400, 1400
};

static int SERVO_MAX_US[N] = {
  2600, 2200, 2600, 2500, 2600, 1755
};

// Safe joint limits in JOINT SPACE degrees (updated March 18)
//                  J1    J2    J3     J4      J5    J6
// J4: -13.5° = 1400µs, 121.5° = 2400µs (recalibrated March 18)
// J5: -135° to 135° (DS3218 270° replacement, recalibrated March 21)
// J6: 0° = 1400µs (open), 44° = 1755µs (closed) (new LewanSoul claw)
static float JOINT_MIN_DEG[N] = {
  -148, -135, -148, -13.5, -135,  0
};

static float JOINT_MAX_DEG[N] = {
   148,  135,  110, 121.5,  135, 44
};

// Servo angle corresponding to JOINT SPACE zero
// Fill in after defining joint zero for each joint
static float ZERO_SERVO_DEG[N] = {
  135, 135, 135, 135, 135, 0
};

// true means invert direction
static bool INVERT[N] = {
  false, false, false, false, false, false
};

// Servo travel per joint
// J1-J4 = DS3218 270°, J5 = DS3218 270° (replaced MG996R March 21), J6 = DS3218 + LewanSoul claw ~44°
static float SERVO_RANGE_DEG[N] = {
  270, 270, 270, 270, 270, 44
};

// 50 Hz update
static const uint32_t DT_MS = 20;

// Default move time
static const uint32_t DEFAULT_T_MS = 1000;

// Sentinel value: "hold current position" for any joint in a pose
static const float HOLD = -999.0f;

// Safe startup pose in JOINT SPACE (home position) — recalibrated March 21
// Verified µs: J1=1229, J2=2153, J3=2030, J4=1400, J5=701, J6=1500
static float STARTUP_Q[N] = {
  -33, 127, 65, -13.5, -98.0, 12.5
};

// Max runtime poses in RAM
static const int MAX_POSES = 20;
static const int MAX_NAME_LEN = 20;

// ---------------- INTERNAL STATE ----------------
Servo servos[N];
bool armed = false;

float q_cur[N] = {0, 0, 0, 0, 0, 0};
float q0[N], qf[N];

uint32_t t_start_ms = 0;
uint32_t T_ms = 0;
bool traj_active = false;
bool abort_all = false;

String rxLine;
uint32_t last_update_ms = 0;

// ---------------- POSE TABLE ----------------
struct PoseEntry {
  bool used;
  char name[MAX_NAME_LEN];
  float q[N];
};

PoseEntry poseTable[MAX_POSES];

// ---------------- FORWARD DECLARATIONS ----------------
static void handleLine(String s);
static void serviceSerial();

// ---------------- HELPERS ----------------
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void copyQ(float dst[N], const float src[N]) {
  for (int i = 0; i < N; i++) dst[i] = src[i];
}

static bool streq_nocase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a;
    char cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = ca - 'A' + 'a';
    if (cb >= 'A' && cb <= 'Z') cb = cb - 'A' + 'a';
    if (ca != cb) return false;
    ++a;
    ++b;
  }
  return (*a == '\0' && *b == '\0');
}

static bool startsWithIgnoreCase(const String& s, const char* prefix) {
  int n = strlen(prefix);
  if ((int)s.length() < n) return false;
  for (int i = 0; i < n; i++) {
    char a = s[i];
    char b = prefix[i];
    if (a >= 'A' && a <= 'Z') a = a - 'A' + 'a';
    if (b >= 'A' && b <= 'Z') b = b - 'A' + 'a';
    if (a != b) return false;
  }
  return true;
}

// Quintic scaling function
// s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
static float s_quintic(float tau) {
  float t2 = tau * tau;
  float t3 = t2 * tau;
  float t4 = t3 * tau;
  float t5 = t4 * tau;
  return 10.0f * t3 - 15.0f * t4 + 6.0f * t5;
}

// Joint-space degrees -> servo pulse width
static int jointDegToUs(int i, float jointDeg) {
  // Clamp joint-space
  jointDeg = clampf(jointDeg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);

  // Map joint-space to servo-angle
  float sign = INVERT[i] ? -1.0f : 1.0f;
  float servoDeg = ZERO_SERVO_DEG[i] + sign * jointDeg;

  // Clamp servo angle to servo travel
  servoDeg = clampf(servoDeg, 0.0f, SERVO_RANGE_DEG[i]);

  // Linear map servo angle to pulse width
  float us = SERVO_MIN_US[i]
           + (servoDeg / SERVO_RANGE_DEG[i]) * (SERVO_MAX_US[i] - SERVO_MIN_US[i]);

  int us_i = (int)lroundf(us);

  if (us_i < SERVO_MIN_US[i]) us_i = SERVO_MIN_US[i];
  if (us_i > SERVO_MAX_US[i]) us_i = SERVO_MAX_US[i];

  return us_i;
}

static void writeAll() {
  if (!armed) return;
  for (int i = 0; i < N; i++) {
    servos[i].writeMicroseconds(jointDegToUs(i, q_cur[i]));
  }
}

static void printCompactQ() {
  Serial.print("Q: ");
  for (int i = 0; i < N; i++) {
    Serial.print(q_cur[i], 2);
    if (i < N - 1) Serial.print(" ");
  }
  Serial.println();
}

static void printQ() {
  Serial.println("STATE:");
  for (int i = 0; i < N; i++) {
    Serial.print("  J");
    Serial.print(i + 1);
    Serial.print(": q=");
    Serial.print(q_cur[i], 2);
    Serial.print(" deg, us=");
    Serial.println(jointDegToUs(i, q_cur[i]));
  }
  Serial.print("  Armed: ");
  Serial.println(armed ? "YES" : "NO");
}

// ---------------- POSE TABLE FUNCTIONS ----------------
static int findPoseIndex(const char* name) {
  for (int i = 0; i < MAX_POSES; i++) {
    if (poseTable[i].used && streq_nocase(poseTable[i].name, name)) {
      return i;
    }
  }
  return -1;
}

static int findFreePoseSlot() {
  for (int i = 0; i < MAX_POSES; i++) {
    if (!poseTable[i].used) return i;
  }
  return -1;
}

static bool getPoseByName(const char* name, float out[N]) {
  int idx = findPoseIndex(name);
  if (idx < 0) return false;
  copyQ(out, poseTable[idx].q);
  return true;
}

static bool savePoseByName(const char* name, const float q[N]) {
  if (strlen(name) == 0 || strlen(name) >= MAX_NAME_LEN) return false;

  int idx = findPoseIndex(name);
  if (idx < 0) idx = findFreePoseSlot();
  if (idx < 0) return false;

  poseTable[idx].used = true;
  strncpy(poseTable[idx].name, name, MAX_NAME_LEN - 1);
  poseTable[idx].name[MAX_NAME_LEN - 1] = '\0';
  copyQ(poseTable[idx].q, q);

  return true;
}

static bool deletePoseByName(const char* name) {
  int idx = findPoseIndex(name);
  if (idx < 0) return false;

  poseTable[idx].used = false;
  poseTable[idx].name[0] = '\0';
  for (int j = 0; j < N; j++) poseTable[idx].q[j] = 0.0f;

  return true;
}

static void listPoses() {
  Serial.println("POSES:");
  bool any = false;
  for (int i = 0; i < MAX_POSES; i++) {
    if (poseTable[i].used) {
      any = true;
      Serial.print("  ");
      Serial.print(poseTable[i].name);
      Serial.print(" = ");
      for (int j = 0; j < N; j++) {
        if (poseTable[i].q[j] <= HOLD + 0.5f) {
          Serial.print("HOLD");
        } else {
          Serial.print(poseTable[i].q[j], 1);
        }
        if (j < N - 1) Serial.print(" ");
      }
      Serial.println();
    }
  }
  if (!any) Serial.println("  (none)");
}

static void initPoseTable() {
  for (int i = 0; i < MAX_POSES; i++) {
    poseTable[i].used = false;
    poseTable[i].name[0] = '\0';
    for (int j = 0; j < N; j++) poseTable[i].q[j] = 0.0f;
  }

  // Hardcoded taught poses — recalibrated March 18
  // HOLD = -999 means "keep current joint position"
  // NOTE: Only HOME is verified. Other poses need re-teaching after J4/J5/J6 hardware changes.
  static const float HOME[N]           = { -33, 127,  65, -13.5, -98.0,  12.5};  // verified March 21 (µs: 1229,2153,2030,1400,701,1500)
  static const float HOME_CARRY[N]     = { -33, 127,  65, -13.5, -98.0, HOLD};   // transport (keeps grip)
  static const float PREPICK[N]        = { -33,  75,  65,  20,   21,    0};    // NEEDS RE-TEACHING
  static const float PREPICK_CARRY[N]  = { -33,  75,  65,  20,   21, HOLD};   // NEEDS RE-TEACHING
  static const float PICKUP_POSE[N]    = { -33,  65,  65,  20,   21,   35};   // NEEDS RE-TEACHING

  savePoseByName("home",           HOME);
  savePoseByName("home_carry",     HOME_CARRY);
  savePoseByName("prepick",        PREPICK);
  savePoseByName("prepick_carry",  PREPICK_CARRY);
  savePoseByName("pickup",         PICKUP_POSE);
}

// ---------------- TRAJECTORY ----------------
static void startTraj(const float target[N], uint32_t duration_ms) {
  abort_all = false;

  for (int i = 0; i < N; i++) {
    q0[i] = q_cur[i];
    if (target[i] <= HOLD + 0.5f) {
      qf[i] = q_cur[i];  // HOLD: keep current position
    } else {
      qf[i] = clampf(target[i], JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
    }
  }

  T_ms = (duration_ms < 100) ? 100 : duration_ms;
  t_start_ms = millis();
  traj_active = true;

  Serial.print("OK T=");
  Serial.println(T_ms);
}

static void updateTraj() {
  if (!traj_active) return;

  uint32_t now = millis();
  uint32_t elapsed = now - t_start_ms;

  float tau = (float)elapsed / (float)T_ms;
  if (tau > 1.0f) tau = 1.0f;

  float s = s_quintic(tau);

  for (int i = 0; i < N; i++) {
    q_cur[i] = q0[i] + (qf[i] - q0[i]) * s;
  }

  writeAll();

  if (elapsed >= T_ms) {
    traj_active = false;
    copyQ(q_cur, qf);
    writeAll();
    Serial.println("DONE");
  }
}

static bool runPoseBlocking(const char* poseName, uint32_t ms) {
  if (abort_all) return false;

  float tgt[N];
  if (!getPoseByName(poseName, tgt)) {
    Serial.print("ERR unknown pose ");
    Serial.println(poseName);
    return false;
  }

  startTraj(tgt, ms);

  while (traj_active) {
    serviceSerial();  // lets STOP work during sequence motion
    if (abort_all) {
      traj_active = false;
      writeAll();
      return false;
    }
    updateTraj();
    delay(DT_MS);
  }

  return !abort_all;
}

// ---------------- COMMAND PARSING HELPERS ----------------
static int findTimeSuffixPos(const String& s) {
  String up = s;
  up.toUpperCase();
  return up.indexOf("T ");
}

static uint32_t parseTimeSuffixMs(const String& s, uint32_t fallback) {
  int tpos = findTimeSuffixPos(s);
  if (tpos < 0) return fallback;

  String msStr = s.substring(tpos + 2);
  msStr.trim();

  long v = msStr.toInt();
  if (v <= 0) return fallback;
  return (uint32_t)v;
}

static String stripTimeSuffix(const String& s) {
  int tpos = findTimeSuffixPos(s);
  if (tpos < 0) {
    String out = s;
    out.trim();
    return out;
  }

  String out = s.substring(0, tpos);
  out.trim();
  return out;
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  HELP");
  Serial.println("  PRINT       Show joint angles + µs");
  Serial.println("  READUS      Print raw µs values");
  Serial.println("  LIST");
  Serial.println("  STOP");
  Serial.println("  ARM         Attach servos at startup pose");
  Serial.println("  DISARM      Detach all servos (go limp)");
  Serial.println();
  Serial.println("  SET j1 j2 j3 j4 j5 j6 [T ms]");
  Serial.println("    Joint-space degrees, -999=hold");
  Serial.println("  SETUS us1 us2 us3 us4 us5 us6 [T ms]");
  Serial.println("    Microseconds directly, -1=hold");
  Serial.println("  GOTO j1 j2 j3 j4 j5 j6");
  Serial.println("    Immediate write (no trajectory)");
  Serial.println();
  Serial.println("  POSE name [T ms]");
  Serial.println("  SAVE name");
  Serial.println("  DELETE name");
  Serial.println("  NUDGE joint_index delta_deg");
  Serial.println();
  Serial.println("  GRIP angle [T ms]");
  Serial.println("  RUNBIN n   (n=1..3)");
  Serial.println("  PICKUP n angle [T ms]");
  Serial.println();
  Serial.println("  --- Drive (no ARM needed) ---");
  Serial.println("  SETVEL left right  Closed-loop velocity (m/s)");
  Serial.println("  DRIVE left right   Open-loop PWM -255..255");
  Serial.println("  TICKS              Read encoder counts");
  Serial.println("  RESET_TICKS        Zero encoders");
  Serial.println("  PI_STATUS          Print PI controller state");
  Serial.println("  ESTOP              Stop motors + arm");
  Serial.println();
  Serial.println("  --- System ID ---");
  Serial.println("  STEP_TEST pwm duration_ms side");
  Serial.println("    pwm: 0..255, duration: ms, side: L or R");
  Serial.println("    Logs ticks at 100Hz. Output: CSV lines.");
}

// ---------------- SERIAL INPUT ----------------
static void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // Filter escape sequences (arrow keys etc)
    if (c == 0x1B) {  // ESC
      // consume the rest of the escape sequence
      delay(2);
      while (Serial.available()) {
        char esc = Serial.read();
        if ((esc >= 'A' && esc <= 'Z') || (esc >= 'a' && esc <= 'z') || esc == '~') break;
      }
      continue;
    }

    if (c == '\n' || c == '\r') {
      Serial.println();  // echo newline
      handleLine(rxLine);
      rxLine = "";
      Serial.print("> ");  // prompt
    } else if (c == 0x7F || c == 0x08) {  // backspace / delete
      if (rxLine.length() > 0) {
        rxLine.remove(rxLine.length() - 1);
        Serial.print("\b \b");  // erase character on screen
      }
    } else if (c >= 0x20) {  // printable characters only
      rxLine += c;
      Serial.print(c);  // echo
      if (rxLine.length() > 200) rxLine = "";
    }
  }
}

// ---------------- SERIAL COMMANDS ----------------
static void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  // Uppercase copy for command recognition
  String up = s;
  up.toUpperCase();

  if (up == "HELP") {
    printHelp();
    return;
  }

  if (up == "PRINT") {
    printQ();
    return;
  }

  if (up == "LIST") {
    listPoses();
    return;
  }

  if (up == "STOP") {
    abort_all = true;
    traj_active = false;
    writeAll();
    Serial.println("OK STOP");
    return;
  }

  // ARM — attach all servos at startup pose
  if (up == "ARM") {
    if (armed) {
      Serial.println("OK already armed");
      return;
    }

    for (int i = 0; i < N; i++) {
      q_cur[i] = clampf(STARTUP_Q[i], JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
    }

    for (int i = 0; i < N; i++) {
      servos[i].attach(SERVO_PINS[i], SERVO_MIN_US[i], SERVO_MAX_US[i]);
    }

    armed = true;
    writeAll();

    Serial.println("OK ARMED");
    printCompactQ();
    return;
  }

  // DISARM — detach all servos
  if (up == "DISARM") {
    traj_active = false;
    abort_all = true;
    armed = false;

    for (int i = 0; i < N; i++) {
      servos[i].detach();
      pinMode(SERVO_PINS[i], OUTPUT);
      digitalWrite(SERVO_PINS[i], LOW);
    }

    Serial.println("OK DISARMED");
    return;
  }

  // READUS — print raw microsecond values for all joints
  if (up == "READUS") {
    Serial.print("US:");
    for (int i = 0; i < N; i++) {
      Serial.print(" ");
      Serial.print(jointDegToUs(i, q_cur[i]));
    }
    Serial.println();
    return;
  }

  // --- DRIVE / TICKS / RESET_TICKS / ESTOP ---
  // These do NOT require armed state (motors are independent of servos)

  // SETVEL left_mps right_mps — closed-loop velocity control (m/s)
  // Uses PI controller with feedforward. Replaces DRIVE for smooth control.
  // Example: SETVEL 0.10 0.12
  if (startsWithIgnoreCase(s, "SETVEL")) {
    String rest = s.substring(6);
    rest.trim();
    float lv = 0.0f, rv = 0.0f;
    if (sscanf(rest.c_str(), "%f %f", &lv, &rv) == 2) {
      // Clamp to max velocity
      lv = constrain(lv, -PI_MAX_VEL, PI_MAX_VEL);
      rv = constrain(rv, -PI_MAX_VEL, PI_MAX_VEL);
      pi_target_vel_L = lv;
      pi_target_vel_R = rv;

      // Initialize PI on first call
      if (!pi_active) {
        pi_prev_ticks_L = readLeftTicks();
        pi_prev_ticks_R = readRightTicks();
        pi_prev_time_us = micros();
        pi_integral_L = 0.0f;
        pi_integral_R = 0.0f;
        pi_filtered_vel_L = 0.0f;
        pi_filtered_vel_R = 0.0f;
        pi_active = true;
      }

      lastDriveMs = millis();
      driveActive = true;
      Serial.println("OK SETVEL");
    } else {
      Serial.println("ERR usage: SETVEL left_mps right_mps");
    }
    return;
  }

  // PI_STATUS — print current PI controller state (for debugging/tuning)
  if (up == "PI_STATUS") {
    Serial.println("PI Controller Status:");
    Serial.print("  Active: "); Serial.println(pi_active ? "YES" : "NO");
    Serial.print("  Target L: "); Serial.print(pi_target_vel_L, 4); Serial.println(" m/s");
    Serial.print("  Target R: "); Serial.print(pi_target_vel_R, 4); Serial.println(" m/s");
    Serial.print("  Filtered L: "); Serial.print(pi_filtered_vel_L, 4); Serial.println(" m/s");
    Serial.print("  Filtered R: "); Serial.print(pi_filtered_vel_R, 4); Serial.println(" m/s");
    Serial.print("  Error L: "); Serial.print(pi_target_vel_L - pi_filtered_vel_L, 4); Serial.println(" m/s");
    Serial.print("  Error R: "); Serial.print(pi_target_vel_R - pi_filtered_vel_R, 4); Serial.println(" m/s");
    Serial.print("  Integral L: "); Serial.println(pi_integral_L, 4);
    Serial.print("  Integral R: "); Serial.println(pi_integral_R, 4);
    Serial.print("  Gains L: Kp="); Serial.print(pi_Kp_L, 2);
    Serial.print(" Ki="); Serial.print(pi_Ki_L, 2);
    Serial.print(" Kf="); Serial.println(pi_Kf_L, 2);
    Serial.print("  Gains R: Kp="); Serial.print(pi_Kp_R, 2);
    Serial.print(" Ki="); Serial.print(pi_Ki_R, 2);
    Serial.print(" Kf="); Serial.println(pi_Kf_R, 2);
    return;
  }

  // DRIVE left right — open-loop PWM control (bypasses PI controller)
  // Use SETVEL for closed-loop control. DRIVE is for testing/manual override.
  if (startsWithIgnoreCase(s, "DRIVE")) {
    String rest = s.substring(5);
    rest.trim();
    int lp = 0, rp = 0;
    if (sscanf(rest.c_str(), "%d %d", &lp, &rp) == 2) {
      lp = max(-255, min(255, lp));
      rp = max(-255, min(255, rp));
      // Disable PI if switching to raw PWM
      if (pi_active) piReset();
      driveMotors(lp, rp);
      lastDriveMs = millis();
      driveActive = true;
      Serial.println("OK DRIVE");
    } else {
      Serial.println("ERR usage: DRIVE left right (-255..255)");
    }
    return;
  }

  // TICKS — return encoder counts
  if (up == "TICKS") {
    Serial.print("TICKS ");
    Serial.print(readLeftTicks());
    Serial.print(" ");
    Serial.println(readRightTicks());
    return;
  }

  // RESET_TICKS — zero both encoder counters
  if (up == "RESET_TICKS") {
    resetTicks();
    Serial.println("OK RESET_TICKS");
    return;
  }

  // STEP_TEST pwm duration_ms side — open-loop step response for system ID
  // Applies a fixed PWM to one motor and logs encoder ticks at 100Hz.
  // Output format: CSV lines "time_ms,ticks" for curve fitting.
  // Example: STEP_TEST 150 2000 L  (PWM=150, 2 seconds, left motor)
  if (startsWithIgnoreCase(s, "STEP_TEST")) {
    String rest = s.substring(9);
    rest.trim();
    int pwm = 0;
    int dur_ms = 2000;
    char side = 'L';
    if (sscanf(rest.c_str(), "%d %d %c", &pwm, &dur_ms, &side) >= 1) {
      pwm = max(0, min(255, pwm));
      dur_ms = max(100, min(10000, dur_ms));
      side = toupper(side);
      bool useLeft = (side != 'R');

      // Reset encoder for the tested motor
      noInterrupts();
      if (useLeft) leftTicks = 0; else rightTicks = 0;
      interrupts();

      Serial.print("STEP_TEST: PWM=");
      Serial.print(pwm);
      Serial.print(" dur=");
      Serial.print(dur_ms);
      Serial.print("ms side=");
      Serial.println(useLeft ? "LEFT" : "RIGHT");
      Serial.println("time_ms,ticks");

      // Small delay before applying step (baseline reading)
      delay(50);

      // Apply step
      uint32_t startMs = millis();
      if (useLeft) {
        driveMotors(pwm, 0);
      } else {
        driveMotors(0, pwm);
      }

      // Log at 100Hz (every 10ms)
      uint32_t nextSample = startMs;
      while ((millis() - startMs) < (uint32_t)dur_ms) {
        if (millis() >= nextSample) {
          long ticks;
          if (useLeft) ticks = readLeftTicks(); else ticks = readRightTicks();
          Serial.print(millis() - startMs);
          Serial.print(",");
          Serial.println(ticks);
          nextSample += 10;
        }
      }

      // Stop motor
      driveMotors(0, 0);
      driveActive = false;

      // Final reading
      long finalTicks;
      if (useLeft) finalTicks = readLeftTicks(); else finalTicks = readRightTicks();
      Serial.print(millis() - startMs);
      Serial.print(",");
      Serial.println(finalTicks);

      Serial.println("STEP_TEST_DONE");
    } else {
      Serial.println("ERR usage: STEP_TEST pwm duration_ms side(L/R)");
    }
    return;
  }

  // ESTOP — emergency stop: motors AND arm
  if (up == "ESTOP") {
    driveMotors(0, 0);
    driveActive = false;
    if (pi_active) piReset();
    abort_all = true;
    traj_active = false;
    if (armed) writeAll();
    Serial.println("OK ESTOP");
    return;
  }

  // All motion commands require armed state
  if (!armed) {
    Serial.println("ERR not armed. Send ARM first.");
    return;
  }

  // SETUS us1 us2 us3 us4 us5 us6 [T ms]
  // Set positions in microseconds directly. Use -1 for HOLD.
  // With T: quintic trajectory. Without T: immediate write.
  if (startsWithIgnoreCase(s, "SETUS")) {
    String rest = s.substring(5);
    rest.trim();

    int us_vals[6];
    unsigned int ms = 0;

    int matched = sscanf(rest.c_str(), "%d %d %d %d %d %d T %u",
      &us_vals[0], &us_vals[1], &us_vals[2],
      &us_vals[3], &us_vals[4], &us_vals[5], &ms);

    if (matched < 6) {
      matched = sscanf(rest.c_str(), "%d %d %d %d %d %d t %u",
        &us_vals[0], &us_vals[1], &us_vals[2],
        &us_vals[3], &us_vals[4], &us_vals[5], &ms);
    }

    if (matched < 6) {
      Serial.println("ERR usage: SETUS us1 us2 us3 us4 us5 us6 [T ms]");
      return;
    }

    // Convert µs to firmware degrees for trajectory/write
    float tgt[N];
    for (int i = 0; i < N; i++) {
      if (us_vals[i] < 0) {
        tgt[i] = HOLD;  // Negative = hold current
      } else {
        // Clamp to servo limits
        if (us_vals[i] < SERVO_MIN_US[i]) us_vals[i] = SERVO_MIN_US[i];
        if (us_vals[i] > SERVO_MAX_US[i]) us_vals[i] = SERVO_MAX_US[i];
        // Inverse of jointDegToUs: µs → servo degrees → joint degrees
        float servoDeg = (float)(us_vals[i] - SERVO_MIN_US[i])
                       / (float)(SERVO_MAX_US[i] - SERVO_MIN_US[i])
                       * SERVO_RANGE_DEG[i];
        float sign = INVERT[i] ? -1.0f : 1.0f;
        tgt[i] = (servoDeg - ZERO_SERVO_DEG[i]) * sign;
      }
    }

    if (matched == 7 && ms > 0) {
      // Trajectory mode: smooth quintic interpolation
      startTraj(tgt, (uint32_t)ms);
    } else {
      // Immediate mode: write directly
      for (int i = 0; i < N; i++) {
        if (tgt[i] > HOLD + 0.5f) q_cur[i] = tgt[i];
      }
      writeAll();
      Serial.println("OK SETUS");
    }
    return;
  }

  // POSE name [T ms]
  if (startsWithIgnoreCase(s, "POSE")) {
    String rest = s.substring(4);
    rest.trim();

    uint32_t ms = parseTimeSuffixMs(rest, DEFAULT_T_MS);
    String name = stripTimeSuffix(rest);

    if (name.length() == 0) {
      Serial.println("ERR missing pose name");
      return;
    }

    float tgt[N];
    if (!getPoseByName(name.c_str(), tgt)) {
      Serial.print("ERR unknown pose ");
      Serial.println(name);
      return;
    }

    startTraj(tgt, ms);
    return;
  }

  // SAVE name
  if (startsWithIgnoreCase(s, "SAVE")) {
    String rest = s.substring(4);
    rest.trim();

    if (rest.length() == 0) {
      Serial.println("ERR missing pose name");
      return;
    }

    if (!savePoseByName(rest.c_str(), q_cur)) {
      Serial.println("ERR could not save pose");
      return;
    }

    Serial.print("OK SAVED ");
    Serial.println(rest);
    return;
  }

  // DELETE name
  if (startsWithIgnoreCase(s, "DELETE")) {
    String rest = s.substring(6);
    rest.trim();

    if (rest.length() == 0) {
      Serial.println("ERR missing pose name");
      return;
    }

    if (!deletePoseByName(rest.c_str())) {
      Serial.println("ERR pose not found");
      return;
    }

    Serial.print("OK DELETED ");
    Serial.println(rest);
    return;
  }

  // NUDGE joint_index delta_deg
  if (startsWithIgnoreCase(s, "NUDGE")) {
    int jointIdx = 0;
    float delta = 0.0f;

    int matched = sscanf(s.c_str(), "NUDGE %d %f", &jointIdx, &delta);
    if (matched != 2) {
      matched = sscanf(s.c_str(), "nudge %d %f", &jointIdx, &delta);
    }

    if (matched != 2) {
      Serial.println("ERR usage: NUDGE joint_index delta_deg");
      return;
    }

    if (jointIdx < 1 || jointIdx > N) {
      Serial.println("ERR joint_index must be 1..6");
      return;
    }

    float tgt[N];
    copyQ(tgt, q_cur);
    tgt[jointIdx - 1] = clampf(
      tgt[jointIdx - 1] + delta,
      JOINT_MIN_DEG[jointIdx - 1],
      JOINT_MAX_DEG[jointIdx - 1]
    );

    startTraj(tgt, 300);
    return;
  }

  // GRIP angle [T ms] — move only gripper (J6)
  if (startsWithIgnoreCase(s, "GRIP")) {
    String rest = s.substring(4);
    rest.trim();

    uint32_t ms = parseTimeSuffixMs(rest, 500);
    String angleStr = stripTimeSuffix(rest);

    if (angleStr.length() == 0) {
      Serial.println("ERR usage: GRIP angle [T ms]");
      return;
    }

    float angle = angleStr.toFloat();

    if (angle < JOINT_MIN_DEG[5] || angle > JOINT_MAX_DEG[5]) {
      Serial.print("ERR J6 range ");
      Serial.print(JOINT_MIN_DEG[5], 0);
      Serial.print("..");
      Serial.println(JOINT_MAX_DEG[5], 0);
      return;
    }

    float tgt[N];
    for (int i = 0; i < N; i++) tgt[i] = HOLD;
    tgt[5] = angle;

    startTraj(tgt, ms);
    return;
  }

  // PICKUP n angle [T ms] — full pick-and-sort cycle
  if (startsWithIgnoreCase(s, "PICKUP")) {
    String rest = s.substring(6);
    rest.trim();

    int n = 0;
    float gripAngle = 0;
    unsigned int gripMs = 500;

    int matched = sscanf(rest.c_str(), "%d %f T %u", &n, &gripAngle, &gripMs);
    if (matched < 2) {
      matched = sscanf(rest.c_str(), "%d %f t %u", &n, &gripAngle, &gripMs);
    }
    if (matched < 2) {
      Serial.println("ERR usage: PICKUP n angle [T ms]");
      Serial.println("  n = bin (1-3), angle = grip (7-70)");
      return;
    }

    if (n < 1 || n > 3) {
      Serial.println("ERR bin must be 1..3");
      return;
    }

    gripAngle = clampf(gripAngle, JOINT_MIN_DEG[5], JOINT_MAX_DEG[5]);

    abort_all = false;

    char pre[16], drop[16];
    snprintf(pre, sizeof(pre), "bin%d_pre", n);
    snprintf(drop, sizeof(drop), "bin%d_drop", n);

    // Build grip-only target
    float gripTgt[N];
    for (int i = 0; i < N; i++) gripTgt[i] = HOLD;
    gripTgt[5] = gripAngle;

    // --- DESCEND (gripper open) ---
    Serial.println("PICKUP: home");
    if (!runPoseBlocking("home", 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("PICKUP: prepick");
    if (!runPoseBlocking("prepick", 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("PICKUP: pickup");
    if (!runPoseBlocking("pickup", 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    // --- GRIP ---
    Serial.print("PICKUP: grip ");
    Serial.println(gripAngle, 1);
    startTraj(gripTgt, gripMs);
    while (traj_active) {
      serviceSerial();
      if (abort_all) { traj_active = false; writeAll(); Serial.println("ABORTED"); return; }
      updateTraj();
      delay(DT_MS);
    }
    delay(200);  // let grip settle

    // --- ASCEND (carrying) ---
    Serial.println("PICKUP: prepick_carry");
    if (!runPoseBlocking("prepick_carry", 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("PICKUP: home_carry");
    if (!runPoseBlocking("home_carry", 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    // --- SORT (1000ms for large J1 rotations) ---
    Serial.println("PICKUP: bin_pre");
    if (!runPoseBlocking(pre, 1000)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("PICKUP: bin_drop");
    if (!runPoseBlocking(drop, 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("PICKUP: bin_pre (retreat)");
    if (!runPoseBlocking(pre, 700)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("PICKUP: home (final)");
    if (!runPoseBlocking("home", 1000)) { Serial.println(abort_all ? "ABORTED" : "ERR"); return; }

    Serial.println("DONE PICKUP");
    return;
  }

  // RUNBIN n
  if (startsWithIgnoreCase(s, "RUNBIN")) {
    String rest = s.substring(6);
    rest.trim();

    int n = rest.toInt();
    if (n < 1 || n > 3) {
      Serial.println("ERR RUNBIN 1|2|3");
      return;
    }

    abort_all = false;

    char pre[16];
    char drop[16];
    snprintf(pre, sizeof(pre), "bin%d_pre", n);
    snprintf(drop, sizeof(drop), "bin%d_drop", n);

    if (!runPoseBlocking("home", 2500)) {
      Serial.println(abort_all ? "ABORTED RUNBIN" : "ERR RUNBIN home");
      return;
    }
    if (!runPoseBlocking(pre, 2500)) {
      Serial.println(abort_all ? "ABORTED RUNBIN" : "ERR RUNBIN pre");
      return;
    }
    if (!runPoseBlocking(drop, 2000)) {
      Serial.println(abort_all ? "ABORTED RUNBIN" : "ERR RUNBIN drop");
      return;
    }
    if (!runPoseBlocking(pre, 2000)) {
      Serial.println(abort_all ? "ABORTED RUNBIN" : "ERR RUNBIN pre");
      return;
    }
    if (!runPoseBlocking("home", 2500)) {
      Serial.println(abort_all ? "ABORTED RUNBIN" : "ERR RUNBIN home");
      return;
    }

    Serial.println("DONE RUNBIN");
    return;
  }

  // SET j1..j6 [T ms]
  {
    float a1, a2, a3, a4, a5, a6;
    unsigned int ms = 0;

    int matched = sscanf(
      s.c_str(),
      "SET %f %f %f %f %f %f T %u",
      &a1, &a2, &a3, &a4, &a5, &a6, &ms
    );

    if (matched == 7) {
      float tgt[N] = {a1, a2, a3, a4, a5, a6};
      startTraj(tgt, (uint32_t)ms);
      return;
    }

    matched = sscanf(
      s.c_str(),
      "set %f %f %f %f %f %f t %u",
      &a1, &a2, &a3, &a4, &a5, &a6, &ms
    );

    if (matched == 7) {
      float tgt[N] = {a1, a2, a3, a4, a5, a6};
      startTraj(tgt, (uint32_t)ms);
      return;
    }

    matched = sscanf(
      s.c_str(),
      "SET %f %f %f %f %f %f",
      &a1, &a2, &a3, &a4, &a5, &a6
    );

    if (matched == 6) {
      float tgt[N] = {a1, a2, a3, a4, a5, a6};
      startTraj(tgt, DEFAULT_T_MS);
      return;
    }

    matched = sscanf(
      s.c_str(),
      "set %f %f %f %f %f %f",
      &a1, &a2, &a3, &a4, &a5, &a6
    );

    if (matched == 6) {
      float tgt[N] = {a1, a2, a3, a4, a5, a6};
      startTraj(tgt, DEFAULT_T_MS);
      return;
    }
  }

  // GOTO j1..j6
  {
    float a1, a2, a3, a4, a5, a6;

    int matched = sscanf(
      s.c_str(),
      "GOTO %f %f %f %f %f %f",
      &a1, &a2, &a3, &a4, &a5, &a6
    );

    if (matched != 6) {
      matched = sscanf(
        s.c_str(),
        "goto %f %f %f %f %f %f",
        &a1, &a2, &a3, &a4, &a5, &a6
      );
    }

    if (matched == 6) {
      traj_active = false;
      abort_all = false;

      q_cur[0] = clampf(a1, JOINT_MIN_DEG[0], JOINT_MAX_DEG[0]);
      q_cur[1] = clampf(a2, JOINT_MIN_DEG[1], JOINT_MAX_DEG[1]);
      q_cur[2] = clampf(a3, JOINT_MIN_DEG[2], JOINT_MAX_DEG[2]);
      q_cur[3] = clampf(a4, JOINT_MIN_DEG[3], JOINT_MAX_DEG[3]);
      q_cur[4] = clampf(a5, JOINT_MIN_DEG[4], JOINT_MAX_DEG[4]);
      q_cur[5] = clampf(a6, JOINT_MIN_DEG[5], JOINT_MAX_DEG[5]);

      writeAll();
      Serial.println("OK GOTO");
      return;
    }
  }

  Serial.print("ERR ");
  Serial.println(s);
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  // SAFETY: Drive wheel motor pins LOW immediately to prevent wheels from spinning.
  // These are MD10C PWM/DIR pins that float HIGH when arm firmware is loaded.
  for (int i = 0; i < NUM_MOTOR_PINS; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    digitalWrite(MOTOR_PINS[i], LOW);
  }

  // SAFETY: Drive servo pins LOW before any blocking code.
  // Prevents powered servos from spazzing on floating pins while
  // waiting for serial connection or during setup delay.
  for (int i = 0; i < N; i++) {
    pinMode(SERVO_PINS[i], OUTPUT);
    digitalWrite(SERVO_PINS[i], LOW);
  }

  // Encoder pins (3.3V, no level shifter needed)
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightISR_B, CHANGE);

  Serial.begin(115200);
  delay(500);

  initPoseTable();

  // Servos are NOT attached here — they start disarmed.
  // User must send ARM after manually positioning the arm.

  for (int i = 0; i < N; i++) {
    q_cur[i] = clampf(STARTUP_Q[i], JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  }

  Serial.println();
  Serial.println("TEENSY ARM+DRIVE READY");
  Serial.println("Servos DISARMED. Position arm by hand, then send ARM.");
  Serial.println("Drive motors ready (DRIVE/TICKS/ESTOP — no ARM needed).");
  Serial.println();
  printCompactQ();
  printHelp();
  Serial.print("> ");  // initial prompt
}

// PI controller timing
static uint32_t lastPiMs = 0;

void loop() {
  serviceSerial();

  uint32_t now = millis();

  // Servo trajectory update at 50Hz (20ms)
  if (now - last_update_ms >= DT_MS) {
    last_update_ms = now;
    updateTraj();
  }

  // PI velocity controller update at 100Hz (10ms)
  if (pi_active && (now - lastPiMs >= (uint32_t)PI_DT_MS)) {
    lastPiMs = now;
    piControllerUpdate();
  }

  // Motor safety timeout: stop wheels if no DRIVE/SETVEL command in 500ms
  if (driveActive && (now - lastDriveMs > 500)) {
    driveMotors(0, 0);
    driveActive = false;
    if (pi_active) piReset();
  }
}
