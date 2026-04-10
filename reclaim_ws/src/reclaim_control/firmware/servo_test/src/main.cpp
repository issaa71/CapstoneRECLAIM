#include <Arduino.h>
#include <Servo.h>

const int NUM_SERVOS = 6;
const int SERVO_PINS[NUM_SERVOS] = {10, 11, 12, 13, 14, 15};
const char* SERVO_NAMES[NUM_SERVOS] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Rotate", "Gripper"
};

Servo servos[NUM_SERVOS];
bool attached[NUM_SERVOS] = {false, false, false, false, false, false};
int selected = 0;

// Current pulse width in microseconds for each servo
int currentUs[NUM_SERVOS] = {1500, 1500, 1500, 1500, 1500, 1500};

// Initial attach pulse per servo (where z sends on first attach)
int attachUs[NUM_SERVOS] = {1500, 1500, 1500, 1500, 1500, 1500};

// Calibrated min/max pulse widths per servo (user sets these with [ and ])
int calMin[NUM_SERVOS] = {500, 500, 500, 500, 500, 500};
int calMax[NUM_SERVOS] = {2500, 2500, 2500, 2500, 2500, 2500};

// Absolute hardware limits — wide range to allow full exploration
const int HARD_MIN = 400;
const int HARD_MAX = 2600;

// Step sizes
const int STEP_COARSE = 10;   // us per coarse step
const int STEP_FINE = 2;      // us per fine step

// ---------------- ZERO REFERENCE + JOINT LIMITS (relative) ----------------
// zeroUs = pulse where you want joint angle = 0
int   zeroUs[NUM_SERVOS]  = {1500, 1500, 1500, 1500, 1500, 1500};
// zeroDeg = mapped degrees at zeroUs (computed from calMin/calMax)
float zeroDeg[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};

// saved joint limits in "relative degrees" (degRel = servoDeg - zeroDeg)
float jointMinDegRel[NUM_SERVOS] = {-999, -999, -999, -999, -999, -999};
float jointMaxDegRel[NUM_SERVOS] = { 999,  999,  999,  999,  999,  999};
// --------------------------------------------------------------------------

// J1-J3 = DS3218 (270°), J4-J6 = MG996R (180°)
const int SERVO_RANGE_DEG[NUM_SERVOS] = {270, 270, 270, 180, 180, 180};

int usToDegrees(int idx, int us) {
    if (calMax[idx] == calMin[idx]) return 0;
    long num = (long)(us - calMin[idx]) * SERVO_RANGE_DEG[idx];
    long den = (calMax[idx] - calMin[idx]);
    return (int)(num / den);
}

float jointDegreesRel(int idx, int us) {
    float servoDeg = (float)usToDegrees(idx, us);
    return servoDeg - zeroDeg[idx];
}

void ensureAttached(int idx) {
    if (!attached[idx]) {
        servos[idx].attach(SERVO_PINS[idx], HARD_MIN, HARD_MAX);
        attached[idx] = true;
    }
}

void moveToUs(int us) {
    ensureAttached(selected);
    currentUs[selected] = constrain(us, HARD_MIN, HARD_MAX);
    servos[selected].writeMicroseconds(currentUs[selected]);

    int servoDeg = usToDegrees(selected, currentUs[selected]);
    float jDeg = jointDegreesRel(selected, currentUs[selected]);

    Serial.print("[");
    Serial.print(SERVO_NAMES[selected]);
    Serial.print("] ");
    Serial.print(currentUs[selected]);
    Serial.print("us  servo=");
    Serial.print(servoDeg);
    Serial.print("°  joint=");
    Serial.print(jDeg, 1);
    Serial.print("°  cal:[");
    Serial.print(calMin[selected]);
    Serial.print("-");
    Serial.print(calMax[selected]);
    Serial.print("]  zeroUs=");
    Serial.print(zeroUs[selected]);
    Serial.print(" (zeroDeg=");
    Serial.print(zeroDeg[selected], 1);
    Serial.println(")");
}

void printHelp() {
    Serial.println();
    Serial.println("=== SERVO TEST — Raw us + Joint Zero ===");
    Serial.println("  1-6 = select servo:");
    Serial.println("        1=Base  2=Shoulder  3=Elbow");
    Serial.println("        4=WristPitch  5=WristRotate  6=Gripper");
    Serial.println();
    Serial.println("  d/a = +/- 10us (coarse)");
    Serial.println("  D/A = +/- 2us (fine)");
    Serial.println("  c   = center (1500us)");
    Serial.println();
    Serial.println("  CALIBRATION (pulse range):");
    Serial.println("  [   = set current pulse as calMin (0°)");
    Serial.println("  ]   = set current pulse as calMax (full range°)");
    Serial.println();
    Serial.println("  JOINT ZERO + LIMITS (relative):");
    Serial.println("  z   = set JOINT 0° (attaches servo if off — hold arm!)");
    Serial.println("  m   = save JOINT MIN (relative deg)");
    Serial.println("  M   = save JOINT MAX (relative deg)");
    Serial.println();
    Serial.println("  p   = print all servos");
    Serial.println("  h   = help");
    Serial.println("=========================================");
    Serial.println();
}

void printAll() {
    Serial.println();
    for (int i = 0; i < NUM_SERVOS; i++) {
        int servoDeg = usToDegrees(i, currentUs[i]);
        float jDeg = jointDegreesRel(i, currentUs[i]);

        Serial.print(i == selected ? " > " : "   ");
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.print(SERVO_NAMES[i]);
        Serial.print(": ");
        Serial.print(currentUs[i]);
        Serial.print("us  servo=");
        Serial.print(servoDeg);
        Serial.print("°  joint=");
        Serial.print(jDeg, 1);
        Serial.print("°  cal:[");
        Serial.print(calMin[i]);
        Serial.print("-");
        Serial.print(calMax[i]);
        Serial.print("]  zeroUs=");
        Serial.print(zeroUs[i]);
        Serial.print("  jointLim:[");
        Serial.print(jointMinDegRel[i], 1);
        Serial.print(", ");
        Serial.print(jointMaxDegRel[i], 1);
        Serial.println("]");
    }
    Serial.println();
}

void updateZeroDegFor(int idx) {
    zeroDeg[idx] = (float)usToDegrees(idx, zeroUs[idx]);
}

void setup() {
    // Immediately disable motor drivers (pins 2-5) to prevent floating PWM
    const int MOTOR_PINS[] = {2, 3, 4, 5};
    for (int pin : MOTOR_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // Servos start DETACHED — no PWM signal until user moves them
    // MUST be before while(!Serial) or pins float while waiting for serial monitor
    for (int i = 0; i < NUM_SERVOS; i++) {
        pinMode(SERVO_PINS[i], OUTPUT);
        digitalWrite(SERVO_PINS[i], LOW);
    }

    Serial.begin(115200);
    while (!Serial) { ; }

    // Initialize zeroDeg based on initial calMin/calMax and zeroUs
    for (int i = 0; i < NUM_SERVOS; i++) {
        updateZeroDegFor(i);
    }

    printHelp();
    Serial.println("  ** Servos are OFF — move a servo to activate it **");
    Serial.println();
    printAll();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case '1': case '2': case '3':
            case '4': case '5': case '6':
                selected = c - '1';
                Serial.print("Selected: ");
                Serial.print(SERVO_NAMES[selected]);
                Serial.print(" @ ");
                Serial.print(currentUs[selected]);
                Serial.println("us");
                break;

            case 'd': moveToUs(currentUs[selected] + STEP_COARSE); break;
            case 'a': moveToUs(currentUs[selected] - STEP_COARSE); break;
            case 'D': moveToUs(currentUs[selected] + STEP_FINE); break;
            case 'A': moveToUs(currentUs[selected] - STEP_FINE); break;
            case 'c': moveToUs(1500); break;

            case '[':
                calMin[selected] = currentUs[selected];
                updateZeroDegFor(selected);
                Serial.print(">> ");
                Serial.print(SERVO_NAMES[selected]);
                Serial.print(" calMin set to ");
                Serial.print(calMin[selected]);
                Serial.println("us (= 0° mapped)");
                break;

            case ']':
                calMax[selected] = currentUs[selected];
                updateZeroDegFor(selected);
                Serial.print(">> ");
                Serial.print(SERVO_NAMES[selected]);
                Serial.print(" calMax set to ");
                Serial.print(calMax[selected]);
                Serial.print("us (= ");
                Serial.print(SERVO_RANGE_DEG[selected]);
                Serial.println("° mapped)");
                break;

            case 'z':
                // If servo is not attached yet, attach at attachUs[] pulse
                // Position arm near that angle by hand BEFORE pressing z!
                if (!attached[selected]) {
                    Serial.print(">> Attaching servo at ");
                    Serial.print(attachUs[selected]);
                    Serial.println("us — hold the arm!");
                    ensureAttached(selected);
                    currentUs[selected] = attachUs[selected];
                    servos[selected].writeMicroseconds(attachUs[selected]);
                    delay(500); // let it settle
                }
                zeroUs[selected] = currentUs[selected];
                updateZeroDegFor(selected);
                Serial.print(">> ");
                Serial.print(SERVO_NAMES[selected]);
                Serial.print(" JOINT ZERO set at ");
                Serial.print(zeroUs[selected]);
                Serial.print("us (zeroDeg=");
                Serial.print(zeroDeg[selected], 1);
                Serial.println("°)");
                break;

            case 'm': {
                float j = jointDegreesRel(selected, currentUs[selected]);
                jointMinDegRel[selected] = j;
                Serial.print(">> ");
                Serial.print(SERVO_NAMES[selected]);
                Serial.print(" JOINT MIN saved at ");
                Serial.print(j, 1);
                Serial.println("° (relative)");
                break;
            }
            case 'M': {
                float j = jointDegreesRel(selected, currentUs[selected]);
                jointMaxDegRel[selected] = j;
                Serial.print(">> ");
                Serial.print(SERVO_NAMES[selected]);
                Serial.print(" JOINT MAX saved at ");
                Serial.print(j, 1);
                Serial.println("° (relative)");
                break;
            }

            case 'p': printAll(); break;
            case 'h': printHelp(); break;
        }
    }
}
