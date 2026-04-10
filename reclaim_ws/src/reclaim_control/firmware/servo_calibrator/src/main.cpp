// servo_calibrator — Single-joint microsecond range finder for 6DOF arm
// ALL servos stay attached and holding position. Only the selected joint moves.

#include <Arduino.h>
#include <Servo.h>

// ---- Pin assignments ----
static const int NUM_SERVOS = 6;
static const int SERVO_PINS[NUM_SERVOS] = {10, 11, 12, 13, 14, 15};  // Updated March 18
static const int MOTOR_PINS[] = {2, 3, 22, 20};  // L(PWM=2,DIR=3) R(PWM=22,DIR=20)

static const char* JOINT_NAMES[NUM_SERVOS] = {
    "J1 Base", "J2 Shoulder", "J3 Elbow",
    "J4 WristPitch", "J5 WristRotate", "J6 Gripper"
};

// ---- Hardware limits ----
static const int HARD_MIN = 400;
static const int HARD_MAX = 2600;
static const int MIDPOINT = 1500;

// ---- Step sizes ----
static const int STEP_COARSE = 50;
static const int STEP_MEDIUM = 10;
static const int STEP_FINE   = 1;

// ---- State ----
enum State { INIT, JOINT_SELECT, CALIBRATING };

static State state = INIT;
static Servo servos[NUM_SERVOS];
static int servoUs[NUM_SERVOS];       // current µs for each servo
static bool servosAttached = false;
static bool servoAttached[NUM_SERVOS] = {false}; // per-servo attach tracking

static int selectedJoint = -1;
static int recordedLow   = -1;
static int recordedHigh  = -1;

// ---- Helpers ----

static void printPosition() {
    Serial.print("[");
    Serial.print(JOINT_NAMES[selectedJoint]);
    Serial.print("] ");
    Serial.print(servoUs[selectedJoint]);
    Serial.print("us");
    if (recordedLow >= 0) {
        Serial.print("  LOW=");
        Serial.print(recordedLow);
    }
    if (recordedHigh >= 0) {
        Serial.print("  HIGH=");
        Serial.print(recordedHigh);
    }
    Serial.println();
}

static void printAllPositions() {
    Serial.println("Current servo positions:");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print("  ");
        Serial.print(JOINT_NAMES[i]);
        Serial.print(": ");
        if (servoAttached[i]) {
            Serial.print(servoUs[i]);
            Serial.print("us");
        } else {
            Serial.print("(not attached)");
        }
        if (i == selectedJoint) Serial.print(" <-- SELECTED");
        Serial.println();
    }
}

static void printHelp() {
    Serial.println();
    Serial.println("=== SERVO CALIBRATOR ===");
    Serial.println("  a/d = nudge -/+ 50us (coarse)");
    Serial.println("  z/x = nudge -/+ 10us (medium)");
    Serial.println("  A/D = nudge -/+ 1us (fine)");
    Serial.println("  l   = record current as LOW limit");
    Serial.println("  h   = record current as HIGH limit");
    Serial.println("  m   = midpoint (1500us)");
    Serial.println("  c   = center between LOW and HIGH");
    Serial.println("  s   = sweep LOW -> HIGH -> LOW");
    Serial.println("  r   = print results + copy-paste line");
    Serial.println("  p   = print all servo positions");
    Serial.println("  q   = pick another joint (all stay holding)");
    Serial.println("  ?   = this help");
    Serial.println("========================");
    Serial.println();
}

static void printResults() {
    Serial.println();
    Serial.println("========== CALIBRATION RESULTS ==========");
    Serial.print("Joint: ");
    Serial.println(JOINT_NAMES[selectedJoint]);

    Serial.print("  LOW:  ");
    if (recordedLow >= 0) { Serial.print(recordedLow); Serial.println("us"); }
    else Serial.println("(not recorded)");

    Serial.print("  HIGH: ");
    if (recordedHigh >= 0) { Serial.print(recordedHigh); Serial.println("us"); }
    else Serial.println("(not recorded)");

    if (recordedLow >= 0 && recordedHigh >= 0) {
        int lo = min(recordedLow, recordedHigh);
        int hi = max(recordedLow, recordedHigh);
        Serial.print("  Range: ");
        Serial.print(hi - lo);
        Serial.println("us");
        Serial.println();
        Serial.println("--- COPY-PASTE FOR FIRMWARE ---");
        Serial.print("{");
        Serial.print(lo);
        Serial.print(", ");
        Serial.print(hi);
        Serial.print("} // ");
        Serial.println(JOINT_NAMES[selectedJoint]);
        Serial.println("-------------------------------");
    }
    Serial.println("=========================================");
    Serial.println();
}

static void doSweep() {
    if (recordedLow < 0 || recordedHigh < 0) {
        Serial.println("ERR: Record both LOW (l) and HIGH (h) first.");
        return;
    }

    int lo = min(recordedLow, recordedHigh);
    int hi = max(recordedLow, recordedHigh);

    // Move to start
    servoUs[selectedJoint] = lo;
    servos[selectedJoint].writeMicroseconds(lo);
    delay(500);

    // Sweep lo → hi
    Serial.print("Sweeping ");
    Serial.print(lo);
    Serial.print("us -> ");
    Serial.print(hi);
    Serial.println("us ...");
    for (int us = lo; us <= hi; us += 5) {
        servos[selectedJoint].writeMicroseconds(us);
        delay(20);
    }
    servos[selectedJoint].writeMicroseconds(hi);
    delay(500);

    // Sweep hi → lo
    Serial.print("Sweeping ");
    Serial.print(hi);
    Serial.print("us -> ");
    Serial.print(lo);
    Serial.println("us ...");
    for (int us = hi; us >= lo; us -= 5) {
        servos[selectedJoint].writeMicroseconds(us);
        delay(20);
    }
    servos[selectedJoint].writeMicroseconds(lo);

    servoUs[selectedJoint] = lo;
    Serial.println("Sweep complete.");
    printPosition();
}

// ---- Nudge helper ----

static void nudge(int delta) {
    servoUs[selectedJoint] = constrain(servoUs[selectedJoint] + delta, HARD_MIN, HARD_MAX);
    servos[selectedJoint].writeMicroseconds(servoUs[selectedJoint]);
    printPosition();
}

// ---- HOME positions (verified March 18) ----
// These are the default positions servos go to on startup.
static const int HOME_US[NUM_SERVOS] = {
    1229,  // J1 Base — verified home March 18
    2153,  // J2 Shoulder — verified home
    2030,  // J3 Elbow — verified home
    1400,  // J4 WristPitch — verified home March 18
    1755,  // J5 WristRotate — verified home March 18 (horizontal grip)
    1500   // J6 Gripper — verified home March 18
};

// ---- Attach all servos ----

static void attachAll() {
    Serial.println("Attaching all servos...");
    for (int i = 0; i < NUM_SERVOS; i++) {
        servoUs[i] = HOME_US[i];
        servos[i].attach(SERVO_PINS[i], HARD_MIN, HARD_MAX);
        servos[i].writeMicroseconds(servoUs[i]);
    }
    servosAttached = true;
    Serial.println("All servos at HOME positions.");
    Serial.println();
}

// ---- State handlers ----

static void promptJointSelect() {
    Serial.println();
    Serial.println("Select joint to calibrate (all servos stay holding):");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print(" = ");
        Serial.print(JOINT_NAMES[i]);
        Serial.print(" (");
        Serial.print(servoUs[i]);
        Serial.println("us)");
    }
    Serial.print("> ");
}

static void handleJointSelect(char c) {
    if (c < '1' || c > '6') return;

    selectedJoint = c - '1';
    recordedLow = -1;
    recordedHigh = -1;

    state = CALIBRATING;

    Serial.println();
    Serial.print("Selected ");
    Serial.print(JOINT_NAMES[selectedJoint]);
    Serial.print(" (pin ");
    Serial.print(SERVO_PINS[selectedJoint]);
    Serial.print(") at ");
    Serial.print(servoUs[selectedJoint]);
    Serial.println("us. Other servos keep holding.");
    printHelp();
    printPosition();
}

static void handleCalibrating(char c) {
    switch (c) {
        case 'a': nudge(-STEP_COARSE); break;
        case 'd': nudge(+STEP_COARSE); break;
        case 'z': nudge(-STEP_MEDIUM); break;
        case 'x': nudge(+STEP_MEDIUM); break;
        case 'A': nudge(-STEP_FINE);   break;
        case 'D': nudge(+STEP_FINE);   break;

        case 'l':
            recordedLow = servoUs[selectedJoint];
            Serial.print(">> LOW limit recorded: ");
            Serial.print(recordedLow);
            Serial.println("us");
            break;

        case 'h':
            recordedHigh = servoUs[selectedJoint];
            Serial.print(">> HIGH limit recorded: ");
            Serial.print(recordedHigh);
            Serial.println("us");
            break;

        case 'm':
            servoUs[selectedJoint] = MIDPOINT;
            servos[selectedJoint].writeMicroseconds(MIDPOINT);
            printPosition();
            break;

        case 'c':
            if (recordedLow < 0 || recordedHigh < 0) {
                Serial.println("ERR: Record both LOW (l) and HIGH (h) first.");
            } else {
                servoUs[selectedJoint] = (recordedLow + recordedHigh) / 2;
                servos[selectedJoint].writeMicroseconds(servoUs[selectedJoint]);
                printPosition();
            }
            break;

        case 's': doSweep(); break;
        case 'r': printResults(); break;
        case 'p': printAllPositions(); break;
        case '?': printHelp(); break;

        case 'q':
            Serial.print("Switching from ");
            Serial.print(JOINT_NAMES[selectedJoint]);
            Serial.print(" (holding at ");
            Serial.print(servoUs[selectedJoint]);
            Serial.println("us)");
            selectedJoint = -1;
            state = JOINT_SELECT;
            promptJointSelect();
            break;

        default:
            break;
    }
}

// ---- Text command buffer ----
static char cmdBuf[64];
static int cmdLen = 0;

// Helper: attach a single servo if not already attached
static void ensureAttached(int idx) {
    if (!servoAttached[idx]) {
        servos[idx].attach(SERVO_PINS[idx], HARD_MIN, HARD_MAX);
        servoAttached[idx] = true;
        servosAttached = true;  // at least one is attached
        Serial.print("  -> Attached ");
        Serial.print(JOINT_NAMES[idx]);
        Serial.print(" on pin ");
        Serial.println(SERVO_PINS[idx]);
    }
}

static bool handleTextCommand(const char* cmd) {
    int joint, value;

    // nudge <joint> <delta_us>  — e.g. "nudge 6 150"
    if (sscanf(cmd, "nudge %d %d", &joint, &value) == 2) {
        if (joint < 1 || joint > 6) {
            Serial.println("ERR: joint must be 1-6");
            return true;
        }
        int idx = joint - 1;
        if (!servoAttached[idx]) {
            Serial.println("ERR: servo not attached yet. Use 'set' first.");
            return true;
        }
        servoUs[idx] = constrain(servoUs[idx] + value, HARD_MIN, HARD_MAX);
        servos[idx].writeMicroseconds(servoUs[idx]);
        Serial.print("[");
        Serial.print(JOINT_NAMES[idx]);
        Serial.print("] ");
        Serial.print(servoUs[idx]);
        Serial.println("us");
        return true;
    }

    // set <joint> home  — e.g. "set 1 home" (auto-attaches at home position)
    {
        int j;
        char word[16];
        if (sscanf(cmd, "set %d %s", &j, word) == 2 && strncmp(word, "home", 4) == 0) {
            if (j < 1 || j > 6) {
                Serial.println("ERR: joint must be 1-6");
                return true;
            }
            int idx = j - 1;
            ensureAttached(idx);
            servoUs[idx] = HOME_US[idx];
            servos[idx].writeMicroseconds(servoUs[idx]);
            Serial.print("[");
            Serial.print(JOINT_NAMES[idx]);
            Serial.print("] HOME -> ");
            Serial.print(servoUs[idx]);
            Serial.println("us");
            return true;
        }
    }

    // set <joint> <us>  — e.g. "set 1 1500" (auto-attaches servo)
    if (sscanf(cmd, "set %d %d", &joint, &value) == 2) {
        if (joint < 1 || joint > 6) {
            Serial.println("ERR: joint must be 1-6");
            return true;
        }
        int idx = joint - 1;
        ensureAttached(idx);
        servoUs[idx] = constrain(value, HARD_MIN, HARD_MAX);
        servos[idx].writeMicroseconds(servoUs[idx]);
        Serial.print("[");
        Serial.print(JOINT_NAMES[idx]);
        Serial.print("] ");
        Serial.print(servoUs[idx]);
        Serial.println("us");
        return true;
    }

    // print — show all positions
    if (strncmp(cmd, "print", 5) == 0) {
        printAllPositions();
        return true;
    }

    // help
    if (strncmp(cmd, "help", 4) == 0) {
        Serial.println();
        Serial.println("=== TEXT COMMANDS ===");
        Serial.println("  nudge <joint> <delta_us>  — e.g. nudge 6 150");
        Serial.println("  set <joint> <us>          — e.g. set 4 1500");
        Serial.println("  print                     — show all positions");
        Serial.println("  help                      — this help");
        Serial.println("  cal                       — enter single-key calibrator mode");
        Serial.println("=====================");
        Serial.println();
        return true;
    }

    // cal — switch to single-key calibrator mode
    if (strncmp(cmd, "cal", 3) == 0) {
        state = JOINT_SELECT;
        promptJointSelect();
        return true;
    }

    return false;
}

// ---- Arduino entry points ----

void setup() {
    // Safety: disable motor drivers immediately
    for (int pin : MOTOR_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // Safety: all servo pins LOW initially (no floating → no spaz)
    for (int i = 0; i < NUM_SERVOS; i++) {
        pinMode(SERVO_PINS[i], OUTPUT);
        digitalWrite(SERVO_PINS[i], LOW);
    }

    Serial.begin(115200);
    while (!Serial) { ; }

    Serial.println();
    Serial.println("================================");
    Serial.println("  RECLAIM Servo Calibrator v6");
    Serial.println("  AUTO HOME on startup");
    Serial.println("  Verified March 18");
    Serial.println("================================");
    Serial.println();

    // Attach all servos at HOME positions
    Serial.println("Attaching all servos at HOME...");
    for (int i = 0; i < NUM_SERVOS; i++) {
        servoUs[i] = HOME_US[i];
        servos[i].attach(SERVO_PINS[i], HARD_MIN, HARD_MAX);
        servos[i].writeMicroseconds(servoUs[i]);
        servoAttached[i] = true;
        Serial.print("  J");
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.print(JOINT_NAMES[i]);
        Serial.print(" -> ");
        Serial.print(HOME_US[i]);
        Serial.println("us");
        delay(200);  // stagger to avoid current spike
    }
    servosAttached = true;
    Serial.println("All servos at HOME.");
    Serial.println();
    Serial.println("Commands: set, nudge, print, help");

    state = INIT;
    cmdLen = 0;
    Serial.println();
    Serial.print("> ");
}

static bool inEscSeq = false;  // true while consuming an escape sequence

void loop() {
    if (!Serial.available()) return;
    char c = Serial.read();

    // ---- Filter escape sequences (arrow keys, etc.) ----
    // ESC [ <chars> sends 0x1B, '[', then one or more chars.
    // Drop the entire sequence so it doesn't corrupt the buffer.
    if (c == 0x1B) {  // ESC
        inEscSeq = true;
        return;
    }
    if (inEscSeq) {
        // ESC sequences end with a letter (A-Z, a-z)
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
            inEscSeq = false;  // sequence complete, drop it
        }
        // Also end on '[' only if it's the second char (CSI intro) — keep consuming
        return;
    }

    // ---- Backspace / Delete ----
    if (c == 0x7F || c == 0x08) {  // DEL or BS
        if (cmdLen > 0) {
            cmdLen--;
            // Echo: erase the character on screen
            Serial.print("\b \b");
        }
        return;
    }

    // ---- Enter: process command ----
    if (c == '\n' || c == '\r') {
        Serial.println();  // newline on screen
        if (cmdLen > 0) {
            cmdBuf[cmdLen] = '\0';
            handleTextCommand(cmdBuf);
            cmdLen = 0;
        }
        Serial.print("> ");  // prompt
        return;
    }

    // ---- Printable characters ----
    if (c >= 32 && c < 127 && cmdLen < 62) {
        cmdBuf[cmdLen++] = c;
        Serial.print(c);  // echo the character
    }
}
