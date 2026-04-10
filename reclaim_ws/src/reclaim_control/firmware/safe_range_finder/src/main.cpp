#include <Arduino.h>
#include <Servo.h>

// ============================================================
// SAFE RANGE FINDER — Find mechanical limits for 5 joints
// Skips J2 (Shoulder, pin 11) because it's busted.
//
// WORKFLOW:
//   1. Select a joint (1, 3, 4, 5, or 6)
//   2. It starts at 90 degrees (center)
//   3. Nudge with 'a'/'d' (5 deg) or 'A'/'D' (1 deg)
//   4. When you hit the mechanical limit going DOWN, press 'L'
//      to record the low limit
//   5. Nudge back up. When you hit the limit going UP, press 'U'
//      to record the high limit
//   6. Press 'r' to print the full results table
//   7. Repeat for all 5 joints
//
// IMPORTANT: Go slow! Listen for grinding/stalling. Back off
// a few degrees from where it stalls — that's your safe limit.
// ============================================================

const int NUM_JOINTS = 6;
const int SERVO_PINS[NUM_JOINTS] = {10, 11, 12, 13, 14, 15};
const char* JOINT_NAMES[NUM_JOINTS] = {
    "J1 Base", "J2 Shoulder (SKIP)", "J3 Elbow",
    "J4 Wrist Pitch", "J5 Wrist Rotate", "J6 Gripper"
};

// J2 is index 1 — we skip it entirely
const int SKIP_JOINT = 1;

Servo servos[NUM_JOINTS];
int angles[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
int selected = 0;  // start on J1

// Recorded limits (-1 means not yet recorded)
int limit_low[NUM_JOINTS]  = {-1, -1, -1, -1, -1, -1};
int limit_high[NUM_JOINTS] = {-1, -1, -1, -1, -1, -1};

void printStatus() {
    Serial.print("  [");
    Serial.print(JOINT_NAMES[selected]);
    Serial.print("] angle = ");
    Serial.print(angles[selected]);
    Serial.print("°");
    if (limit_low[selected] >= 0) {
        Serial.print("  low=");
        Serial.print(limit_low[selected]);
    }
    if (limit_high[selected] >= 0) {
        Serial.print("  high=");
        Serial.print(limit_high[selected]);
    }
    Serial.println();
}

void moveToAngle(int angle) {
    if (selected == SKIP_JOINT) {
        Serial.println("  ** J2 is disabled. Select a different joint. **");
        return;
    }
    angles[selected] = constrain(angle, 0, 180);
    servos[selected].write(angles[selected]);
    printStatus();
}

void printResults() {
    Serial.println();
    Serial.println("╔════════════════════════════════════════════════╗");
    Serial.println("║         SAFE RANGE RESULTS                    ║");
    Serial.println("╠════════════════════════════════════════════════╣");
    Serial.println("║  Joint            Current   Low    High       ║");
    Serial.println("╠════════════════════════════════════════════════╣");
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i == SKIP_JOINT) {
            Serial.println("║  J2 Shoulder       ---    SKIPPED             ║");
            continue;
        }
        Serial.print("║  ");
        Serial.print(JOINT_NAMES[i]);
        // pad name
        int nameLen = strlen(JOINT_NAMES[i]);
        for (int p = nameLen; p < 18; p++) Serial.print(" ");

        // current angle
        if (angles[i] < 100) Serial.print(" ");
        if (angles[i] < 10)  Serial.print(" ");
        Serial.print(angles[i]);
        Serial.print("°     ");

        // low limit
        if (limit_low[i] >= 0) {
            if (limit_low[i] < 100) Serial.print(" ");
            if (limit_low[i] < 10)  Serial.print(" ");
            Serial.print(limit_low[i]);
            Serial.print("°   ");
        } else {
            Serial.print("  --    ");
        }

        // high limit
        if (limit_high[i] >= 0) {
            if (limit_high[i] < 100) Serial.print(" ");
            if (limit_high[i] < 10)  Serial.print(" ");
            Serial.print(limit_high[i]);
            Serial.print("°");
        } else {
            Serial.print("  --");
        }

        // pad to end of box
        Serial.println("       ║");
    }
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println();

    // Print copy-paste friendly format
    Serial.println("--- COPY THESE INTO CHAT ---");
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i == SKIP_JOINT) {
            Serial.println("J2: SKIPPED (busted)");
            continue;
        }
        Serial.print("J");
        Serial.print(i + 1);
        Serial.print(": low=");
        if (limit_low[i] >= 0) Serial.print(limit_low[i]);
        else Serial.print("??");
        Serial.print(" high=");
        if (limit_high[i] >= 0) Serial.print(limit_high[i]);
        else Serial.print("??");
        Serial.println();
    }
    Serial.println("----------------------------");
    Serial.println();
}

void printHelp() {
    Serial.println();
    Serial.println("=== SAFE RANGE FINDER (J2 Disabled) ===");
    Serial.println("  1     = select J1 (Base)");
    Serial.println("  3     = select J3 (Elbow)");
    Serial.println("  4     = select J4 (Wrist Pitch)");
    Serial.println("  5     = select J5 (Wrist Rotate)");
    Serial.println("  6     = select J6 (Gripper)");
    Serial.println("  d/a   = nudge +/- 5 degrees");
    Serial.println("  D/A   = nudge +/- 1 degree (fine)");
    Serial.println("  c     = center at 90");
    Serial.println("  L     = record current angle as LOW limit");
    Serial.println("  U     = record current angle as HIGH limit");
    Serial.println("  r     = print results table");
    Serial.println("  p     = print all current angles");
    Serial.println("  h     = this help");
    Serial.println("========================================");
    Serial.println();
    Serial.println("PROCEDURE:");
    Serial.println("  1. Select joint");
    Serial.println("  2. Nudge DOWN with 'a' until it stalls/grinds");
    Serial.println("  3. Back off 2-3 degrees, press 'L'");
    Serial.println("  4. Nudge UP with 'd' until it stalls/grinds");
    Serial.println("  5. Back off 2-3 degrees, press 'U'");
    Serial.println("  6. Repeat for all joints, then press 'r'");
    Serial.println();
}

void printAll() {
    Serial.println();
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i == SKIP_JOINT) {
            Serial.println("   2. J2 Shoulder -- DISABLED");
            continue;
        }
        Serial.print(i == selected ? " > " : "   ");
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.print(JOINT_NAMES[i]);
        Serial.print(": ");
        Serial.print(angles[i]);
        Serial.print("°");
        if (limit_low[i] >= 0) {
            Serial.print("  [low=");
            Serial.print(limit_low[i]);
            Serial.print("]");
        }
        if (limit_high[i] >= 0) {
            Serial.print("  [high=");
            Serial.print(limit_high[i]);
            Serial.print("]");
        }
        Serial.println();
    }
    Serial.println();
}

void setup() {
    // Disable motor driver pins to prevent floating PWM
    const int MOTOR_PINS[] = {2, 3, 4, 5};
    for (int pin : MOTOR_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    Serial.begin(115200);
    while (!Serial) { ; }

    // Attach and center all servos EXCEPT J2
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i == SKIP_JOINT) continue;  // don't touch J2
        servos[i].attach(SERVO_PINS[i]);
        servos[i].write(90);
    }

    Serial.println();
    Serial.println("*****************************************");
    Serial.println("*  J2 (Shoulder, pin 11) is DISABLED    *");
    Serial.println("*  All other joints centered at 90°     *");
    Serial.println("*****************************************");
    printHelp();
    printAll();
}

void loop() {
    if (!Serial.available()) return;

    char c = Serial.read();
    switch (c) {
        // Joint selection
        case '1': selected = 0; Serial.print("Selected: "); printStatus(); break;
        case '2':
            Serial.println("  ** J2 is disabled. Pick another joint. **");
            break;
        case '3': selected = 2; Serial.print("Selected: "); printStatus(); break;
        case '4': selected = 3; Serial.print("Selected: "); printStatus(); break;
        case '5': selected = 4; Serial.print("Selected: "); printStatus(); break;
        case '6': selected = 5; Serial.print("Selected: "); printStatus(); break;

        // Coarse nudge
        case 'd': moveToAngle(angles[selected] + 5); break;
        case 'a': moveToAngle(angles[selected] - 5); break;

        // Fine nudge
        case 'D': moveToAngle(angles[selected] + 1); break;
        case 'A': moveToAngle(angles[selected] - 1); break;

        // Center
        case 'c': moveToAngle(90); break;

        // Record limits
        case 'L':
            if (selected == SKIP_JOINT) {
                Serial.println("  ** J2 is disabled **");
                break;
            }
            limit_low[selected] = angles[selected];
            Serial.print("  >> LOW limit recorded: ");
            Serial.print(angles[selected]);
            Serial.println("°");
            break;
        case 'U':
            if (selected == SKIP_JOINT) {
                Serial.println("  ** J2 is disabled **");
                break;
            }
            limit_high[selected] = angles[selected];
            Serial.print("  >> HIGH limit recorded: ");
            Serial.print(angles[selected]);
            Serial.println("°");
            break;

        // Display
        case 'r': printResults(); break;
        case 'p': printAll(); break;
        case 'h': printHelp(); break;
    }
}
