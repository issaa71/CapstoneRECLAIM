#include <Arduino.h>
#include <Servo.h>

// All 6 servo pins
static const int PINS[6] = {10, 11, 12, 13, 14, 15};

// Home positions in µs (verified March 21)
// J1=1229, J2=2153, J3=2030, J4=1400, J5=2000(NEW DS3218), J6=1500
static int HOME_US[6] = {1229, 2153, 2030, 1400, 2000, 1500};

static const char* NAMES[6] = {"J1_Base", "J2_Shoulder", "J3_Elbow", "J4_WristPitch", "J5_WristRotate", "J6_Gripper"};

Servo servos[6];
bool attached[6] = {false, false, false, false, false, false};
int current_us[6] = {1229, 2153, 2030, 1400, 2000, 1500};

String inputBuffer = "";

void printStatus() {
    Serial.println("--- STATUS ---");
    for (int i = 0; i < 6; i++) {
        Serial.print("  J");
        Serial.print(i + 1);
        Serial.print(" (");
        Serial.print(NAMES[i]);
        Serial.print("): ");
        Serial.print(current_us[i]);
        Serial.print(" us ");
        Serial.println(attached[i] ? "[ATTACHED]" : "[detached]");
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    // Drive motor pins LOW (safety)
    pinMode(2, OUTPUT); digitalWrite(2, LOW);
    pinMode(3, OUTPUT); digitalWrite(3, LOW);
    pinMode(22, OUTPUT); digitalWrite(22, LOW);
    pinMode(20, OUTPUT); digitalWrite(20, LOW);

    // Drive ALL servo pins LOW before attaching (prevents spaz)
    for (int i = 0; i < 6; i++) {
        pinMode(PINS[i], OUTPUT);
        digitalWrite(PINS[i], LOW);
    }

    Serial.println("=== INDIVIDUAL SERVO TEST ===");
    Serial.println("All servos DETACHED. Attach one at a time.");
    Serial.println("");
    Serial.println("Commands:");
    Serial.println("  ATTACH <1-6>       — attach servo at HOME position");
    Serial.println("  DETACH <1-6>       — release servo");
    Serial.println("  SET <1-6> <us>     — set servo to µs (500-2500)");
    Serial.println("  NUDGE <1-6> <us>   — nudge by ± µs");
    Serial.println("  HOME <1-6>         — move to home position");
    Serial.println("  HOMEALL            — attach all at home (200ms stagger)");
    Serial.println("  STATUS             — show all servos");
    Serial.println("  SETHOME <1-6> <us> — update home µs value");
    Serial.println("");
    printStatus();
}

void handleCommand(String cmd) {
    cmd.trim();
    String upper = cmd;
    upper.toUpperCase();

    if (upper == "STATUS") {
        printStatus();
        return;
    }

    if (upper == "HOMEALL") {
        Serial.println("Attaching all servos at home positions (200ms stagger)...");
        for (int i = 0; i < 6; i++) {
            current_us[i] = HOME_US[i];
            if (!attached[i]) {
                servos[i].attach(PINS[i]);
                attached[i] = true;
            }
            servos[i].writeMicroseconds(current_us[i]);
            Serial.print("  J");
            Serial.print(i + 1);
            Serial.print(" → ");
            Serial.print(current_us[i]);
            Serial.println(" us");
            delay(200);
        }
        Serial.println("All attached.");
        return;
    }

    // Parse commands with joint number
    if (upper.startsWith("ATTACH ")) {
        int j = cmd.substring(7).toInt();
        if (j < 1 || j > 6) { Serial.println("ERR: joint 1-6"); return; }
        int idx = j - 1;
        current_us[idx] = HOME_US[idx];
        if (!attached[idx]) {
            servos[idx].attach(PINS[idx]);
            attached[idx] = true;
        }
        servos[idx].writeMicroseconds(current_us[idx]);
        Serial.print("J");
        Serial.print(j);
        Serial.print(" ATTACHED at ");
        Serial.print(current_us[idx]);
        Serial.println(" us (home)");
        return;
    }

    if (upper.startsWith("DETACH ")) {
        int j = cmd.substring(7).toInt();
        if (j < 1 || j > 6) { Serial.println("ERR: joint 1-6"); return; }
        int idx = j - 1;
        servos[idx].detach();
        attached[idx] = false;
        Serial.print("J");
        Serial.print(j);
        Serial.println(" DETACHED");
        return;
    }

    if (upper.startsWith("SET ")) {
        int j = 0, us = 0;
        if (sscanf(cmd.c_str() + 4, "%d %d", &j, &us) == 2) {
            if (j < 1 || j > 6) { Serial.println("ERR: joint 1-6"); return; }
            if (us < 500 || us > 2500) { Serial.println("ERR: us 500-2500"); return; }
            int idx = j - 1;
            current_us[idx] = us;
            if (!attached[idx]) {
                servos[idx].attach(PINS[idx]);
                attached[idx] = true;
            }
            servos[idx].writeMicroseconds(current_us[idx]);
            Serial.print("J");
            Serial.print(j);
            Serial.print(" = ");
            Serial.print(us);
            Serial.println(" us");
        } else {
            Serial.println("ERR: SET <1-6> <us>");
        }
        return;
    }

    if (upper.startsWith("NUDGE ")) {
        int j = 0, delta = 0;
        if (sscanf(cmd.c_str() + 6, "%d %d", &j, &delta) == 2) {
            if (j < 1 || j > 6) { Serial.println("ERR: joint 1-6"); return; }
            int idx = j - 1;
            current_us[idx] += delta;
            if (current_us[idx] < 500) current_us[idx] = 500;
            if (current_us[idx] > 2500) current_us[idx] = 2500;
            if (!attached[idx]) {
                servos[idx].attach(PINS[idx]);
                attached[idx] = true;
            }
            servos[idx].writeMicroseconds(current_us[idx]);
            Serial.print("J");
            Serial.print(j);
            Serial.print(" = ");
            Serial.print(current_us[idx]);
            Serial.println(" us");
        } else {
            Serial.println("ERR: NUDGE <1-6> <delta>");
        }
        return;
    }

    if (upper.startsWith("HOME ")) {
        int j = cmd.substring(5).toInt();
        if (j < 1 || j > 6) { Serial.println("ERR: joint 1-6"); return; }
        int idx = j - 1;
        current_us[idx] = HOME_US[idx];
        if (!attached[idx]) {
            servos[idx].attach(PINS[idx]);
            attached[idx] = true;
        }
        servos[idx].writeMicroseconds(current_us[idx]);
        Serial.print("J");
        Serial.print(j);
        Serial.print(" → HOME ");
        Serial.print(current_us[idx]);
        Serial.println(" us");
        return;
    }

    if (upper.startsWith("SETHOME ")) {
        int j = 0, us = 0;
        if (sscanf(cmd.c_str() + 8, "%d %d", &j, &us) == 2) {
            if (j < 1 || j > 6) { Serial.println("ERR: joint 1-6"); return; }
            if (us < 500 || us > 2500) { Serial.println("ERR: us 500-2500"); return; }
            int idx = j - 1;
            HOME_US[idx] = us;
            Serial.print("J");
            Serial.print(j);
            Serial.print(" home updated to ");
            Serial.print(us);
            Serial.println(" us");
        } else {
            Serial.println("ERR: SETHOME <1-6> <us>");
        }
        return;
    }

    Serial.println("Unknown command. Use ATTACH, DETACH, SET, NUDGE, HOME, HOMEALL, STATUS, SETHOME");
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        Serial.print(c);  // echo

        if (c == '\n' || c == '\r') {
            Serial.println();
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer);
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}
