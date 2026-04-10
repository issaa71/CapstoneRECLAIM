#include <Arduino.h>
#include <Servo.h>

// ONLY controls J5 (wrist rotate) on pin 14. No other servos touched.
Servo j5;
const int J5_PIN = 14;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    // Drive ALL motor pins LOW (safety — prevent wheel spin)
    pinMode(2, OUTPUT); digitalWrite(2, LOW);   // Left PWM
    pinMode(3, OUTPUT); digitalWrite(3, LOW);   // Left DIR
    pinMode(22, OUTPUT); digitalWrite(22, LOW); // Right PWM
    pinMode(20, OUTPUT); digitalWrite(20, LOW); // Right DIR

    Serial.println("=== J5 TEST ONLY ===");
    Serial.println("Commands:");
    Serial.println("  SET <us>    — set J5 to microseconds (500-2500)");
    Serial.println("  NUDGE <us>  — nudge J5 by ± microseconds");
    Serial.println("  DETACH      — release J5");
    Serial.println("  PRINT       — show current us");
    Serial.println("");
    Serial.println("J5 is DETACHED. Type SET 1500 to start at center.");
}

int current_us = 1500;
bool attached = false;
String inputBuffer = "";

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        Serial.print(c); // echo

        if (c == '\n' || c == '\r') {
            Serial.println();
            inputBuffer.trim();

            if (inputBuffer.length() > 0) {
                String upper = inputBuffer;
                upper.toUpperCase();

                if (upper.startsWith("SET ")) {
                    int us = inputBuffer.substring(4).toInt();
                    if (us >= 500 && us <= 2500) {
                        current_us = us;
                        if (!attached) {
                            j5.attach(J5_PIN);
                            attached = true;
                        }
                        j5.writeMicroseconds(current_us);
                        Serial.print("J5 = ");
                        Serial.print(current_us);
                        Serial.println(" us");
                    } else {
                        Serial.println("ERR: range 500-2500");
                    }
                }
                else if (upper.startsWith("NUDGE ")) {
                    int delta = inputBuffer.substring(6).toInt();
                    current_us += delta;
                    current_us = constrain(current_us, 500, 2500);
                    if (!attached) {
                        j5.attach(J5_PIN);
                        attached = true;
                    }
                    j5.writeMicroseconds(current_us);
                    Serial.print("J5 = ");
                    Serial.print(current_us);
                    Serial.println(" us");
                }
                else if (upper == "DETACH") {
                    j5.detach();
                    attached = false;
                    Serial.println("J5 detached");
                }
                else if (upper == "PRINT") {
                    Serial.print("J5 = ");
                    Serial.print(current_us);
                    Serial.print(" us (");
                    Serial.print(attached ? "attached" : "detached");
                    Serial.println(")");
                }
                else {
                    Serial.println("Unknown command. Use SET, NUDGE, DETACH, PRINT");
                }
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}
