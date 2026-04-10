#include <Arduino.h>

// --- Left motor pins ---
const int L_PWM = 2;
const int L_DIR = 3;
const int L_ENC_A = 6;
const int L_ENC_B = 7;

// --- Right motor pins ---
const int R_PWM = 22;
const int R_DIR = 20;
const int R_ENC_A = 19;
const int R_ENC_B = 18;

// --- Encoder state ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- Encoder ISRs (Left) ---
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

// --- Encoder ISRs (Right) ---
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

// --- Motor helpers ---
void leftForward(int pwm)  { digitalWrite(L_DIR, LOW);  analogWrite(L_PWM, pwm); }
void leftReverse(int pwm)  { digitalWrite(L_DIR, HIGH); analogWrite(L_PWM, pwm); }
void leftStop()             { analogWrite(L_PWM, 0); }

void rightForward(int pwm) { digitalWrite(R_DIR, HIGH); analogWrite(R_PWM, pwm); }
void rightReverse(int pwm) { digitalWrite(R_DIR, LOW);  analogWrite(R_PWM, pwm); }
void rightStop()            { analogWrite(R_PWM, 0); }

void allStop() { leftStop(); rightStop(); }

long readLeft() {
    noInterrupts();
    long t = leftTicks;
    interrupts();
    return t;
}

long readRight() {
    noInterrupts();
    long t = rightTicks;
    interrupts();
    return t;
}

void resetTicks() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
}

// --- Test: spin both motors forward, print ticks ---
void testBoth() {
    Serial.println("\n=== BOTH MOTORS: Forward 100% for 6s ===");
    resetTicks();

    leftForward(255);
    rightForward(255);

    for (int i = 0; i < 30; i++) {
        delay(200);
        Serial.print("  L=");
        Serial.print(readLeft());
        Serial.print("\tR=");
        Serial.println(readRight());
    }

    allStop();
    Serial.print("  STOPPED. L=");
    Serial.print(readLeft());
    Serial.print("  R=");
    Serial.println(readRight());

    delay(500);

    Serial.println("\n=== BOTH MOTORS: Reverse 100% for 6s ===");
    resetTicks();

    leftReverse(255);
    rightReverse(255);

    for (int i = 0; i < 30; i++) {
        delay(200);
        Serial.print("  L=");
        Serial.print(readLeft());
        Serial.print("\tR=");
        Serial.println(readRight());
    }

    allStop();
    Serial.print("  STOPPED. L=");
    Serial.print(readLeft());
    Serial.print("  R=");
    Serial.println(readRight());

    Serial.println("\n=== DONE ===");
    Serial.println("Send any key to re-run");
}

// --- Main ---
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    pinMode(L_PWM, OUTPUT);
    pinMode(L_DIR, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT);

    pinMode(L_ENC_A, INPUT_PULLUP);
    pinMode(L_ENC_B, INPUT_PULLUP);
    pinMode(R_ENC_A, INPUT_PULLUP);
    pinMode(R_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftISR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightISR_B, CHANGE);

    allStop();

    Serial.println("Dual Motor + Encoder Test");
    Serial.println("Pins: L(PWM=2 DIR=3 ENC=6,7) R(PWM=22 DIR=20 ENC=19,18)");
    Serial.println("Send any key to start...");
}

void loop() {
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        testBoth();
    }
}
