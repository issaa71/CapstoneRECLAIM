#include <Arduino.h>

// --- Pin definitions ---
const int PWM_PIN = 2;   // MD10C PWM (speed)
const int DIR_PIN = 3;   // MD10C DIR (direction)
const int ENC_A  = 6;    // Encoder channel A
const int ENC_B  = 7;    // Encoder channel B

// --- Encoder state ---
volatile long encoderTicks = 0;

void encoderISR_A() {
    if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
        encoderTicks--;
    } else {
        encoderTicks++;
    }
}

void encoderISR_B() {
    if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
}

// --- Motor helpers ---
void motorForward(int pwmVal) {
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, pwmVal);
}

void motorReverse(int pwmVal) {
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, pwmVal);
}

void motorStop() {
    analogWrite(PWM_PIN, 0);
}

long readTicks() {
    noInterrupts();
    long t = encoderTicks;
    interrupts();
    return t;
}

// --- Test phases ---
void runTest() {
    Serial.println();
    Serial.println("=== MOTOR + ENCODER TEST ===");
    Serial.println();

    // Reset encoder
    noInterrupts();
    encoderTicks = 0;
    interrupts();

    // Phase 1: Forward at 50% for 2 seconds
    Serial.println("--- Phase 1: Forward (PWM=128) ---");
    motorForward(128);
    for (int i = 0; i < 10; i++) {
        delay(200);
        Serial.print("[FWD ] PWM=128  Ticks=");
        Serial.println(readTicks());
        if (Serial.available()) { motorStop(); return; }
    }

    // Phase 2: Stop for 1 second
    motorStop();
    Serial.println("--- Phase 2: Stop ---");
    long ticksAtStop = readTicks();
    Serial.print("[STOP] Ticks=");
    Serial.println(ticksAtStop);
    delay(1000);
    Serial.print("[STOP] Ticks after 1s=");
    Serial.print(readTicks());
    if (readTicks() == ticksAtStop) {
        Serial.println("  (OK - stable)");
    } else {
        Serial.println("  (WARN - drifting!)");
    }

    if (Serial.available()) { return; }

    // Phase 3: Reverse at 50% for 2 seconds
    Serial.println("--- Phase 3: Reverse (PWM=128) ---");
    motorReverse(128);
    for (int i = 0; i < 10; i++) {
        delay(200);
        Serial.print("[REV ] PWM=128  Ticks=");
        Serial.println(readTicks());
        if (Serial.available()) { motorStop(); return; }
    }

    // Phase 4: Stop for 1 second
    motorStop();
    Serial.println("--- Phase 4: Stop ---");
    ticksAtStop = readTicks();
    Serial.print("[STOP] Ticks=");
    Serial.println(ticksAtStop);
    delay(1000);
    Serial.print("[STOP] Ticks after 1s=");
    Serial.print(readTicks());
    if (readTicks() == ticksAtStop) {
        Serial.println("  (OK - stable)");
    } else {
        Serial.println("  (WARN - drifting!)");
    }

    if (Serial.available()) { return; }

    // Phase 5: Speed ramp 0 -> 255 over 3 seconds
    Serial.println("--- Phase 5: Speed Ramp (forward) ---");
    noInterrupts();
    encoderTicks = 0;
    interrupts();

    long prevTicks = 0;
    unsigned long prevTime = millis();

    for (int pwm = 0; pwm <= 255; pwm += 15) {
        motorForward(pwm);
        delay(200);

        long curTicks = readTicks();
        unsigned long curTime = millis();

        long dTicks = curTicks - prevTicks;
        float dTimeSec = (curTime - prevTime) / 1000.0;
        // JGB37-520: 7 PPR encoder, 30:1 gearbox = 210 ticks/rev (quadrature = x4 -> 840)
        // We'll show raw ticks/sec and estimated RPM assuming 210 ticks/rev
        float ticksPerSec = dTicks / dTimeSec;
        float rpm = (ticksPerSec / 210.0) * 60.0;

        Serial.print("[RAMP] PWM=");
        Serial.print(pwm);
        Serial.print("\tTicks=");
        Serial.print(curTicks);
        Serial.print("\tTicks/s=");
        Serial.print(ticksPerSec, 1);
        Serial.print("\tRPM=");
        Serial.println(rpm, 1);

        prevTicks = curTicks;
        prevTime = curTime;

        if (Serial.available()) { motorStop(); return; }
    }

    motorStop();

    Serial.println();
    Serial.println("=== TEST COMPLETE ===");
    Serial.println("Send any character to re-run.");
}

// --- Main ---
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR_B, CHANGE);

    motorStop();

    Serial.println("Motor + Encoder Test Ready");
    Serial.println("Send any character to start test...");
}

void loop() {
    if (Serial.available()) {
        while (Serial.available()) Serial.read(); // flush
        runTest();
    }
}
