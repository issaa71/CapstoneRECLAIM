#include <Arduino.h>

// --- Pin definitions ---
const int PWM_PIN = 2;
const int DIR_PIN = 3;
const int ENC_A  = 6;
const int ENC_B  = 7;

// --- Encoder state ---
volatile long encoderTicks = 0;

void encoderISR_A() {
    if (digitalRead(ENC_A) == digitalRead(ENC_B)) encoderTicks--;
    else encoderTicks++;
}

void encoderISR_B() {
    if (digitalRead(ENC_A) == digitalRead(ENC_B)) encoderTicks++;
    else encoderTicks--;
}

long readTicks() {
    noInterrupts();
    long t = encoderTicks;
    interrupts();
    return t;
}

// --- State ---
int currentPWM = 128;
bool motorRunning = false;
bool motorForward = true;
unsigned long lastPrintTime = 0;

void applyMotor() {
    if (motorRunning) {
        digitalWrite(DIR_PIN, motorForward ? HIGH : LOW);
        analogWrite(PWM_PIN, currentPWM);
    } else {
        analogWrite(PWM_PIN, 0);
    }
}

void printStatus() {
    const char* state = motorRunning ? (motorForward ? "FWD" : "REV") : "STOP";
    Serial.print("[");
    Serial.print(state);
    Serial.print("] PWM=");
    Serial.print(motorRunning ? currentPWM : 0);
    Serial.print("  Ticks=");
    Serial.println(readTicks());
}

void printHelp() {
    Serial.println();
    Serial.println("=== MANUAL MOTOR CONTROL ===");
    Serial.println("  w = forward");
    Serial.println("  s = reverse");
    Serial.println("  x = stop");
    Serial.println("  + = increase speed (+25)");
    Serial.println("  - = decrease speed (-25)");
    Serial.println("  r = reset encoder count");
    Serial.println("  h = show this help");
    Serial.println("============================");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR_B, CHANGE);

    analogWrite(PWM_PIN, 0);
    printHelp();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'w':
                motorRunning = true;
                motorForward = true;
                applyMotor();
                break;
            case 's':
                motorRunning = true;
                motorForward = false;
                applyMotor();
                break;
            case 'x':
                motorRunning = false;
                applyMotor();
                break;
            case '+':
            case '=':
                currentPWM = min(currentPWM + 25, 255);
                applyMotor();
                Serial.print("Speed: ");
                Serial.println(currentPWM);
                break;
            case '-':
                currentPWM = max(currentPWM - 25, 0);
                applyMotor();
                Serial.print("Speed: ");
                Serial.println(currentPWM);
                break;
            case 'r':
                noInterrupts();
                encoderTicks = 0;
                interrupts();
                Serial.println("Encoder reset.");
                break;
            case 'h':
                printHelp();
                break;
        }
    }

    // Print status every 200ms while motor is running
    if (motorRunning && (millis() - lastPrintTime >= 200)) {
        lastPrintTime = millis();
        printStatus();
    }
}
