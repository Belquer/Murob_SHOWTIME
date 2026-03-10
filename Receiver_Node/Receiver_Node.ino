/*
 * Receiver_Node.ino
 * Murob SHOWTIME — ENG/MIP 380, Drexel University 2026-01
 *
 * Listens for ESP-NOW broadcast packets from the Gateway Sender and, if the
 * `target_node` field matches MY_NODE_ID, dispatches the command to the
 * appropriate actuator subroutine.
 *
 * ── Pin assignment (example — adjust per physical build) ─────────────────────
 *   SERVO_PIN      = 13
 *   MOTOR_ENA_PIN  = 25   (PWM)
 *   MOTOR_IN1_PIN  = 26
 *   MOTOR_IN2_PIN  = 27
 *   SOLENOID_PIN   = 32
 *   STEPPER_STEP   = 18
 *   STEPPER_DIR    = 19
 *
 * ── Libraries required ────────────────────────────────────────────────────────
 *   ESP32Servo      — https://github.com/madhephaestus/ESP32Servo
 *   FastAccelStepper — https://github.com/gin66/FastAccelStepper
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>

// ─── Node identity ─────────────────────────────────────────────────────────
// Change this constant when flashing each of the 6 nodes (1-6).
static const uint8_t MY_NODE_ID = 1;

// ─── Shared payload definition (must match Gateway_Sender) ────────────────
struct master_payload_t {
    uint8_t  target_node;
    uint8_t  actuator_type;
    int32_t  parameter_1;
    int16_t  parameter_2;
    bool     trigger_state;
};

// ─── Pin definitions ──────────────────────────────────────────────────────
static const int SERVO_PIN      = 13;
static const int MOTOR_ENA_PIN  = 25;
static const int MOTOR_IN1_PIN  = 26;
static const int MOTOR_IN2_PIN  = 27;
static const int SOLENOID_PIN   = 32;
static const int STEPPER_STEP   = 18;
static const int STEPPER_DIR    = 19;

// ─── Actuator objects ─────────────────────────────────────────────────────
static Servo servo;
static FastAccelStepperEngine stepperEngine;
static FastAccelStepper *stepper = nullptr;

// ─── Solenoid state-machine ───────────────────────────────────────────────
static const uint32_t SOLENOID_PULSE_MS = 50; // pulse width in milliseconds
static bool     solenoidActive    = false;
static uint32_t solenoidOnTime    = 0;

// ─── ESP-NOW receive flag (written in callback, read in loop) ─────────────
static volatile bool        newDataReceived = false;
static master_payload_t     incomingPayload;
static master_payload_t     pendingPayload;

// ─── Forward declarations ────────────────────────────────────────────────
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len);
static void handleServo(int32_t parameter_1);
static void handleDCMotor(int32_t parameter_1, int16_t parameter_2);
static void handleSolenoid(bool trigger_state);
static void handleStepper(int32_t parameter_1);
static void updateSolenoid();

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // --- GPIO setup ---
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(SOLENOID_PIN,  OUTPUT);
    digitalWrite(MOTOR_ENA_PIN, LOW);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(SOLENOID_PIN,  LOW);

    // --- Servo ---
    servo.attach(SERVO_PIN);
    servo.write(90); // neutral position

    // --- Stepper ---
    stepperEngine.init();
    stepper = stepperEngine.stepperConnectToPin(STEPPER_STEP);
    if (stepper) {
        stepper->setDirectionPin(STEPPER_DIR);
        stepper->setSpeedInHz(2000);       // steps per second
        stepper->setAcceleration(1000);    // steps per second²
    }

    // --- WiFi + ESP-NOW ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[RX] ESP-NOW init failed — halting.");
        while (true) { delay(1000); }
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.print("[RX] Node ");
    Serial.print(MY_NODE_ID);
    Serial.println(" ready.");
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    // Update any time-driven actuator state machines
    updateSolenoid();

    // Check if a new ESP-NOW packet arrived
    if (newDataReceived) {
        // Copy atomically-enough for single-writer / single-reader on ESP32
        noInterrupts();
        pendingPayload     = incomingPayload;
        newDataReceived    = false;
        interrupts();

        // Route only packets addressed to this node
        if (pendingPayload.target_node == MY_NODE_ID) {
            switch (pendingPayload.actuator_type) {
                case 0: handleServo(pendingPayload.parameter_1);   break;
                case 1: handleDCMotor(pendingPayload.parameter_1,
                                      pendingPayload.parameter_2); break;
                case 2: handleSolenoid(pendingPayload.trigger_state); break;
                case 3: handleStepper(pendingPayload.parameter_1); break;
                default:
                    Serial.print("[RX] Unknown actuator_type: ");
                    Serial.println(pendingPayload.actuator_type);
                    break;
            }
        }
    }
}

// ─── ESP-NOW receive callback ─────────────────────────────────────────────
// Runs in ISR context — MUST be non-blocking; only copy data + set flag.
static void IRAM_ATTR onDataRecv(const uint8_t *mac,
                                  const uint8_t *data, int len) {
    (void)mac;
    if (len == static_cast<int>(sizeof(master_payload_t))) {
        memcpy(&incomingPayload, data, sizeof(master_payload_t));
        newDataReceived = true;
    }
}

// ─── Actuator subroutines ─────────────────────────────────────────────────

/*
 * handleServo — map parameter_1 (0-180 or raw MIDI 0-127) to a servo angle.
 */
static void handleServo(int32_t parameter_1) {
    int angle = static_cast<int>(
        map(parameter_1, 0, 127, 0, 180)
    );
    angle = constrain(angle, 0, 180);
    servo.write(angle);
    Serial.print("[RX] Servo → ");
    Serial.println(angle);
}

/*
 * handleDCMotor — parameter_1 (0-127) → PWM speed on ENA;
 *                 parameter_2 sign / value → direction (IN1/IN2).
 */
static void handleDCMotor(int32_t parameter_1, int16_t parameter_2) {
    uint8_t speed = static_cast<uint8_t>(
        map(constrain(parameter_1, 0, 127), 0, 127, 0, 255)
    );
    analogWrite(MOTOR_ENA_PIN, speed);

    if (parameter_2 >= 0) {
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, LOW);
    } else {
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
    }
    Serial.print("[RX] DC Motor speed=");
    Serial.print(speed);
    Serial.print(" dir=");
    Serial.println(parameter_2 >= 0 ? "FWD" : "REV");
}

/*
 * handleSolenoid — trigger_state HIGH fires a timed pulse; the state machine
 *                  in updateSolenoid() turns the pin LOW after SOLENOID_PULSE_MS
 *                  without ever calling delay().
 */
static void handleSolenoid(bool trigger_state) {
    if (trigger_state && !solenoidActive) {
        digitalWrite(SOLENOID_PIN, HIGH);
        solenoidOnTime  = millis();
        solenoidActive  = true;
        Serial.println("[RX] Solenoid ON");
    }
}

/*
 * updateSolenoid — called every loop() iteration to deactivate solenoid after
 *                  the pulse duration has elapsed.
 */
static void updateSolenoid() {
    if (solenoidActive &&
        // Unsigned subtraction wraps correctly through millis() 49-day rollover
        (millis() - solenoidOnTime >= SOLENOID_PULSE_MS)) {
        digitalWrite(SOLENOID_PIN, LOW);
        solenoidActive = false;
        Serial.println("[RX] Solenoid OFF");
    }
}

/*
 * handleStepper — move to the absolute position given by parameter_1 (steps).
 *                 FastAccelStepper handles acceleration in hardware-timed ISRs,
 *                 so the call is fully non-blocking.
 */
static void handleStepper(int32_t parameter_1) {
    if (stepper) {
        stepper->moveTo(static_cast<long>(parameter_1));
        Serial.print("[RX] Stepper → ");
        Serial.println(parameter_1);
    }
}
