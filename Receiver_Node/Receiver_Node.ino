/**
 * @file    Receiver_Node.ino
 * @brief   ESP-NOW Receiver Node template for the Murob SHOWTIME distributed
 *          cyber-physical network.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  HOW TO CONFIGURE THIS FILE FOR EACH PHYSICAL NODE
 *  ──────────────────────────────────────────────────
 *  Step 1 — Set MY_NODE_ID (line ~40) to the node number you are flashing
 *           (1 through 6).
 *
 *  Step 2 — In the "Per-Node Pin Definitions" block below, find the
 *           corresponding #elif branch and edit the pin numbers to match
 *           your physical wiring for that node.
 *
 *  That is the only two-step change required per node.  The compiler will
 *  automatically pick the correct set of pin constants for the chosen ID.
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The node listens for ESP-NOW broadcast packets from the Gateway Sender,
 * checks whether the packet's `target_node` field matches its own ID, and
 * dispatches commands to the appropriate actuator subroutine.
 *
 * Actuator dispatch table:
 *   case 0  — Servo motor           (ESP32Servo library)
 *   case 1  — DC motor via H-Bridge (PWM + direction GPIO)
 *   case 2  — Solenoid              (millis()-based, no delay())
 *   case 3  — Stepper motor         (FastAccelStepper library)
 *
 * Dependencies:
 *   - ESP32 Arduino core (esp_now.h, WiFi.h)
 *   - ESP32Servo          https://github.com/madhephaestus/ESP32Servo
 *   - FastAccelStepper    https://github.com/gin66/FastAccelStepper
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>

// ═══════════════════════════════════════════════════════════════════════════
//  STEP 1 — Set this to the node number you are currently flashing (1-6)
// ═══════════════════════════════════════════════════════════════════════════
static const uint8_t MY_NODE_ID = 1;

// ═══════════════════════════════════════════════════════════════════════════
//  STEP 2 — Per-Node Pin Definitions
//
//  Each node has its own #elif branch.  Find the branch whose number matches
//  MY_NODE_ID and replace the pin numbers with the GPIOs you have wired on
//  that particular ESP32 board.  Every node's branch is shown here so the
//  whole project's wiring plan is visible in one place.
//
//  ⚠  The values below are PLACEHOLDER defaults (all nodes start with the
//     same numbers).  You MUST update each node's branch to reflect its
//     actual physical wiring before flashing.
//
//  PIN REFERENCE (all nodes share the same actuator roles):
//    PIN_SERVO      — Signal wire of the servo (any GPIO)
//    PIN_MOTOR_ENA  — PWM-capable GPIO → H-Bridge ENA (speed)
//    PIN_MOTOR_IN1  — GPIO → H-Bridge IN1 (direction bit 1)
//    PIN_MOTOR_IN2  — GPIO → H-Bridge IN2 (direction bit 2)
//    PIN_SOLENOID   — GPIO → solenoid driver gate / relay
//    PIN_STEP       — GPIO → stepper driver STEP pulse
//    PIN_DIR        — GPIO → stepper driver DIR
//
//  ESP32 GPIOs safe for general output: 4,5,13,14,16,17,18,19,21,22,23,25,26,27,32,33
//  PWM-capable (LEDC): all of the above — prefer 25/26/32/33 for motor ENA.
//  Avoid: 0,2,6-11,12,15,34-39 for output (strapping / input-only pins).
// ═══════════════════════════════════════════════════════════════════════════

#if   MY_NODE_ID == 1
// ── Node 1 — update these values to match Node 1's physical wiring ───────
    #define PIN_SERVO       13
    #define PIN_MOTOR_ENA   25   // must be PWM-capable
    #define PIN_MOTOR_IN1   26
    #define PIN_MOTOR_IN2   27
    #define PIN_SOLENOID    14
    #define PIN_STEP        18
    #define PIN_DIR         19

#elif MY_NODE_ID == 2
// ── Node 2 — update these values to match Node 2's physical wiring ───────
    #define PIN_SERVO       13
    #define PIN_MOTOR_ENA   25   // must be PWM-capable
    #define PIN_MOTOR_IN1   26
    #define PIN_MOTOR_IN2   27
    #define PIN_SOLENOID    14
    #define PIN_STEP        18
    #define PIN_DIR         19

#elif MY_NODE_ID == 3
// ── Node 3 — update these values to match Node 3's physical wiring ───────
    #define PIN_SERVO       13
    #define PIN_MOTOR_ENA   25   // must be PWM-capable
    #define PIN_MOTOR_IN1   26
    #define PIN_MOTOR_IN2   27
    #define PIN_SOLENOID    14
    #define PIN_STEP        18
    #define PIN_DIR         19

#elif MY_NODE_ID == 4
// ── Node 4 — update these values to match Node 4's physical wiring ───────
    #define PIN_SERVO       13
    #define PIN_MOTOR_ENA   25   // must be PWM-capable
    #define PIN_MOTOR_IN1   26
    #define PIN_MOTOR_IN2   27
    #define PIN_SOLENOID    14
    #define PIN_STEP        18
    #define PIN_DIR         19

#elif MY_NODE_ID == 5
// ── Node 5 — update these values to match Node 5's physical wiring ───────
    #define PIN_SERVO       13
    #define PIN_MOTOR_ENA   25   // must be PWM-capable
    #define PIN_MOTOR_IN1   26
    #define PIN_MOTOR_IN2   27
    #define PIN_SOLENOID    14
    #define PIN_STEP        18
    #define PIN_DIR         19

#elif MY_NODE_ID == 6
// ── Node 6 — update these values to match Node 6's physical wiring ───────
    #define PIN_SERVO       13
    #define PIN_MOTOR_ENA   25   // must be PWM-capable
    #define PIN_MOTOR_IN1   26
    #define PIN_MOTOR_IN2   27
    #define PIN_SOLENOID    14
    #define PIN_STEP        18
    #define PIN_DIR         19

#else
    #error "MY_NODE_ID must be a value between 1 and 6. Please set it at the top of this file."
#endif

// ---------------------------------------------------------------------------
// Solenoid pulse width and DC motor speed scale — shared across all nodes
// ---------------------------------------------------------------------------

// How long the solenoid stays energised after a trigger (milliseconds)
static const uint32_t SOLENOID_PULSE_MS = 50;

// DC Motor speed — parameter_1 is mapped from [0, MAX_SPEED_VALUE] to PWM
// [0, 255].  The upper bound mirrors the positive range of int16_t (32767) to
// allow a future upgrade where speed arrives as a signed 16-bit value.
static const int32_t MAX_SPEED_VALUE = 32767;

// ---------------------------------------------------------------------------
// Shared payload struct — must be byte-identical to the Gateway Sender.
// ---------------------------------------------------------------------------
struct master_payload_t {
    uint8_t  target_node;   // ID of the intended Receiver Node (1-6)
    uint8_t  actuator_type; // 0=Servo, 1=DC Motor, 2=Solenoid, 3=Stepper
    int32_t  parameter_1;   // Primary parameter  (angle, speed, position …)
    int16_t  parameter_2;   // Secondary parameter (direction, …)
    bool     trigger_state; // General-purpose boolean trigger flag
};

// ---------------------------------------------------------------------------
// Actuator objects
// ---------------------------------------------------------------------------
static Servo servo;

static FastAccelStepperEngine stepperEngine;
static FastAccelStepper       *stepper = nullptr;

// ---------------------------------------------------------------------------
// Inter-context communication between the ESP-NOW callback (runs in a
// different RTOS task context) and the main loop.
// ---------------------------------------------------------------------------
static volatile bool     newDataReceived = false;
static master_payload_t  incomingPayload;

// ---------------------------------------------------------------------------
// Solenoid state machine variables
// ---------------------------------------------------------------------------
static bool     solenoidActive     = false;
static uint32_t solenoidStartMs    = 0;

// ---------------------------------------------------------------------------
// actuatorServo()
// Map parameter_1 (arbitrary int32 range) to a valid servo angle [0, 180].
// ---------------------------------------------------------------------------
static void actuatorServo(const master_payload_t &payload)
{
    // Clamp / map parameter_1 to [0, 180] degrees
    int32_t angle = payload.parameter_1;
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    servo.write((int)angle);
}

// ---------------------------------------------------------------------------
// actuatorDCMotor()
// parameter_1 : desired speed as a raw value [0, 32767] mapped to PWM [0, 255]
// parameter_2 : direction — positive = forward (IN1 HIGH, IN2 LOW)
//                           zero/neg  = reverse (IN1 LOW,  IN2 HIGH)
// ---------------------------------------------------------------------------
static void actuatorDCMotor(const master_payload_t &payload)
{
    // Map parameter_1 from [0, 32767] to [0, 255]
    int32_t rawSpeed = payload.parameter_1;
    if (rawSpeed < 0)              rawSpeed = 0;
    if (rawSpeed > MAX_SPEED_VALUE) rawSpeed = MAX_SPEED_VALUE;
    uint8_t pwmSpeed = (uint8_t)map(rawSpeed, 0, MAX_SPEED_VALUE, 0, 255);

    analogWrite(PIN_MOTOR_ENA, pwmSpeed);

    if (payload.parameter_2 > 0) {
        // Forward
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
    } else {
        // Reverse
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
    }
}

// ---------------------------------------------------------------------------
// actuatorSolenoidCommand()
// Called once when a new solenoid command arrives.
// When trigger_state is true, energise the solenoid and start the timer;
// when false, de-energise immediately.
// The companion actuatorSolenoidUpdate() must be called every loop iteration.
// ---------------------------------------------------------------------------
static void actuatorSolenoidCommand(const master_payload_t &payload)
{
    if (payload.trigger_state) {
        digitalWrite(PIN_SOLENOID, HIGH);
        solenoidActive  = true;
        solenoidStartMs = millis();
    } else {
        // Explicit OFF command
        digitalWrite(PIN_SOLENOID, LOW);
        solenoidActive = false;
    }
}

// ---------------------------------------------------------------------------
// actuatorSolenoidUpdate()
// Non-blocking millis()-based state machine — call every loop iteration.
// Turns the solenoid off once SOLENOID_PULSE_MS has elapsed.
// ---------------------------------------------------------------------------
static void actuatorSolenoidUpdate()
{
    // millis() returns uint32_t; unsigned subtraction wraps correctly at
    // roll-over (~49.7 days), so this comparison is always safe.
    if (solenoidActive && (millis() - solenoidStartMs >= SOLENOID_PULSE_MS)) {
        digitalWrite(PIN_SOLENOID, LOW);
        solenoidActive = false;
    }
}

// ---------------------------------------------------------------------------
// actuatorStepper()
// Move stepper to an absolute position specified by parameter_1.
// FastAccelStepper generates hardware-timed step pulses in the background so
// the call returns immediately.
// ---------------------------------------------------------------------------
static void actuatorStepper(const master_payload_t &payload)
{
    if (stepper == nullptr) return;

    int32_t targetPosition = payload.parameter_1;
    stepper->moveTo(targetPosition);
}

// ---------------------------------------------------------------------------
// ESP-NOW receive callback
// IMPORTANT: This callback executes in an RTOS task context (not the main
// Arduino loop), so it must be strictly non-blocking.  Only a memcpy and a
// flag write are performed here; all processing happens in loop().
// ---------------------------------------------------------------------------
void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int dataLen)
{
    (void)macAddr;

    if (dataLen == sizeof(master_payload_t)) {
        memcpy(&incomingPayload, data, sizeof(master_payload_t));
        newDataReceived = true;
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    Serial.print("[INFO] Receiver Node ");
    Serial.print(MY_NODE_ID);
    Serial.println(" starting up…");

    // --- GPIO initialisation ---
    pinMode(PIN_SOLENOID,  OUTPUT);
    digitalWrite(PIN_SOLENOID, LOW);

    pinMode(PIN_MOTOR_ENA, OUTPUT);
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    analogWrite(PIN_MOTOR_ENA, 0);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);

    // --- Servo ---
    servo.attach(PIN_SERVO);
    servo.write(90); // neutral position on startup

    // --- Stepper ---
    stepperEngine.init();
    stepper = stepperEngine.stepperConnectToPin(PIN_STEP);
    if (stepper != nullptr) {
        stepper->setDirectionPin(PIN_DIR);
        stepper->setAcceleration(5000);  // steps/s²
        stepper->setSpeedInHz(2000);     // steps/s
    }

    // --- Wi-Fi (Station mode required by ESP-NOW) ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // --- ESP-NOW ---
    if (esp_now_init() != ESP_OK) {
        Serial.println("[FATAL] ESP-NOW initialisation failed — halting.");
        while (true) { delay(1000); }
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("[INFO] ESP-NOW ready — listening for commands.");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop()
{
    // Always service the solenoid state machine regardless of new data
    actuatorSolenoidUpdate();

    // Check for a newly received ESP-NOW packet
    if (newDataReceived) {
        newDataReceived = false; // clear flag before processing

        // Only act on packets addressed to this node
        if (incomingPayload.target_node == MY_NODE_ID) {

            switch (incomingPayload.actuator_type) {

                case 0: // ---- Servo ------------------------------------------
                    actuatorServo(incomingPayload);
                    break;

                case 1: // ---- DC Motor ----------------------------------------
                    actuatorDCMotor(incomingPayload);
                    break;

                case 2: // ---- Solenoid ----------------------------------------
                    actuatorSolenoidCommand(incomingPayload);
                    break;

                case 3: // ---- Stepper Motor ------------------------------------
                    actuatorStepper(incomingPayload);
                    break;

                default:
                    Serial.print("[WARN] Unknown actuator_type: ");
                    Serial.println(incomingPayload.actuator_type);
                    break;
            }
        }
        // Packets addressed to other nodes are silently ignored.
    }
}
