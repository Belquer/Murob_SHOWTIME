/**
 * @file    Gateway_Sender.ino
 * @brief   ESP-NOW Gateway Sender for the Murob SHOWTIME distributed
 *          cyber-physical network.
 *
 * Reads comma-delimited command strings from a host PC (e.g. Ableton Live /
 * Max for Live) over USB-Serial and re-broadcasts them to all Receiver Nodes
 * via the ESP-NOW protocol using the universal broadcast MAC address so that
 * no individual node MAC registration is required.
 *
 * Expected serial message format (115 200 baud):
 *   <target_node,actuator_type,parameter_1,parameter_2,trigger_state>
 *
 * Example:
 *   <3,0,90,0,0>   -> Send to node 3, Servo, angle 90
 *   <1,2,0,0,1>    -> Send to node 1, Solenoid, trigger ON
 *
 * Dependencies:
 *   - ESP32 Arduino core (includes esp_now.h and WiFi.h)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// Shared payload struct — must be identical in Gateway_Sender and
// Receiver_Node so that the raw byte layout matches on both ends.
// ---------------------------------------------------------------------------
struct master_payload_t {
    uint8_t  target_node;   // ID of the intended Receiver Node (1-6)
    uint8_t  actuator_type; // 0=Servo, 1=DC Motor, 2=Solenoid, 3=Stepper
    int32_t  parameter_1;   // Primary parameter  (angle, speed, position …)
    int16_t  parameter_2;   // Secondary parameter (direction, …)
    bool     trigger_state; // General-purpose boolean trigger flag
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static const uint32_t SERIAL_BAUD      = 115200;
static const uint16_t SERIAL_BUF_SIZE  = 128;

// Universal ESP-NOW broadcast MAC — packets are received by every node in
// range without needing to register individual peer MAC addresses.
static const uint8_t BROADCAST_MAC[6]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
// Serial parser state
// ---------------------------------------------------------------------------
static char    serialBuf[SERIAL_BUF_SIZE];
static uint8_t serialBufIdx  = 0;
static bool    inMessage     = false;  // true while inside '<' … '>' markers

// ---------------------------------------------------------------------------
// ESP-NOW send callback — fired after each transmission attempt.
// ---------------------------------------------------------------------------
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status)
{
    // Uncomment for debugging:
    // Serial.print("[TX] Status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    (void)macAddr;
    (void)status;
}

// ---------------------------------------------------------------------------
// parseAndSend()
// Parses a null-terminated string of comma-separated values into a
// master_payload_t and transmits it via ESP-NOW broadcast.
// Expected format: "target_node,actuator_type,parameter_1,parameter_2,trigger_state"
// ---------------------------------------------------------------------------
static void parseAndSend(const char *msg)
{
    master_payload_t payload = {0, 0, 0, 0, false};

    // strtok works on a mutable copy
    char buf[SERIAL_BUF_SIZE];
    strncpy(buf, msg, SERIAL_BUF_SIZE - 1);
    buf[SERIAL_BUF_SIZE - 1] = '\0';

    char *token = strtok(buf, ",");
    if (token == nullptr) return;
    uint8_t nodeId = (uint8_t)atoi(token);
    if (nodeId < 1 || nodeId > 6) {
        Serial.print("[WARN] target_node out of range (1-6): ");
        Serial.println(nodeId);
        return;
    }
    payload.target_node = nodeId;

    token = strtok(nullptr, ",");
    if (token == nullptr) return;
    uint8_t actType = (uint8_t)atoi(token);
    if (actType > 3) {
        Serial.print("[WARN] actuator_type out of range (0-3): ");
        Serial.println(actType);
        return;
    }
    payload.actuator_type = actType;

    token = strtok(nullptr, ",");
    if (token == nullptr) return;
    payload.parameter_1 = (int32_t)atol(token);

    token = strtok(nullptr, ",");
    if (token == nullptr) return;
    payload.parameter_2 = (int16_t)atoi(token);

    token = strtok(nullptr, ",");
    if (token == nullptr) return;
    payload.trigger_state = (atoi(token) != 0);

    // Transmit the raw struct bytes via ESP-NOW broadcast
    esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&payload),
        sizeof(master_payload_t)
    );

    if (result != ESP_OK) {
        Serial.print("[ERR] esp_now_send failed: ");
        Serial.println(result);
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(SERIAL_BAUD);

    // ESP-NOW requires Wi-Fi to be initialised (Station mode is sufficient;
    // no AP or connection is needed).
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // ensure we are not attempting to join any AP

    // Initialise ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[FATAL] ESP-NOW initialisation failed — halting.");
        while (true) { delay(1000); }
    }

    // Register send-status callback
    esp_now_register_send_cb(onDataSent);

    // Register the broadcast address as a peer.
    // Using broadcast does not require ACKs, so peer_addr is the only field
    // that needs to be set (channel 0 = current channel, encrypt = false).
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
    peerInfo.channel  = 0;
    peerInfo.encrypt  = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[FATAL] Failed to add broadcast peer — halting.");
        while (true) { delay(1000); }
    }

    Serial.println("[INFO] Gateway Sender ready.");
    Serial.println("[INFO] Waiting for messages in format: <node,type,p1,p2,trigger>");
}

// ---------------------------------------------------------------------------
// loop()
// Non-blocking serial parser.  Characters are processed one at a time so
// that the MCU is never stalled waiting for a complete message.
// ---------------------------------------------------------------------------
void loop()
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '<') {
            // Start-of-message marker — reset buffer and begin capture
            serialBufIdx = 0;
            inMessage    = true;
        } else if (c == '>') {
            // End-of-message marker — null-terminate and process
            if (inMessage && serialBufIdx > 0) {
                serialBuf[serialBufIdx] = '\0';
                parseAndSend(serialBuf);
            }
            inMessage    = false;
            serialBufIdx = 0;
        } else if (inMessage) {
            // Accumulate characters; guard against buffer overflow
            if (serialBufIdx < SERIAL_BUF_SIZE - 1) {
                serialBuf[serialBufIdx++] = c;
            } else {
                // Buffer overflow — discard this partial message
                inMessage    = false;
                serialBufIdx = 0;
                Serial.println("[WARN] Serial buffer overflow — message discarded.");
            }
        }
    }
}
