/*
 * Gateway_Sender.ino
 * Murob SHOWTIME — ENG/MIP 380, Drexel University 2026-01
 *
 * Reads comma-delimited commands from a PC host (Ableton/Max for Live) over
 * USB-Serial and re-broadcasts them to all Receiver Nodes via ESP-NOW.
 *
 * Message format on Serial:
 *   <target_node,actuator_type,parameter_1,parameter_2,trigger_state>
 *
 * ESP-NOW target: broadcast (FF:FF:FF:FF:FF:FF)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ─── Shared payload definition ────────────────────────────────────────────────
struct master_payload_t {
    uint8_t  target_node;   // which receiver to act (1-6)
    uint8_t  actuator_type; // 0=Servo 1=DC Motor 2=Solenoid 3=Stepper
    int32_t  parameter_1;   // primary parameter (angle, speed, position …)
    int16_t  parameter_2;   // secondary parameter (direction, duration …)
    bool     trigger_state; // digital trigger (on/off)
};

// ─── Constants ────────────────────────────────────────────────────────────────
static const uint8_t  BROADCAST_MAC[6]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint32_t SERIAL_BAUD       = 115200;
static const uint8_t  RX_BUFFER_SIZE    = 64;

// ─── Non-blocking serial parser state ────────────────────────────────────────
static char    rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxIndex    = 0;
static bool    inPacket   = false;

// ─── Forward declarations ─────────────────────────────────────────────────────
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status);
static bool parsePayload(const char *buf, master_payload_t &payload);
static void broadcastPayload(const master_payload_t &payload);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);

    // Initialise WiFi in Station mode (required by ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[GW] ESP-NOW init failed — halting.");
        while (true) { delay(1000); }
    }

    esp_now_register_send_cb(onDataSent);

    // Register broadcast peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel  = 0;   // current channel
    peer.encrypt  = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[GW] Failed to add broadcast peer.");
    }

    Serial.println("[GW] Gateway Sender ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // Non-blocking accumulation of incoming serial bytes
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());

        if (c == '<') {
            // Start of new packet — reset buffer
            rxIndex  = 0;
            inPacket = true;
        } else if (c == '>' && inPacket) {
            // End of packet — null-terminate and process
            rxBuffer[rxIndex] = '\0';
            inPacket = false;

            master_payload_t payload = {};
            if (parsePayload(rxBuffer, payload)) {
                broadcastPayload(payload);
            } else {
                Serial.print("[GW] Parse error: ");
                Serial.println(rxBuffer);
            }
        } else if (inPacket) {
            // Accumulate data, guard against buffer overrun
            if (rxIndex < RX_BUFFER_SIZE - 1U) {
                rxBuffer[rxIndex++] = c;
            }
        }
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/*
 * parsePayload — parse a null-terminated string of the form
 *   "target_node,actuator_type,parameter_1,parameter_2,trigger_state"
 * into a master_payload_t.  Returns true on success.
 */
static bool parsePayload(const char *buf, master_payload_t &payload) {
    // Work on a local copy so strtok does not mutate the caller's buffer
    char tmp[RX_BUFFER_SIZE];
    strncpy(tmp, buf, RX_BUFFER_SIZE - 1);
    tmp[RX_BUFFER_SIZE - 1] = '\0';

    char *token = strtok(tmp, ",");
    if (!token) return false;
    payload.target_node = static_cast<uint8_t>(atoi(token));

    token = strtok(nullptr, ",");
    if (!token) return false;
    payload.actuator_type = static_cast<uint8_t>(atoi(token));

    token = strtok(nullptr, ",");
    if (!token) return false;
    payload.parameter_1 = static_cast<int32_t>(atol(token));

    token = strtok(nullptr, ",");
    if (!token) return false;
    payload.parameter_2 = static_cast<int16_t>(atoi(token));

    token = strtok(nullptr, ",");
    if (!token) return false;
    payload.trigger_state = (atoi(token) != 0);

    return true;
}

/*
 * broadcastPayload — cast the struct to a byte array and send via ESP-NOW.
 */
static void broadcastPayload(const master_payload_t &payload) {
    esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&payload),
        sizeof(payload)
    );
    if (result != ESP_OK) {
        Serial.print("[GW] esp_now_send error: ");
        Serial.println(result);
    }
}

/*
 * onDataSent — ESP-NOW send callback (called in ISR context; keep it brief).
 */
static void IRAM_ATTR onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    (void)status;
    // Extend here for delivery diagnostics if needed.
}
