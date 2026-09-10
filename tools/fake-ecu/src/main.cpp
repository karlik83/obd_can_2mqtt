/*
 * fake_ecu / main.cpp
 *
 * Simulated vehicle ECU for testing ../../src/obd_can.cpp WITHOUT a car.
 *
 * Runs on a second, cheap ESP32 devkit + CAN transceiver. Answers standard
 * OBD2 requests (service 0x01) and simulates realistic, changing values for:
 *   - PID 0x0C  engine RPM
 *   - PID 0x0D  vehicle speed
 *   - PID 0x05  coolant temperature
 *   - PID 0x11  throttle position
 *   - PID 0x42  control module voltage (substitute for "AT RV")
 *
 * Additionally:
 *   - Service 0x09 PID 0x02 (VIN) -> exercises the multi-frame / ISO-TP path
 *     incl. the flow-control handling in obd_can.cpp
 *   - Service 0x03 (read DTCs) -> returns 2 test trouble codes
 *   - Service 0x04 (clear DTCs) -> acknowledges with a positive response
 *
 * EV / UDS simulation (exercises the physically addressed service 0x22 path):
 *   - On 0x7E0 -> 0x7E8 (this ECU):
 *       22 1164  displayed SoC        2 bytes, (A*256+B)/100  [%]
 *       22 10E0  odometer             4 bytes, A              [km]
 *   - On 0x7E5 -> 0x7ED (simulated e-Golf-style battery ECU "J840"):
 *       22 028C  gross SoC            1 byte,  A              [%]
 *       22 1E3B  HV pack voltage      2 bytes, (A*256+B)/10   [V]
 *       22 1E3D  HV pack current      2 bytes, INT_16 * 0.1   [A]  (neg = discharge)
 *       22 1E0C  cell temperatures    7 bytes, A-40 each [degC] -> MULTI-FRAME
 *
 * WIRING (test board <-> obd2-mqtt board):
 *   test board CAN-H  <-> obd2-mqtt board CAN-H
 *   test board CAN-L  <-> obd2-mqtt board CAN-L
 *   connect GND of both boards (common ground!)
 *   120 Ohm between H and L at EACH end of the bus (measure ~60 Ohm across it).
 *   Do NOT add extra resistors - the Waveshare boards already have onboard
 *   termination (jumper). CJMCU-230 boards do not transmit reliably; use a
 *   Waveshare "SN65HVD230 CAN Board" instead.
 *
 * Pins here: TX=GPIO13, RX=GPIO14 (LilyGO T7-S3 / ESP32-S3-WROOM-1-N16R8).
 * No strapping pins, no clash with the USB-JTAG (GPIO19/20) or internal
 * flash/PSRAM. Freely routable through the S3 GPIO matrix - adjust below if
 * needed.
 */

#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN   GPIO_NUM_13
#define CAN_RX_PIN   GPIO_NUM_14

// The bus bitrate must match the test partner (obd_can_config.h:
// OBD_CAN_BITRATE_KBPS). 500 = normal, 125 only for wiring diagnostics.
#ifndef FAKE_ECU_BITRATE_KBPS
#define FAKE_ECU_BITRATE_KBPS  500
#endif
#if FAKE_ECU_BITRATE_KBPS == 500
#define FAKE_ECU_TIMING  TWAI_TIMING_CONFIG_500KBITS()
#elif FAKE_ECU_BITRATE_KBPS == 250
#define FAKE_ECU_TIMING  TWAI_TIMING_CONFIG_250KBITS()
#elif FAKE_ECU_BITRATE_KBPS == 125
#define FAKE_ECU_TIMING  TWAI_TIMING_CONFIG_125KBITS()
#endif

#define OBD_REQUEST_ID    0x7DF  // functional request from the tester
#define OBD_MY_REQUEST_ID 0x7E0  // "physical" request addressed to this ECU
#define OBD_RESPONSE_ID   0x7E8  // response ID of this simulated ECU

// Second simulated control unit: an e-Golf-style HV battery ECU, physically
// addressed. Uses the +8 response convention (like the real e-Golf BMS).
#define OBD_BMS_REQUEST_ID  0x7E5
#define OBD_BMS_RESPONSE_ID 0x7ED

// ---------------------------------------------------------------------
// Simulated sensor values - they change slowly so that you can see in the
// log that actual "live" data is coming in.
// ---------------------------------------------------------------------
uint16_t simRpm = 800;       // idle RPM as the starting point
uint8_t  simSpeed = 0;       // km/h
uint8_t  simCoolant = 70;    // deg C (OBD2 formula adds the +40 offset: A-40)
uint8_t  simThrottle = 15;   // 0-100 % (scaled to 0-255)
bool     simDirectionUp = true;

// EV / HV battery simulation
uint8_t  simSoc = 64;                 // %
bool     simSocUp = false;
uint32_t simOdometer = 42123;         // km
double   simPackVoltage = 355.0;      // V
int16_t  simPackCurrent = -12;        // A (negative = discharge)
int8_t   simCellTemp[7] = {21, 22, 22, 23, 22, 21, 24}; // deg C

void updateSimValues() {
    // RPM sweeps between 800 and 3000
    if (simDirectionUp) {
        simRpm += 50;
        if (simRpm >= 3000) simDirectionUp = false;
    } else {
        simRpm -= 50;
        if (simRpm <= 800) simDirectionUp = true;
    }
    simSpeed = (simRpm - 800) / 40;         // rough coupling to RPM
    simThrottle = 15 + (simRpm - 800) / 30; // rough coupling as well

    // SoC drifts slowly between 40 and 90 %
    if (simSocUp) { if (++simSoc >= 90) simSocUp = false; }
    else { if (--simSoc <= 40) simSocUp = true; }

    // Pack voltage follows SoC; current tracks "load" (RPM), positive while the
    // simulated SoC is rising ("charging").
    simPackVoltage = 320.0 + simSoc * 0.9;
    simPackCurrent = simSocUp ? 8 : static_cast<int16_t>(-(int) (simRpm - 800) / 20);
    simOdometer += (simSpeed / 20); // rough, just so the value moves
}

// ---------------------------------------------------------------------
// CAN helpers
// ---------------------------------------------------------------------
bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.extd = 0;
    msg.data_length_code = 8;
    for (int i = 0; i < 8; i++) {
        msg.data[i] = i < len ? data[i] : 0x00;
    }
    return twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK;
}

// Waits for a Flow Control frame (0x30) from the tester, e.g. after sending a
// First Frame of a multi-frame response (VIN).
bool waitForFlowControl(uint32_t reqId, uint32_t timeoutMs) {
    twai_message_t msg;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            if ((msg.identifier == reqId || msg.identifier == OBD_REQUEST_ID) &&
                msg.data_length_code > 0 && (msg.data[0] & 0xF0) == 0x30) {
                return true;
            }
        }
    }
    return false; // no FC received - the tester may not follow the protocol
}

// ---------------------------------------------------------------------
// Service 0x01 - current sensor data (single-frame responses)
// ---------------------------------------------------------------------
void handleMode01(uint8_t pid) {
    uint8_t resp[8] = {0};
    uint8_t len = 0;

    switch (pid) {
        case 0x0C: { // RPM, formula: ((A*256)+B)/4
            uint16_t raw = simRpm * 4;
            resp[0] = 0x04; resp[1] = 0x41; resp[2] = 0x0C;
            resp[3] = (raw >> 8) & 0xFF; resp[4] = raw & 0xFF;
            len = 5;
            break;
        }
        case 0x0D: // speed, formula: A
            resp[0] = 0x03; resp[1] = 0x41; resp[2] = 0x0D; resp[3] = simSpeed;
            len = 4;
            break;
        case 0x05: // coolant temp, formula: A-40
            resp[0] = 0x03; resp[1] = 0x41; resp[2] = 0x05; resp[3] = simCoolant + 40;
            len = 4;
            break;
        case 0x11: // throttle, formula: A*100/255
            resp[0] = 0x03; resp[1] = 0x41; resp[2] = 0x11;
            resp[3] = static_cast<uint8_t>(simThrottle * 255 / 100);
            len = 4;
            break;
        case 0x42: { // control module voltage, formula: ((A*256)+B)/1000
            uint16_t raw = 13800; // simulates 13.8V
            resp[0] = 0x04; resp[1] = 0x41; resp[2] = 0x42;
            resp[3] = (raw >> 8) & 0xFF; resp[4] = raw & 0xFF;
            len = 5;
            break;
        }
        default:
            // Negative Response: service not supported (unknown PID)
            resp[0] = 0x03; resp[1] = 0x7F; resp[2] = 0x01; resp[3] = 0x12;
            len = 4;
            break;
    }

    sendFrame(OBD_RESPONSE_ID, resp, len);
    Serial.printf("-> Mode01 PID 0x%02X answered\n", pid);
}

// ---------------------------------------------------------------------
// Service 0x09 PID 0x02 - VIN, deliberately sent as a multi-frame response
// (17 VIN chars + 1 filler byte -> exercises First/Consecutive Frame)
// ---------------------------------------------------------------------
void handleVin() {
    const char *vin = "WVWZZZ1KZAW123456"; // 18 bytes incl. leading filler byte
    const uint8_t totalLen = 1 + 1 + 18;   // service echo + PID + 1 filler + 17 VIN chars -> simplified length
    (void) totalLen;

    uint8_t payload[64] = {0};
    payload[0] = 0x49; // service 0x09 + 0x40
    payload[1] = 0x02; // PID echo
    payload[2] = 0x01; // "number of data items" per spec, simplified to a fixed 1
    memcpy(&payload[3], vin, strlen(vin));
    const uint8_t fullLen = 3 + strlen(vin);

    // First Frame: PCI = 0x1 + upper nibble of the length, then the lower byte
    uint8_t ff[8];
    ff[0] = 0x10 | ((fullLen >> 8) & 0x0F);
    ff[1] = fullLen & 0xFF;
    memcpy(&ff[2], payload, 6);
    sendFrame(OBD_RESPONSE_ID, ff, 8);
    Serial.println("-> VIN First Frame sent, waiting for Flow Control...");

    if (!waitForFlowControl(OBD_MY_REQUEST_ID, 500)) {
        Serial.println("   no Flow Control frame received - sending anyway (test mode)");
    }

    uint8_t sent = 6;
    uint8_t seq = 1;
    while (sent < fullLen) {
        uint8_t chunk = (fullLen - sent) < 7 ? (fullLen - sent) : 7;
        uint8_t cf[8] = {0};
        cf[0] = 0x20 | (seq & 0x0F);
        memcpy(&cf[1], payload + sent, chunk);
        sendFrame(OBD_RESPONSE_ID, cf, 8);
        sent += chunk;
        seq++;
        delay(10); // simulated Separation Time
    }
    Serial.println("-> VIN sent completely");
}

// ---------------------------------------------------------------------
// Generic ISO-TP sender: single frame if it fits, else First Frame +
// Consecutive Frames (waits for the tester's Flow Control).
// ---------------------------------------------------------------------
void sendIsoTp(uint32_t respId, uint32_t reqId, const uint8_t *data, uint8_t len) {
    if (len <= 7) {
        uint8_t sf[8] = {0};
        sf[0] = len; // PCI: Single Frame, lower nibble = length
        memcpy(&sf[1], data, len);
        sendFrame(respId, sf, 1 + len);
        return;
    }

    uint8_t ff[8];
    ff[0] = 0x10 | ((len >> 8) & 0x0F);
    ff[1] = len & 0xFF;
    memcpy(&ff[2], data, 6);
    sendFrame(respId, ff, 8);
    if (!waitForFlowControl(reqId, 500)) {
        Serial.println("   no Flow Control - sending CFs anyway (test mode)");
    }

    uint8_t sent = 6;
    uint8_t seq = 1;
    while (sent < len) {
        uint8_t chunk = (len - sent) < 7 ? (len - sent) : 7;
        uint8_t cf[8] = {0};
        cf[0] = 0x20 | (seq & 0x0F);
        memcpy(&cf[1], data + sent, chunk);
        sendFrame(respId, cf, 1 + chunk);
        sent += chunk;
        seq++;
        delay(10);
    }
}

// ---------------------------------------------------------------------
// Service 0x22 - ReadDataByIdentifier (UDS). Physically addressed; the
// DID is the 2 bytes after the service byte.
// ---------------------------------------------------------------------
void handleMode22(uint32_t reqId, uint16_t did) {
    const uint32_t respId = (reqId == OBD_BMS_REQUEST_ID) ? OBD_BMS_RESPONSE_ID : OBD_RESPONSE_ID;

    uint8_t p[16] = {0};
    p[0] = 0x62;               // service 0x22 + 0x40
    p[1] = (did >> 8) & 0xFF;  // echoed DID
    p[2] = did & 0xFF;
    uint8_t len = 3;

    if (reqId == OBD_MY_REQUEST_ID && did == 0x1164) {          // displayed SoC
        uint16_t raw = simSoc * 100;
        p[3] = (raw >> 8) & 0xFF; p[4] = raw & 0xFF; len = 5;
    } else if (reqId == OBD_MY_REQUEST_ID && did == 0x10E0) {   // odometer [km]
        p[3] = (simOdometer >> 24) & 0xFF; p[4] = (simOdometer >> 16) & 0xFF;
        p[5] = (simOdometer >> 8) & 0xFF;  p[6] = simOdometer & 0xFF; len = 7;
    } else if (reqId == OBD_BMS_REQUEST_ID && did == 0x028C) {  // gross SoC [%]
        p[3] = simSoc; len = 4;
    } else if (reqId == OBD_BMS_REQUEST_ID && did == 0x1E3B) {  // pack voltage [V*10]
        uint16_t raw = (uint16_t) (simPackVoltage * 10.0);
        p[3] = (raw >> 8) & 0xFF; p[4] = raw & 0xFF; len = 5;
    } else if (reqId == OBD_BMS_REQUEST_ID && did == 0x1E3D) {  // pack current [A*10, signed]
        uint16_t raw = (uint16_t) (int16_t) (simPackCurrent * 10);
        p[3] = (raw >> 8) & 0xFF; p[4] = raw & 0xFF; len = 5;
    } else if (reqId == OBD_BMS_REQUEST_ID && did == 0x1E0C) {  // 7 cell temps, A-40 -> multi-frame
        for (int i = 0; i < 7; i++) p[3 + i] = (uint8_t) (simCellTemp[i] + 40);
        len = 3 + 7;
    } else {
        // Negative Response 0x7F <service> <NRC 0x31 requestOutOfRange>
        uint8_t nr[4] = {0x03, 0x7F, 0x22, 0x31};
        sendFrame(respId, nr, 4);
        Serial.printf("-> Mode22 DID 0x%04X on 0x%03lX: NRC 0x31\n", did, (unsigned long) reqId);
        return;
    }

    sendIsoTp(respId, reqId, p, len);
    Serial.printf("-> Mode22 DID 0x%04X on 0x%03lX answered (%u bytes)\n",
                  did, (unsigned long) reqId, len);
}

// ---------------------------------------------------------------------
// Service 0x03 - stored DTCs (2 test trouble codes: P0301, P0420)
// ---------------------------------------------------------------------
void handleMode03() {
    uint8_t resp[8] = {0x05, 0x43, 0x02, 0x03, 0x01, 0x04, 0x20, 0x00};
    // byte0=PCI(5 data bytes) byte1=0x43 byte2=number of DTCs(2)
    // byte3-4 = P0301 (0x0301) byte5-6 = P0420 (0x0420)
    sendFrame(OBD_RESPONSE_ID, resp, 8);
    Serial.println("-> Mode03 (DTCs) answered: P0301, P0420");
}

// ---------------------------------------------------------------------
// Service 0x04 - clear DTCs (always acknowledge success)
// ---------------------------------------------------------------------
void handleMode04() {
    uint8_t resp[8] = {0x01, 0x44, 0, 0, 0, 0, 0, 0};
    sendFrame(OBD_RESPONSE_ID, resp, 2);
    Serial.println("-> Mode04 (clear DTCs) acknowledged");
}

// ---------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------
// One-shot CAN self-test: send a frame with self-reception in NO_ACK mode.
// It runs ESP-TX -> transceiver -> CANH/CANL -> transceiver -> ESP-RX. If it
// comes back, the local CAN hardware (pins, transceiver, Rs, power) is ok and
// the problem is in the board-to-board wiring.
int selfTestResult = -1; // -1 = not run, 0 = fail, 1 = ok

static void canSelfTest() {
    twai_driver_uninstall();
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t t = FAKE_ECU_TIMING;
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        Serial.println("SELFTEST: driver init failed");
        return;
    }

    twai_message_t tx = {};
    tx.identifier = 0x1AB;
    tx.self = 1; // Self-Reception Request
    tx.data_length_code = 3;
    tx.data[0] = 0xDE; tx.data[1] = 0xAD; tx.data[2] = 0xBE;

    bool got = false;
    if (twai_transmit(&tx, pdMS_TO_TICKS(50)) == ESP_OK) {
        twai_message_t rx;
        const uint32_t end = millis() + 200;
        while (millis() < end) {
            if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK && rx.identifier == 0x1AB) {
                got = true;
                break;
            }
        }
    }
    twai_status_info_t st{};
    twai_get_status_info(&st);
    selfTestResult = got ? 1 : 0;
    Serial.printf("SELFTEST: %s  (busErr=%lu txErr=%lu)\n",
                  got ? "OK - local CAN HW works, problem is in the board-to-board wiring"
                      : "FAILED - check transceiver/wiring/Rs/power on THIS board",
                  (unsigned long) st.bus_error_count, (unsigned long) st.tx_error_counter);

    twai_stop();
    twai_driver_uninstall();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Fake ECU starting ===");

    canSelfTest();

#ifdef FAKE_ECU_NO_ACK
    const twai_mode_t canMode = TWAI_MODE_NO_ACK; // receives, but does not acknowledge
#else
    const twai_mode_t canMode = TWAI_MODE_NORMAL;
#endif
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, canMode);
    twai_timing_config_t t_config = FAKE_ECU_TIMING;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("ERROR: twai_driver_install failed");
        while (true) delay(1000);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("ERROR: twai_start failed");
        while (true) delay(1000);
    }

    Serial.println("CAN bus ready, waiting for OBD2 requests (0x7DF / 0x7E0 / 0x7E5)...");
}

unsigned long lastSimUpdate = 0;
uint32_t rxCount = 0;
#ifdef FAKE_ECU_DEBUG
unsigned long lastHeartbeat = 0;
#endif

void loop() {
    // Keep the simulated values moving every 500ms
    if (millis() - lastSimUpdate > 500) {
        updateSimValues();
        lastSimUpdate = millis();
    }

#ifdef FAKE_ECU_DEBUG
    // Heartbeat (1 Hz frame on 0x555) + TWAI status over Serial - shows the test
    // partner that this ECU is alive on the bus, and helps with wiring faults.
    // Off by default so the simulated ECU behaves "cleanly".
    if (millis() - lastHeartbeat > 1000) {
        lastHeartbeat = millis();
        twai_status_info_t st{};
        twai_get_status_info(&st);
        const uint8_t hb[8] = {0xAA, 0x55, (uint8_t) (millis() / 1000), 0, 0, 0, 0, 0};
        const bool ok = sendFrame(0x555, hb, 3);
        Serial.printf("hb: selftest=%s tx=%s rx=%lu state=%d txErr=%lu rxErr=%lu busErr=%lu txq=%lu rxq=%lu\n",
                      selfTestResult == 1 ? "OK" : selfTestResult == 0 ? "FAIL" : "?",
                      ok ? "ok" : "FAIL", (unsigned long) rxCount, (int) st.state,
                      (unsigned long) st.tx_error_counter, (unsigned long) st.rx_error_counter,
                      (unsigned long) st.bus_error_count,
                      (unsigned long) st.msgs_to_tx, (unsigned long) st.msgs_to_rx);
    }
#endif

    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(50)) != ESP_OK) {
        return;
    }
    rxCount++;
#ifdef FAKE_ECU_DEBUG
    Serial.printf("RX id %03lX [", (unsigned long) msg.identifier);
    for (int i = 0; i < msg.data_length_code; i++) Serial.printf("%02X ", msg.data[i]);
    Serial.println("]");
#endif

    // Only react to functional (0x7DF), the OBD ECU (0x7E0) or the simulated
    // battery ECU (0x7E5). Flow Control frames (0x30) are handled separately in
    // waitForFlowControl() and ignored here.
    if (msg.identifier != OBD_REQUEST_ID && msg.identifier != OBD_MY_REQUEST_ID &&
        msg.identifier != OBD_BMS_REQUEST_ID) {
        return;
    }
    if (msg.data_length_code < 3) {
        return;
    }
    if ((msg.data[0] & 0xF0) == 0x30) {
        return; // Flow Control, does not belong here
    }

    const uint8_t service = msg.data[1];
    const uint8_t pid = msg.data[2];

    if (service == 0x22) {
        const uint16_t did = (static_cast<uint16_t>(msg.data[2]) << 8) | msg.data[3];
        handleMode22(msg.identifier, did);
        return;
    }

    // Everything else is only answered on the functional / OBD-ECU address.
    if (msg.identifier == OBD_BMS_REQUEST_ID) {
        return;
    }

    switch (service) {
        case 0x01:
            handleMode01(pid);
            break;
        case 0x03:
            handleMode03();
            break;
        case 0x04:
            handleMode04();
            break;
        case 0x09:
            if (pid == 0x02) {
                handleVin();
            }
            break;
        default:
            Serial.printf("unknown service 0x%02X ignored\n", service);
            break;
    }
}
