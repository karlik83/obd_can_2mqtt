/*
 * This program is free software; you can use it, redistribute it
 * and / or modify it under the terms of the GNU General Public License
 * (GPL) as published by the Free Software Foundation; either version 3
 * of the License or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program, in a file called gpl.txt or license.txt.
 * If not, write to the Free Software Foundation Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307 USA
 */
//
// obd_can.h
//
// Drop-in replacement for ELMduino's ELM327 class that does not talk to an
// external ELM327 chip over Bluetooth, but uses the ESP32's own TWAI (CAN)
// controller directly and builds/reassembles OBD2 / ISO-15765 requests itself.
//
// IMPORTANT: this class is deliberately named exactly like the ELMduino class
// (ELM327) and offers the same members used by OBDState.cpp/OBDStates.cpp/obd.cpp.
// That way OBDState.h/OBDStates.h do NOT need changes - they only switch between
//   #include <ELMduino.h>      (Bluetooth variant)
// and
//   #include "obd_can.h"       (USE_CAN, this file)
#pragma once

#ifndef OBD2_MQTT_OBD_CAN_H
#define OBD2_MQTT_OBD_CAN_H

#include <Arduino.h>
#include <string>
#include "driver/twai.h"
#include "obd_can_config.h"

// ---------------------------------------------------------------------
// Status codes, compatible with the ELMduino constants used in
// OBDState.cpp/obd.cpp (ELM_SUCCESS / ELM_NO_DATA / ELM_GETTING_MSG).
// The other values are purely internal and not read by the existing code.
// ---------------------------------------------------------------------
typedef enum {
    ELM_SUCCESS = 0,
    ELM_NO_DATA,
    ELM_GETTING_MSG,
    ELM_TIMEOUT,
    ELM_BUS_ERROR,
    ELM_NO_RESPONSE,
    ELM_GENERAL_ERROR
} elm_can_rxstate_t;

// AUTOMATIC protocol constant, used by obd.cpp/Settings as the default (the
// protocol parameter of begin()/OBDClass). Meaningless in CAN mode, but the
// symbol must exist because main.cpp/settings.cpp still reference "AUTOMATIC".
#ifndef AUTOMATIC
#define AUTOMATIC '0'
#endif

// Macros/strings referenced by OBDState.cpp for the (unused in CAN mode) header
// handling - they only need to exist so the code compiles unchanged.
#define SET_HEADER            "AT SH %s"
#define SET_ALL_TO_DEFAULTS   "AT D"
#define RESPONSE_OK            "OK"

// Defined in ELMduino.h; OBDState.cpp uses it to group the "supported PIDs"
// queries (0x00/0x20/0x40/...).
#ifndef PID_INTERVAL_OFFSET
#define PID_INTERVAL_OFFSET   0x20
#endif

#define OBD_CAN_MAX_DTC        16
#define OBD_CAN_PAYLOAD_LEN    64

// Compatible with ELMduino's DTC_Response struct (codesFound / codes[i]),
// see obd.cpp: elm327.DTC_Response.codesFound / .codes[i]
struct DTCResponseCompat {
    uint8_t codesFound = 0;
    std::string codes[OBD_CAN_MAX_DTC];
};

class ELM327 {
public:
    // --- Members read by OBDState.cpp / OBDStates.cpp / obd.cpp ---
    elm_can_rxstate_t nb_rx_state = ELM_GETTING_MSG;
    bool connected = false;
    bool specifyNumResponses = true;

    // Only used as a truthy check (OBDState.cpp:403, OBDStates.cpp:182)
    // -> indicates whether the CAN interface is initialized.
    void *elm_port = nullptr;

    static const uint16_t PAYLOAD_LEN = OBD_CAN_PAYLOAD_LEN;
    char payload[OBD_CAN_PAYLOAD_LEN + 1] = {'\0'};

    DTCResponseCompat DTC_Response;

    ELM327();

    // --- Init / connection ---
    // Replaces elm327.begin(stream, debug, timeout, protocol) of the Bluetooth
    // variant. Called from obd.cpp (USE_CAN branch) with the pins from
    // obd_can_config.h.
    bool begin(gpio_num_t txPin = OBD_CAN_TX_PIN, gpio_num_t rxPin = OBD_CAN_RX_PIN,
               bool debug = false, uint32_t timeoutMs = OBD_CAN_RESPONSE_TIMEOUT_MS);

    void end();

    // --- Core function, called from every OBDState::readValue() ---
    // (see OBDState.cpp:126,434,438)
    double processPID(uint8_t service, uint16_t pid, uint8_t numResponses,
                       uint8_t numExpectedBytes, double scaleFactor = 1, double bias = 0);

    // Called from OBDState.cpp for AT header commands. In CAN mode the only ones
    // issued are "AT SH <hex>" (set the target ECU request ID, for UDS service
    // 0x22 on a physically addressed control unit) and "AT D" (reset to the
    // functional broadcast). Both are parsed here; anything else is a no-op.
    // Always returns ELM_SUCCESS with an "OK" payload so the flow does not block.
    elm_can_rxstate_t sendCommand_Blocking(const char *cmd);

    // --- Diagnostic Trouble Codes (Service 0x03/0x04) ---
    void currentDTCCodes();

    bool resetDTC();

    // The standard ELM327 AT command "AT RV" reads the supply voltage right at
    // the OBD connector pin of the adapter - which does not exist without an
    // ELM327 chip. As a substitute PID 0x42 (Control Module Voltage) is queried,
    // which is available on most vehicles but only works while the ECUs are
    // communicating (not with the ignition off).
    double batteryVoltage();

private:
    bool initialized = false;
    bool debug = false;
    uint32_t timeoutMs = OBD_CAN_RESPONSE_TIMEOUT_MS;

    // Target ECU request ID set via "AT SH <hex>". 0 = functional broadcast
    // (0x7DF). A non-zero value switches requestPID() to physical addressing and
    // makes 16-bit DIDs (service 0x22) usable.
    uint32_t reqHeader = 0;

    // The request ID actually used by the request currently in flight - set by
    // requestPID()/currentDTCCodes()/resetDTC() so receiveIsoTp() knows which
    // response IDs to accept and where to send the Flow Control frame.
    uint32_t activeReqId = OBD_CAN_REQUEST_ID;

    bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len) const;

    // Sends an OBD2 request (service+PID) and returns the fully reassembled
    // response (after ISO-TP reassembly) including the two echo bytes
    // (service+0x40, PID) at the front.
    bool requestPID(uint8_t service, uint16_t pid, uint8_t *outData, uint8_t &outLen);

    // Keeps receiving frames until either a complete ISO-TP message has been
    // reassembled or the timeout is reached.
    bool receiveIsoTp(uint32_t &responseId, uint8_t *outData, uint8_t &outLen);

    static std::string decodeDTC(uint8_t b1, uint8_t b2);
};

#endif //OBD2_MQTT_OBD_CAN_H
