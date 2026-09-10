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
// Only compile in the native CAN build. Otherwise the ELM327 class defined here
// collides at link time with the identically named class from the ELMduino lib.
#ifdef USE_CAN

#include "obd_can.h"

ELM327::ELM327() {
    memset(payload, 0, sizeof(payload));
}

#ifdef OBD_CAN_DEBUG
// Local loopback self-test: send a frame with self-reception in NO_ACK mode.
// It runs ESP-TX -> transceiver -> CANH/CANL -> transceiver -> ESP-RX.
// If it comes back, the local CAN hardware (pins, transceiver, Rs, power) is ok.
static void obdCanSelfTest(gpio_num_t txPin, gpio_num_t rxPin) {
    twai_stop();
    twai_driver_uninstall();
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NO_ACK);
    twai_timing_config_t t = OBD_CAN_TIMING_CONFIG;
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        Serial.println("obd_can: SELFTEST driver init failed");
        return;
    }
    twai_message_t tx = {};
    tx.identifier = 0x2CD;
    tx.self = 1;
    tx.data_length_code = 3;
    tx.data[0] = 0xBE; tx.data[1] = 0xEF; tx.data[2] = 0x01;
    bool got = false;
    if (twai_transmit(&tx, pdMS_TO_TICKS(50)) == ESP_OK) {
        twai_message_t rx;
        const uint32_t end = millis() + 200;
        while (millis() < end) {
            if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK && rx.identifier == 0x2CD) { got = true; break; }
        }
    }
    twai_status_info_t st{};
    twai_get_status_info(&st);
    Serial.printf("obd_can: SELFTEST %s (busErr=%lu txErr=%lu)\n",
                  got ? "OK - local CAN HW works" : "FAILED - check transceiver/Rs/power/pins",
                  (unsigned long) st.bus_error_count, (unsigned long) st.tx_error_counter);
    twai_stop();
    twai_driver_uninstall();
}
#endif

bool ELM327::begin(gpio_num_t txPin, gpio_num_t rxPin, bool debugEnabled, uint32_t responseTimeoutMs) {
    this->debug = debugEnabled;
    this->timeoutMs = responseTimeoutMs;

#ifdef OBD_CAN_DEBUG
    obdCanSelfTest(txPin, rxPin);
#endif

    // OBD_CAN_LISTEN_TEST: NO_ACK mode - the controller does NOT acknowledge
    // received frames, but still receives them. Together with OBD_CAN_DEBUG
    // (raw frame log in receiveIsoTp) this lets you check whether the peer's
    // frames physically arrive, independent of the ACK handshake.
#ifdef OBD_CAN_LISTEN_TEST
    const twai_mode_t canMode = TWAI_MODE_NO_ACK;
#else
    const twai_mode_t canMode = TWAI_MODE_NORMAL;
#endif
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, canMode);
    twai_timing_config_t t_config = OBD_CAN_TIMING_CONFIG;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // In case a driver from a previous (failed) begin() is still installed,
    // clean up before re-initializing.
    twai_stop();
    twai_driver_uninstall();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("obd_can: twai_driver_install failed");
        connected = false;
        elm_port = nullptr;
        return false;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("obd_can: twai_start failed");
        connected = false;
        elm_port = nullptr;
        return false;
    }

    initialized = true;
    connected = true;
    // Truthy marker for the elm_port checks in OBDState.cpp/OBDStates.cpp -
    // it is never dereferenced, only checked for != nullptr.
    elm_port = reinterpret_cast<void *>(0x1);

    Serial.println("obd_can: TWAI (CAN) driver started");

#ifdef OBD_CAN_UDS_SELFTEST
    // One-shot check of the physically addressed UDS (service 0x22) path against
    // tools/fake-ecu. Build with -DOBD_CAN_UDS_SELFTEST (add -DOBD_CAN_DEBUG for
    // the raw frame log). Not for production images.
    delay(300);
    struct { const char *sh; uint8_t svc; uint16_t did; uint8_t bytes; const char *what; } tests[] = {
        {"AT SH 7E0", 0x22, 0x1164, 2, "displayed SoC /100 %"},
        {"AT SH 7E5", 0x22, 0x028C, 1, "gross SoC %"},
        {"AT SH 7E5", 0x22, 0x1E3B, 2, "pack voltage /10 V"},
        {"AT SH 7E5", 0x22, 0x1E3D, 2, "pack current *0.1 A (INT16)"},
        {"AT SH 7E5", 0x22, 0x1E0C, 7, "cell temps A-40 (multi-frame)"},
    };
    for (auto &t : tests) {
        sendCommand_Blocking(t.sh);
        const double v = processPID(t.svc, t.did, 1, t.bytes);
        Serial.printf("obd_can: UDS %s %04X -> state=%d value=%.2f payload=%s  (%s)\n",
                      t.sh, t.did, (int) nb_rx_state, v, payload, t.what);
    }
    sendCommand_Blocking("AT D");
#endif

    return true;
}

void ELM327::end() {
    if (initialized) {
        twai_stop();
        twai_driver_uninstall();
        initialized = false;
    }
    connected = false;
    elm_port = nullptr;
}

bool ELM327::sendFrame(const uint32_t id, const uint8_t *data, const uint8_t len) const {
    twai_message_t message = {};
    message.identifier = id;
    message.extd = OBD_CAN_USE_EXTENDED_ID;
    message.data_length_code = 8;

    for (int i = 0; i < 8; i++) {
        // ISO-15765 padding with 0x00 for unused bytes.
        message.data[i] = i < len ? data[i] : 0x00;
    }

    return twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool ELM327::receiveIsoTp(uint32_t &responseId, uint8_t *outData, uint8_t &outLen) {
    twai_message_t message;
    const uint32_t startMs = millis();

    uint16_t totalLen = 0;
    uint16_t received = 0;
    bool firstFrameSeen = false;
    uint8_t expectedSeq = 1;

    while (millis() - startMs < timeoutMs) {
        const uint32_t remaining = timeoutMs - (millis() - startMs);
        if (twai_receive(&message, pdMS_TO_TICKS(remaining < 20 ? 20 : remaining)) != ESP_OK) {
            continue;
        }

#ifdef OBD_CAN_DEBUG
        Serial.printf("obd_can: RX id %03lX [", static_cast<unsigned long>(message.identifier));
        for (int i = 0; i < message.data_length_code; i++) Serial.printf("%02X ", message.data[i]);
        Serial.println("]");
#endif

        // Functional broadcast (0x7DF): responses come back on 0x7E8..0x7EF.
        // Physical addressing ("AT SH", UDS): the response ID is ECU/gateway
        // specific and does not follow a single convention (e.g. VAG: 0x7E5 ->
        // 0x7ED but 0x710 -> 0x77A). Accept the whole diagnostic window and let
        // the service/DID echo check in requestPID() do the real filtering.
        const bool idOk = (activeReqId == OBD_CAN_REQUEST_ID)
                              ? (message.identifier >= OBD_CAN_RESPONSE_ID_MIN &&
                                 message.identifier <= OBD_CAN_RESPONSE_ID_MAX)
                              : (message.identifier >= 0x700 && message.identifier <= 0x7FF &&
                                 message.identifier != activeReqId);
        if (!idOk) {
            continue; // not relevant bus traffic
        }

        if (message.data_length_code == 0) {
            continue;
        }

        const uint8_t pci = message.data[0];
        const uint8_t frameType = (pci & 0xF0) >> 4;

        if (frameType == 0x0) {
            // Single Frame: length is in the lower nibble of the PCI
            const uint8_t len = pci & 0x0F;
            if (len == 0 || len > 7) continue;

            responseId = message.identifier;
            outLen = len;
            memcpy(outData, &message.data[1], len);
            return true;
        }

        if (frameType == 0x1) {
            // First Frame of a multi-frame message
            totalLen = ((pci & 0x0F) << 8) | message.data[1];
            if (totalLen == 0 || totalLen > OBD_CAN_PAYLOAD_LEN) continue;

            received = 6; // 6 data bytes are already in the First Frame
            memcpy(outData, &message.data[2], 6);
            firstFrameSeen = true;
            expectedSeq = 1;
            responseId = message.identifier;

            // Send a Flow Control frame to the responding ECU. For a functional
            // request that is (response ID - 8); for a physically addressed one
            // it is the request ID we used.
            const uint32_t fcTarget = (activeReqId == OBD_CAN_REQUEST_ID)
                                          ? (message.identifier - 8)
                                          : activeReqId;
            const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
            sendFrame(fcTarget, fc, 3);
            continue;
        }

        if (frameType == 0x2 && firstFrameSeen) {
            // Consecutive Frame
            const uint8_t seq = pci & 0x0F;
            if (seq != (expectedSeq & 0x0F)) {
                // Sequence mismatch (lost frame or similar) - abort
                return false;
            }

            const uint8_t remainingBytes = totalLen - received;
            const uint8_t chunk = remainingBytes < 7 ? remainingBytes : 7;
            memcpy(outData + received, &message.data[1], chunk);
            received += chunk;
            expectedSeq++;

            if (received >= totalLen) {
                outLen = totalLen;
                return true;
            }
            continue;
        }
    }

    return false; // timeout
}

// UDS "ReadDataByIdentifier" (0x22) and any PID > 0xFF carry a 2-byte DID;
// standard modes (01/02/09 ...) carry a single PID byte.
static inline bool obdCanTwoByteDid(const uint8_t service, const uint16_t pid) {
    return service == 0x22 || pid > 0xFF;
}

bool ELM327::requestPID(const uint8_t service, const uint16_t pid, uint8_t *outData, uint8_t &outLen) {
    // Mode/service requests with one PID byte (the standard case for mode 01,
    // 02, 09 ...) or a 2-byte DID (UDS service 0x22). Service 0x03/0x04 (DTCs)
    // use currentDTCCodes()/resetDTC().
    const bool twoByteDid = obdCanTwoByteDid(service, pid);
    activeReqId = reqHeader != 0 ? reqHeader : OBD_CAN_REQUEST_ID;

    uint8_t request[8] = {0};
    uint8_t requestLen;
    if (twoByteDid) {
        request[0] = 0x03; // ISO-TP Single Frame, 3 data bytes
        request[1] = service;
        request[2] = (pid >> 8) & 0xFF;
        request[3] = pid & 0xFF;
        requestLen = 4;
    } else {
        request[0] = 0x02;
        request[1] = service;
        request[2] = pid & 0xFF;
        requestLen = 3;
    }

    if (!sendFrame(activeReqId, request, requestLen)) {
        nb_rx_state = ELM_GENERAL_ERROR;
        return false;
    }

    uint32_t responseId = 0;
    uint8_t raw[OBD_CAN_PAYLOAD_LEN] = {0};
    uint8_t rawLen = 0;

    if (!receiveIsoTp(responseId, raw, rawLen)) {
#ifdef OBD_CAN_DEBUG
        twai_status_info_t st{};
        twai_get_status_info(&st);
        Serial.printf("obd_can: req %02X %02X -> TIMEOUT  [state=%d txq=%lu rxq=%lu txErr=%lu rxErr=%lu txFail=%lu arbLost=%lu busErr=%lu]\n",
                      service, pid & 0xFF, (int) st.state,
                      (unsigned long) st.msgs_to_tx, (unsigned long) st.msgs_to_rx,
                      (unsigned long) st.tx_error_counter, (unsigned long) st.rx_error_counter,
                      (unsigned long) st.tx_failed_count, (unsigned long) st.arb_lost_count,
                      (unsigned long) st.bus_error_count);
#endif
        nb_rx_state = ELM_TIMEOUT;
        return false;
    }

#ifdef OBD_CAN_DEBUG
    Serial.printf("obd_can: req %02X %02X -> id %03lX [", service, pid & 0xFF,
                  static_cast<unsigned long>(responseId));
    for (uint8_t i = 0; i < rawLen; i++) Serial.printf("%02X ", raw[i]);
    Serial.println("]");
#endif

    // Expected positive response: byte0 = service + 0x40 (echo), then the echoed
    // PID (1 byte) or DID (2 bytes).
    const uint8_t echoLen = twoByteDid ? 3 : 2;
    const bool didEcho = !twoByteDid ||
                         (rawLen >= 3 && raw[1] == ((pid >> 8) & 0xFF) && raw[2] == (pid & 0xFF));
    if (rawLen < echoLen || raw[0] != static_cast<uint8_t>(service + 0x40) || !didEcho) {
#ifdef OBD_CAN_DEBUG
        if (rawLen >= 3 && raw[0] == 0x7F) {
            // Negative Response: raw[1] = requested service, raw[2] = NRC
            // (0x11 serviceNotSupported, 0x31 requestOutOfRange, 0x7F/0x22
            // serviceNotSupportedInActiveSession -> needs 10 03, ...).
            Serial.printf("obd_can: NRC for %02X %04X -> 0x%02X\n", service, pid, raw[2]);
        }
#endif
        nb_rx_state = ELM_NO_DATA;
        return false;
    }

    outLen = rawLen;
    memcpy(outData, raw, rawLen);
    return true;
}

double ELM327::processPID(const uint8_t service, const uint16_t pid, const uint8_t numResponses,
                           const uint8_t numExpectedBytes, const double scaleFactor, const double bias) {
    nb_rx_state = ELM_GETTING_MSG;

    uint8_t raw[OBD_CAN_PAYLOAD_LEN] = {0};
    uint8_t rawLen = 0;

    if (!requestPID(service, pid, raw, rawLen)) {
        payload[0] = '\0';
        return 0;
    }

    // Store the raw response (hex) in the payload buffer - used e.g. for
    // expression evaluation ($payload) and diagnostic output.
    size_t pos = 0;
    for (int i = 0; i < rawLen && pos + 2 < sizeof(payload); i++) {
        pos += snprintf(payload + pos, sizeof(payload) - pos, "%02X", raw[i]);
    }
    payload[pos] = '\0';

    // The data starts after the echo bytes: service+0x40 plus the PID (1 byte)
    // or DID (2 bytes). numExpectedBytes reads the value from this offset, big
    // endian; fields at a deeper offset are extracted in a computed state via
    // the expression byte accessor ($rawState.bN:M) on the full hex payload.
    const uint8_t echoLen = obdCanTwoByteDid(service, pid) ? 3 : 2;
    const uint8_t *data = raw + echoLen;
    const uint8_t dataLen = rawLen >= echoLen ? rawLen - echoLen : 0;
    const uint8_t useLen = numExpectedBytes < dataLen ? numExpectedBytes : dataLen;

    uint32_t rawValue = 0;
    for (int i = 0; i < useLen; i++) {
        rawValue = (rawValue << 8) | data[i];
    }

    nb_rx_state = ELM_SUCCESS;
    return static_cast<double>(rawValue) * scaleFactor + bias;
}

elm_can_rxstate_t ELM327::sendCommand_Blocking(const char *cmd) {
    // OBDState.cpp issues exactly two AT commands, formatted from the macros in
    // obd_can.h: "AT SH <HEX>" before a request that carries pid.header, and
    // "AT D" afterwards. Parse them so physically addressed UDS reads work; any
    // other command is a harmless no-op.
    if (cmd != nullptr) {
        if (strncmp(cmd, "AT SH ", 6) == 0) {
            reqHeader = strtoul(cmd + 6, nullptr, 16);
        } else if (strcmp(cmd, "AT D") == 0) {
            reqHeader = 0;
        }
    }
    strlcpy(payload, RESPONSE_OK, sizeof(payload));
    nb_rx_state = ELM_SUCCESS;
    return ELM_SUCCESS;
}

std::string ELM327::decodeDTC(const uint8_t b1, const uint8_t b2) {
    // SAE J2012 encoding: the top 2 bits of b1 determine the prefix.
    static const char prefixes[4] = {'P', 'C', 'B', 'U'};
    const char prefix = prefixes[(b1 >> 6) & 0x03];
    const uint8_t digit1 = (b1 >> 4) & 0x03;

    char code[6];
    snprintf(code, sizeof(code), "%c%01X%01X%02X", prefix, digit1, b1 & 0x0F, b2);
    return {code};
}

void ELM327::currentDTCCodes() {
    // Service 0x03: read stored DTCs (no PID byte needed)
    const uint8_t request[8] = {0x01, 0x03, 0, 0, 0, 0, 0, 0};
    activeReqId = OBD_CAN_REQUEST_ID; // functional broadcast, standard response window

    DTC_Response.codesFound = 0;

    if (!sendFrame(OBD_CAN_REQUEST_ID, request, 1)) {
        nb_rx_state = ELM_GENERAL_ERROR;
        return;
    }

    uint32_t responseId = 0;
    uint8_t raw[OBD_CAN_PAYLOAD_LEN] = {0};
    uint8_t rawLen = 0;

    if (!receiveIsoTp(responseId, raw, rawLen) || rawLen < 1 || raw[0] != 0x43) {
        nb_rx_state = ELM_NO_DATA;
        return;
    }

    // raw[1] = number of DTCs, followed by 2-byte pairs per code
    const uint8_t count = rawLen >= 2 ? raw[1] : 0;
    const uint8_t maxCodes = count < OBD_CAN_MAX_DTC ? count : OBD_CAN_MAX_DTC;

    for (int i = 0; i < maxCodes; i++) {
        const int off = 2 + i * 2;
        if (off + 1 >= rawLen) break;
        if (raw[off] == 0 && raw[off + 1] == 0) continue; // skip empty code

        DTC_Response.codes[DTC_Response.codesFound++] = decodeDTC(raw[off], raw[off + 1]);
    }

    nb_rx_state = ELM_SUCCESS;
}

bool ELM327::resetDTC() {
    // Service 0x04: clear all stored DTCs
    const uint8_t request[8] = {0x01, 0x04, 0, 0, 0, 0, 0, 0};
    activeReqId = OBD_CAN_REQUEST_ID;

    if (!sendFrame(OBD_CAN_REQUEST_ID, request, 1)) {
        return false;
    }

    uint32_t responseId = 0;
    uint8_t raw[OBD_CAN_PAYLOAD_LEN] = {0};
    uint8_t rawLen = 0;

    // The positive response to service 0x04 is 0x44 (no further payload)
    return receiveIsoTp(responseId, raw, rawLen) && rawLen >= 1 && raw[0] == 0x44;
}

double ELM327::batteryVoltage() {
    // Substitute for "AT RV" (the ELM327's own ADC measurement on OBD pin 16):
    // PID 0142 "Control module voltage", formula per SAE J1979: ((A*256)+B)/1000
    const double raw = processPID(0x01, 0x42, 1, 2, 1, 0);
    return raw / 1000.0;
}

#endif // USE_CAN
