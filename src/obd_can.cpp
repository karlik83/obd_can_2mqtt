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
#include "obd_can.h"

ELM327::ELM327() {
    memset(payload, 0, sizeof(payload));
}

bool ELM327::begin(gpio_num_t txPin, gpio_num_t rxPin, bool debugEnabled, uint32_t responseTimeoutMs) {
    this->debug = debugEnabled;
    this->timeoutMs = responseTimeoutMs;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Falls von einem vorherigen (fehlgeschlagenen) begin() noch ein Treiber
    // installiert ist, sauber aufraeumen, bevor neu initialisiert wird.
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
    // Truthy-Marker fuer die elm_port-Checks in OBDState.cpp/OBDStates.cpp -
    // es wird nie dereferenziert, nur auf != nullptr geprueft.
    elm_port = reinterpret_cast<void *>(0x1);

    Serial.println("obd_can: TWAI (CAN) driver started");
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
        // ISO-15765-Padding mit 0x00 fuer ungenutzte Bytes.
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

        if (message.identifier < OBD_CAN_RESPONSE_ID_MIN || message.identifier > OBD_CAN_RESPONSE_ID_MAX) {
            continue; // nicht relevanter Bus-Traffic
        }

        if (message.data_length_code == 0) {
            continue;
        }

        const uint8_t pci = message.data[0];
        const uint8_t frameType = (pci & 0xF0) >> 4;

        if (frameType == 0x0) {
            // Single Frame: laenge steckt im unteren Nibble von PCI
            const uint8_t len = pci & 0x0F;
            if (len == 0 || len > 7) continue;

            responseId = message.identifier;
            outLen = len;
            memcpy(outData, &message.data[1], len);
            return true;
        }

        if (frameType == 0x1) {
            // First Frame einer Multi-Frame-Nachricht
            totalLen = ((pci & 0x0F) << 8) | message.data[1];
            if (totalLen == 0 || totalLen > OBD_CAN_PAYLOAD_LEN) continue;

            received = 6; // 6 Datenbytes stecken bereits im First Frame
            memcpy(outData, &message.data[2], 6);
            firstFrameSeen = true;
            expectedSeq = 1;
            responseId = message.identifier;

            // Flow-Control-Frame an die passende Request-ID der antwortenden
            // ECU senden (Response-ID - 8 == zugehoerige Request-ID).
            const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
            sendFrame(message.identifier - 8, fc, 3);
            continue;
        }

        if (frameType == 0x2 && firstFrameSeen) {
            // Consecutive Frame
            const uint8_t seq = pci & 0x0F;
            if (seq != (expectedSeq & 0x0F)) {
                // Sequenz passt nicht (verlorenes Frame o.ae.) - abbrechen
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

    return false; // Timeout
}

bool ELM327::requestPID(const uint8_t service, const uint16_t pid, uint8_t *outData, uint8_t &outLen) {
    // Mode/Service-Requests mit einem PID-Byte (Standard-Fall fuer Mode 01,
    // 02, 09 ...). Service 0x03/0x04 (DTCs) nutzen currentDTCCodes()/resetDTC().
    const uint8_t request[8] = {
        0x02, service, static_cast<uint8_t>(pid & 0xFF), 0, 0, 0, 0, 0
    };

    if (!sendFrame(OBD_CAN_REQUEST_ID, request, 3)) {
        nb_rx_state = ELM_GENERAL_ERROR;
        return false;
    }

    uint32_t responseId = 0;
    uint8_t raw[OBD_CAN_PAYLOAD_LEN] = {0};
    uint8_t rawLen = 0;

    if (!receiveIsoTp(responseId, raw, rawLen)) {
        nb_rx_state = ELM_TIMEOUT;
        return false;
    }

    // Erwartete positive Antwort: Byte0 = service+0x40 (Echo), Byte1 = PID
    if (rawLen < 2 || raw[0] != static_cast<uint8_t>(service + 0x40)) {
        // Byte0 == 0x7F bedeutet "Negative Response" (z.B. PID nicht unterstuetzt)
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

    // Rohantwort (hex) im payload-Puffer ablegen - wird u.a. fuer
    // Expression-Auswertung ($payload) und Diagnose-Ausgabe genutzt.
    size_t pos = 0;
    for (int i = 0; i < rawLen && pos + 2 < sizeof(payload); i++) {
        pos += snprintf(payload + pos, sizeof(payload) - pos, "%02X", raw[i]);
    }
    payload[pos] = '\0';

    // Nutzdaten beginnen nach den zwei Echo-Bytes (Service+0x40, PID)
    const uint8_t *data = raw + 2;
    const uint8_t dataLen = rawLen >= 2 ? rawLen - 2 : 0;
    const uint8_t useLen = numExpectedBytes < dataLen ? numExpectedBytes : dataLen;

    uint32_t rawValue = 0;
    for (int i = 0; i < useLen; i++) {
        rawValue = (rawValue << 8) | data[i];
    }

    nb_rx_state = ELM_SUCCESS;
    return static_cast<double>(rawValue) * scaleFactor + bias;
}

elm_can_rxstate_t ELM327::sendCommand_Blocking(const char *cmd) {
    // Im CAN-Modus gibt es keine AT-Kommandos (Header setzen etc.) mehr.
    // Wird von OBDState.cpp nur fuer optionale Header-Konfiguration
    // aufgerufen - dort einfach als Erfolg quittieren, damit der bestehende
    // Ablauf (SET_HEADER / SET_ALL_TO_DEFAULTS) nicht blockiert.
    (void) cmd;
    strlcpy(payload, RESPONSE_OK, sizeof(payload));
    nb_rx_state = ELM_SUCCESS;
    return ELM_SUCCESS;
}

std::string ELM327::decodeDTC(const uint8_t b1, const uint8_t b2) {
    // SAE-J2012-Kodierung: die oberen 2 Bit von b1 bestimmen das Praefix.
    static const char prefixes[4] = {'P', 'C', 'B', 'U'};
    const char prefix = prefixes[(b1 >> 6) & 0x03];
    const uint8_t digit1 = (b1 >> 4) & 0x03;

    char code[6];
    snprintf(code, sizeof(code), "%c%01X%01X%02X", prefix, digit1, b1 & 0x0F, b2);
    return {code};
}

void ELM327::currentDTCCodes() {
    // Service 0x03: gespeicherte DTCs auslesen (kein PID-Byte noetig)
    const uint8_t request[8] = {0x01, 0x03, 0, 0, 0, 0, 0, 0};

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

    // raw[1] = Anzahl DTCs, danach folgen Paare aus je 2 Byte pro Code
    const uint8_t count = rawLen >= 2 ? raw[1] : 0;
    const uint8_t maxCodes = count < OBD_CAN_MAX_DTC ? count : OBD_CAN_MAX_DTC;

    for (int i = 0; i < maxCodes; i++) {
        const int off = 2 + i * 2;
        if (off + 1 >= rawLen) break;
        if (raw[off] == 0 && raw[off + 1] == 0) continue; // Leercode ueberspringen

        DTC_Response.codes[DTC_Response.codesFound++] = decodeDTC(raw[off], raw[off + 1]);
    }

    nb_rx_state = ELM_SUCCESS;
}

bool ELM327::resetDTC() {
    // Service 0x04: alle gespeicherten DTCs loeschen
    const uint8_t request[8] = {0x01, 0x04, 0, 0, 0, 0, 0, 0};

    if (!sendFrame(OBD_CAN_REQUEST_ID, request, 1)) {
        return false;
    }

    uint32_t responseId = 0;
    uint8_t raw[OBD_CAN_PAYLOAD_LEN] = {0};
    uint8_t rawLen = 0;

    // Positive Antwort auf Service 0x04 ist 0x44 (kein weiterer Payload)
    return receiveIsoTp(responseId, raw, rawLen) && rawLen >= 1 && raw[0] == 0x44;
}

double ELM327::batteryVoltage() {
    // Ersatz fuer "AT RV" (ELM327-eigene ADC-Messung am OBD-Pin 16):
    // PID 0142 "Control module voltage", Formel laut SAE J1979: ((A*256)+B)/1000
    const double raw = processPID(0x01, 0x42, 1, 2, 1, 0);
    return raw / 1000.0;
}
