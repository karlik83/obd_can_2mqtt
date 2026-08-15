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
// Drop-in-Ersatz fuer ELMduino's ELM327-Klasse, der aber nicht mit einem
// externen ELM327-Chip per Bluetooth spricht, sondern den ESP32-eigenen
// TWAI-Controller (CAN) direkt nutzt und OBD2/ISO-15765-Anfragen selbst
// aufbaut und zusammensetzt.
//
// WICHTIG: Diese Klasse heisst bewusst genauso wie die ELMduino-Klasse
// (ELM327) und bietet dieselben von OBDState.cpp/OBDStates.cpp/obd.cpp
// verwendeten Member. Dadurch muessen OBDState.h/OBDStates.h NICHT
// angepasst werden - dort wird lediglich zwischen
//   #include <ELMduino.h>      (Bluetooth-Variante)
// und
//   #include "obd_can.h"       (USE_CAN, diese Datei)
// umgeschaltet.
#pragma once

#ifndef OBD2_MQTT_OBD_CAN_H
#define OBD2_MQTT_OBD_CAN_H

#include <Arduino.h>
#include <string>
#include "driver/twai.h"
#include "obd_can_config.h"

// ---------------------------------------------------------------------
// Status-Codes, kompatibel zu den in OBDState.cpp/obd.cpp verwendeten
// ELMduino-Konstanten (ELM_SUCCESS / ELM_NO_DATA / ELM_GETTING_MSG).
// Weitere Werte sind rein intern und werden vom bestehenden Code nicht
// abgefragt.
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

// AUTOMATIC/-Protokoll-Konstante, wird von obd.cpp/Settings als Default
// verwendet (protocol-Parameter von begin()/OBDClass). Im CAN-Modus ohne
// Bedeutung, muss aber als Symbol existieren, da main.cpp/settings.cpp
// weiterhin "AUTOMATIC" referenzieren.
#ifndef AUTOMATIC
#define AUTOMATIC '0'
#endif

// Von OBDState.cpp fuer das (im CAN-Modus ungenutzte) Header-Handling
// referenzierte Makros/Strings - muessen nur existieren, damit der Code
// unveraendert compiliert.
#define SET_HEADER            "AT SH %s"
#define SET_ALL_TO_DEFAULTS   "AT D"
#define RESPONSE_OK            "OK"

#define OBD_CAN_MAX_DTC        16
#define OBD_CAN_PAYLOAD_LEN    64

// Kompatibel zu ELMduino's DTC_Response-Struct (codesFound / codes[i]),
// siehe obd.cpp: elm327.DTC_Response.codesFound / .codes[i]
struct DTCResponseCompat {
    uint8_t codesFound = 0;
    std::string codes[OBD_CAN_MAX_DTC];
};

class ELM327 {
public:
    // --- Von OBDState.cpp / OBDStates.cpp / obd.cpp gelesene Member ---
    elm_can_rxstate_t nb_rx_state = ELM_GETTING_MSG;
    bool connected = false;
    bool specifyNumResponses = true;

    // Nur als Truthy-Check verwendet (OBDState.cpp:403, OBDStates.cpp:182)
    // -> zeigt an, ob die CAN-Schnittstelle initialisiert ist.
    void *elm_port = nullptr;

    static const uint16_t PAYLOAD_LEN = OBD_CAN_PAYLOAD_LEN;
    char payload[OBD_CAN_PAYLOAD_LEN + 1] = {'\0'};

    DTCResponseCompat DTC_Response;

    ELM327();

    // --- Init / Verbindung ---
    // Ersetzt elm327.begin(stream, debug, timeout, protocol) der Bluetooth-
    // Variante. Wird aus obd.cpp (USE_CAN-Zweig) mit den Pins aus
    // obd_can_config.h aufgerufen.
    bool begin(gpio_num_t txPin = OBD_CAN_TX_PIN, gpio_num_t rxPin = OBD_CAN_RX_PIN,
               bool debug = false, uint32_t timeoutMs = OBD_CAN_RESPONSE_TIMEOUT_MS);

    void end();

    // --- Kernfunktion, von jedem OBDState::readValue() aufgerufen ---
    // (siehe OBDState.cpp:126,434,438)
    double processPID(uint8_t service, uint16_t pid, uint8_t numResponses,
                       uint8_t numExpectedBytes, double scaleFactor = 1, double bias = 0);

    // Wird in OBDState.cpp fuer AT-Header-Kommandos aufgerufen. Im CAN-Modus
    // gibt es keine AT-Kommandos mehr - liefert daher immer ELM_SUCCESS mit
    // leerem Payload, damit der bestehende Ablauf nicht blockiert.
    elm_can_rxstate_t sendCommand_Blocking(const char *cmd);

    // --- Diagnostic Trouble Codes (Service 0x03/0x04) ---
    void currentDTCCodes();

    bool resetDTC();

    // Standard-ELM327-AT-Kommando "AT RV" liest die Versorgungsspannung am
    // OBD-Stecker direkt am Adapter-Pin - das gibt es ohne ELM327-Chip nicht.
    // Ersatzweise wird PID 0x42 (Control Module Voltage) abgefragt, was auf
    // den allermeisten Fahrzeugen verfuegbar ist, aber nur bei aktiver
    // Steuergeraete-Kommunikation funktioniert (nicht bei Zuendung aus).
    double batteryVoltage();

private:
    bool initialized = false;
    bool debug = false;
    uint32_t timeoutMs = OBD_CAN_RESPONSE_TIMEOUT_MS;

    bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len) const;

    // Sendet eine OBD2-Anfrage (Service+PID) und liefert die vollstaendig
    // zusammengesetzte Antwort (nach ISO-TP-Reassembly) inkl. der beiden
    // Echo-Bytes (Service+0x40, PID) am Anfang.
    bool requestPID(uint8_t service, uint16_t pid, uint8_t *outData, uint8_t &outLen);

    // Nimmt solange Frames entgegen, bis entweder eine vollstaendige
    // ISO-TP-Nachricht zusammengesetzt wurde oder der Timeout erreicht ist.
    bool receiveIsoTp(uint32_t &responseId, uint8_t *outData, uint8_t &outLen);

    static std::string decodeDTC(uint8_t b1, uint8_t b2);
};

#endif //OBD2_MQTT_OBD_CAN_H
