/*
 * fake_ecu / main.cpp
 *
 * Simulierte Fahrzeug-ECU fuer den Test von ../../src/obd_can.cpp OHNE Auto.
 *
 * Laeuft auf einem zweiten, GUENSTIGEN ESP32-Devkit + SN65HVD230-CAN-
 * Transceiver. Beantwortet Standard-OBD2-Anfragen (Service 0x01) und
 * simuliert dabei realistische, sich veraendernde Werte fuer:
 *   - PID 0x0C  Drehzahl (RPM)
 *   - PID 0x0D  Geschwindigkeit
 *   - PID 0x05  Kuehlmitteltemperatur
 *   - PID 0x11  Drosselklappenstellung
 *   - PID 0x42  Steuergeraete-Spannung (Ersatz fuer "AT RV")
 *
 * Zusaetzlich:
 *   - Service 0x09 PID 0x02 (VIN) -> testet den Multi-Frame/ISO-TP-Pfad
 *     inkl. Flow-Control-Handling deiner obd_can.cpp
 *   - Service 0x03 (DTCs lesen) -> liefert 2 Testfehlercodes
 *   - Service 0x04 (DTCs loeschen) -> quittiert mit positiver Antwort
 *
 * VERKABELUNG (Testboard <-> obd2-mqtt-Board):
 *   Testboard SN65HVD230 CAN-H  <-> obd2-mqtt-Board SN65HVD230 CAN-H
 *   Testboard SN65HVD230 CAN-L  <-> obd2-mqtt-Board SN65HVD230 CAN-L
 *   GND von beiden Boards verbinden (gemeinsame Masse!)
 *   An JEDEM Ende der CAN-H/CAN-L-Leitung 120 Ohm zwischen H und L -
 *   also insgesamt 2x 120 Ohm (nicht mehr, sonst Bus zu stark belastet).
 *
 * Pins hier: TX=GPIO13, RX=GPIO14 (LilyGO T7_S3 / ESP32-S3-WROOM-1-N16R8).
 * Keine Strapping-Pins, kollidieren nicht mit USB-JTAG (GPIO19/20) oder
 * internem Flash/PSRAM. Ueber die GPIO-Matrix des S3 frei waehlbar - bei
 * Bedarf unten anpassen.
 */

#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN   GPIO_NUM_13
#define CAN_RX_PIN   GPIO_NUM_14

#define OBD_REQUEST_ID   0x7DF   // funktionale Anfrage vom Tester
#define OBD_MY_REQUEST_ID 0x7E0  // "physische" Anfrage direkt an diese ECU
#define OBD_RESPONSE_ID  0x7E8   // Antwort-ID dieser simulierten ECU

// ---------------------------------------------------------------------
// Simulierte Sensorwerte - veraendern sich langsam, damit man im Log
// sieht, dass tatsaechlich "lebende" Daten reinkommen.
// ---------------------------------------------------------------------
uint16_t simRpm = 800;       // Standgas-RPM als Start
uint8_t  simSpeed = 0;       // km/h
uint8_t  simCoolant = 70;    // Grad C + 40 Offset (OBD2-Formel: A-40)
uint8_t  simThrottle = 15;   // 0-100 % (skaliert auf 0-255)
bool     simDirectionUp = true;

void updateSimValues() {
    // RPM pendelt zwischen 800 und 3000
    if (simDirectionUp) {
        simRpm += 50;
        if (simRpm >= 3000) simDirectionUp = false;
    } else {
        simRpm -= 50;
        if (simRpm <= 800) simDirectionUp = true;
    }
    simSpeed = (simRpm - 800) / 40;         // grobe Kopplung an RPM
    simThrottle = 15 + (simRpm - 800) / 30; // ebenfalls grob gekoppelt
}

// ---------------------------------------------------------------------
// CAN-Hilfsfunktionen
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

// Wartet auf einen Flow-Control-Frame (0x30) vom Tester, z.B. nach dem
// Senden eines First Frame bei einer Multi-Frame-Antwort (VIN).
bool waitForFlowControl(uint32_t timeoutMs) {
    twai_message_t msg;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            if ((msg.identifier == OBD_MY_REQUEST_ID || msg.identifier == OBD_REQUEST_ID) &&
                msg.data_length_code > 0 && (msg.data[0] & 0xF0) == 0x30) {
                return true;
            }
        }
    }
    return false; // kein FC bekommen - Tester haelt sich evtl. nicht ans Protokoll
}

// ---------------------------------------------------------------------
// Service 0x01 - aktuelle Sensordaten (Single-Frame-Antworten)
// ---------------------------------------------------------------------
void handleMode01(uint8_t pid) {
    uint8_t resp[8] = {0};
    uint8_t len = 0;

    switch (pid) {
        case 0x0C: { // RPM, Formel: ((A*256)+B)/4
            uint16_t raw = simRpm * 4;
            resp[0] = 0x04; resp[1] = 0x41; resp[2] = 0x0C;
            resp[3] = (raw >> 8) & 0xFF; resp[4] = raw & 0xFF;
            len = 5;
            break;
        }
        case 0x0D: // Speed, Formel: A
            resp[0] = 0x03; resp[1] = 0x41; resp[2] = 0x0D; resp[3] = simSpeed;
            len = 4;
            break;
        case 0x05: // Kuehlmitteltemp, Formel: A-40
            resp[0] = 0x03; resp[1] = 0x41; resp[2] = 0x05; resp[3] = simCoolant + 40;
            len = 4;
            break;
        case 0x11: // Drosselklappe, Formel: A*100/255
            resp[0] = 0x03; resp[1] = 0x41; resp[2] = 0x11;
            resp[3] = static_cast<uint8_t>(simThrottle * 255 / 100);
            len = 4;
            break;
        case 0x42: { // Steuergeraete-Spannung, Formel: ((A*256)+B)/1000
            uint16_t raw = 13800; // simuliert 13.8V
            resp[0] = 0x04; resp[1] = 0x41; resp[2] = 0x42;
            resp[3] = (raw >> 8) & 0xFF; resp[4] = raw & 0xFF;
            len = 5;
            break;
        }
        default:
            // Negative Response: Service nicht unterstuetzt (PID unbekannt)
            resp[0] = 0x03; resp[1] = 0x7F; resp[2] = 0x01; resp[3] = 0x12;
            len = 4;
            break;
    }

    sendFrame(OBD_RESPONSE_ID, resp, len);
    Serial.printf("-> Mode01 PID 0x%02X beantwortet\n", pid);
}

// ---------------------------------------------------------------------
// Service 0x09 PID 0x02 - VIN, absichtlich als Multi-Frame-Antwort
// (17 Zeichen VIN + 1 Fuellbyte -> testet First/Consecutive Frame)
// ---------------------------------------------------------------------
void handleVin() {
    const char *vin = "WVWZZZ1KZAW123456"; // 18 Byte inkl. Fuellbyte vorne
    const uint8_t totalLen = 1 + 1 + 18;   // ServiceEcho + PID + 1 Fuellbyte + 17 VIN-Zeichen -> vereinfachte Laenge
    (void) totalLen;

    uint8_t payload[64] = {0};
    payload[0] = 0x49; // Service 0x09 + 0x40
    payload[1] = 0x02; // PID Echo
    payload[2] = 0x01; // "Anzahl Datensaetze" laut Norm, hier vereinfacht fix 1
    memcpy(&payload[3], vin, strlen(vin));
    const uint8_t fullLen = 3 + strlen(vin);

    // First Frame: PCI = 0x1 + oberes Nibble der Laenge, dann unteres Byte
    uint8_t ff[8];
    ff[0] = 0x10 | ((fullLen >> 8) & 0x0F);
    ff[1] = fullLen & 0xFF;
    memcpy(&ff[2], payload, 6);
    sendFrame(OBD_RESPONSE_ID, ff, 8);
    Serial.println("-> VIN First Frame gesendet, warte auf Flow Control...");

    if (!waitForFlowControl(500)) {
        Serial.println("   Kein Flow-Control-Frame erhalten - sende trotzdem (Testmodus)");
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
        delay(10); // simulierte Separation Time
    }
    Serial.println("-> VIN komplett gesendet");
}

// ---------------------------------------------------------------------
// Service 0x03 - gespeicherte DTCs (2 Testfehlercodes: P0301, P0420)
// ---------------------------------------------------------------------
void handleMode03() {
    uint8_t resp[8] = {0x05, 0x43, 0x02, 0x03, 0x01, 0x04, 0x20, 0x00};
    // Byte0=PCI(5 Datenbytes) Byte1=0x43 Byte2=AnzahlDTCs(2)
    // Byte3-4 = P0301 (0x0301) Byte5-6 = P0420 (0x0420)
    sendFrame(OBD_RESPONSE_ID, resp, 8);
    Serial.println("-> Mode03 (DTCs) beantwortet: P0301, P0420");
}

// ---------------------------------------------------------------------
// Service 0x04 - DTCs loeschen (immer erfolgreich quittieren)
// ---------------------------------------------------------------------
void handleMode04() {
    uint8_t resp[8] = {0x01, 0x44, 0, 0, 0, 0, 0, 0};
    sendFrame(OBD_RESPONSE_ID, resp, 2);
    Serial.println("-> Mode04 (DTCs loeschen) quittiert");
}

// ---------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Fake-ECU startet ===");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("FEHLER: twai_driver_install fehlgeschlagen");
        while (true) delay(1000);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("FEHLER: twai_start fehlgeschlagen");
        while (true) delay(1000);
    }

    Serial.println("CAN-Bus bereit, warte auf OBD2-Anfragen (0x7DF/0x7E0)...");
}

unsigned long lastSimUpdate = 0;

void loop() {
    // Simulierte Werte alle 500ms weiterlaufen lassen
    if (millis() - lastSimUpdate > 500) {
        updateSimValues();
        lastSimUpdate = millis();
    }

    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(50)) != ESP_OK) {
        return;
    }

    // Nur auf funktionale (0x7DF) oder direkt an uns gerichtete (0x7E0)
    // Anfragen reagieren - Flow-Control-Frames (0x30) werden separat in
    // waitForFlowControl() behandelt und hier ignoriert.
    if (msg.identifier != OBD_REQUEST_ID && msg.identifier != OBD_MY_REQUEST_ID) {
        return;
    }
    if (msg.data_length_code < 3) {
        return;
    }
    if ((msg.data[0] & 0xF0) == 0x30) {
        return; // Flow-Control, gehoert nicht hierher
    }

    uint8_t service = msg.data[1];
    uint8_t pid = msg.data[2];

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
            Serial.printf("Unbekannter Service 0x%02X ignoriert\n", service);
            break;
    }
}
