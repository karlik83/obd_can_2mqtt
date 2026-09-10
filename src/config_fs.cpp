/*
 * config_fs.cpp - see config_fs.h
 */
#include "config_fs.h"

#include <Arduino.h>
#include <LittleFS.h>

#include "settings.h" // SETTINGS_FILE
#include "obd.h"      // STATES_FILE

// Own LittleFS instance bound to the "cfg" partition label.
static fs::LittleFSFS CfgFS;
static bool cfgMounted = false;

fs::FS &configFs() {
    return cfgMounted ? static_cast<fs::FS &>(CfgFS) : static_cast<fs::FS &>(LittleFS);
}

static void migrateFile(const char *path) {
    if (CfgFS.exists(path) || !LittleFS.exists(path)) {
        return;
    }

    File src = LittleFS.open(path, FILE_READ);
    File dst = CfgFS.open(path, FILE_WRITE);
    if (src && dst) {
        uint8_t buf[512];
        size_t n;
        while ((n = src.read(buf, sizeof(buf))) > 0) {
            dst.write(buf, n);
        }
        Serial.printf("config: migrated %s from spiffs to cfg partition\n", path);
    }
    if (src) {
        src.close();
    }
    if (dst) {
        dst.close();
    }
}

void initConfigFs() {
    // 4th arg is the partition label; the UI partition keeps the "spiffs" label
    // and is the one `uploadfs` targets.
    cfgMounted = CfgFS.begin(true, "/cfg", 5, "cfg");
    if (!cfgMounted) {
        Serial.println("config: no 'cfg' partition - using LittleFS");
        return;
    }

    Serial.println("config: 'cfg' partition mounted");
    migrateFile(SETTINGS_FILE);
    migrateFile(STATES_FILE);
}
