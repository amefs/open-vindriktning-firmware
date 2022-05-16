#ifndef Config_h
#define Config_h

#include "manual_config.hpp"
#include <ArduinoJson.h>
#include <LittleFS.h>

namespace Config {
#ifndef MANUAL
    char ntp_server[80] = "pool.ntp.org";
    char mqtt_server[80] = "example.tld";

    char username[24] = "";
    char password[24] = "";

    void save() {
        DynamicJsonDocument json(512);
        json["ntp_server"] = ntp_server;
        json["mqtt_server"] = mqtt_server;
        json["username"] = username;
        json["password"] = password;

        File configFile = LittleFS.open("/config.json", "w");
        if (!configFile) {
            return;
        }

        serializeJson(json, configFile);
        configFile.close();
    }

    void load() {
        if (LittleFS.begin()) {

            if (LittleFS.exists("/config.json")) {
                File configFile = LittleFS.open("/config.json", "r");

                if (configFile) {
                    const size_t size = configFile.size();
                    std::unique_ptr<char[]> buf(new char[size]);

                    configFile.readBytes(buf.get(), size);
                    DynamicJsonDocument json(512);

                    if (DeserializationError::Ok == deserializeJson(json, buf.get())) {
                        strcpy(ntp_server, json["ntp_server"]);
                        strcpy(mqtt_server, json["mqtt_server"]);
                        strcpy(username, json["username"]);
                        strcpy(password, json["password"]);
                    }
                }
            }
        }
    }
#endif
} // namespace Config

#endif
