#include <ArduinoJson.h>
#include <DNSServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TimeLib.h>

#include "Config.h"

#ifndef MANUAL
#include "ESPAsyncWiFiManager.h"
#endif

#ifdef BME680
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>
#endif

#ifdef DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#ifdef DHTSensor
#include <DHT.h>
#endif

#ifdef PM1006K
#include "SerialCom.h"
#endif
#include "Types.h"
#include "devicetime.h"

#ifdef PM1006K
particleSensorState_t state;
#endif

#ifdef BME680
Adafruit_BME680 bme;
bmeSensorState_t bmestate;
#endif

#ifdef DS18B20
// GPIO where the DS18B20 is connected to
const int oneWireBus = 5;          
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature ds18b20(&oneWire);
ds18b20SensorState_t ds18b20state;
#endif

#ifdef DHTSensor
#define DHTPIN 5
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
dht22SensorState_t dht22state;
#endif

#ifdef USE_ADC
const int analogInPin = A0;
adcState_t adcstate;
#endif

#ifdef HWV10
const int LED_PIN = 12;
#endif
#ifdef HWV11
const int LED_PIN = 2;
#endif

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)
DNSServer dns;

// init Time
time_t timestamp = 0;

uint8_t mqttRetryCounter = 0;

#ifndef MANUAL
AsyncWiFiManager wifiManager(&server,&dns);
#endif
WiFiClient wifiClient;
PubSubClient mqttClient;
#include "update.h"

#ifndef MANUAL
AsyncWiFiManagerParameter custom_ntp_server("ntp", "ntp server", Config::ntp_server, sizeof(Config::ntp_server));
AsyncWiFiManagerParameter custom_mqtt_server("server", "mqtt server", Config::mqtt_server, sizeof(Config::mqtt_server));
AsyncWiFiManagerParameter custom_mqtt_user("user", "MQTT username", Config::username, sizeof(Config::username));
AsyncWiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", Config::password, sizeof(Config::password));
#endif

#ifdef PM1006K
uint32_t pm25PreviousMillis = 0;
const uint16_t pm25Interval = 1000;
#endif

uint32_t lastMqttConnectionAttempt = 0;
const uint16_t mqttConnectionInterval = 60000; // 1 minute = 60 seconds = 60000 milliseconds

uint32_t statusPublishPreviousMillis = 0;
const uint16_t statusPublishInterval = 5000; // 5 seconds = 5000 milliseconds
const uint16_t statusPublishDelay = 120000; // 2 minute = 120 seconds = 120000 milliseconds

uint32_t ledBlinkPreviousMillis = 0;
const uint16_t ledBlinkInterval = 500;
int ledState = LOW;

#ifdef BME680
uint32_t bmePreviousMillis = 0;
const uint16_t bmeInterval = 1000; // 1 seconds = 1000 milliseconds
#endif

#ifdef DS18B20
unsigned long ds18b20PreviousMillis = 0;
const long ds18b20Interval = 5000;
#endif

#ifdef DHTSensor
unsigned long dht22PreviousMillis = 0;
const long dht22Interval = 1000;
#endif

#ifdef USE_ADC
unsigned long adcPreviousMillis = 0;
const long adcInterval = 1000;
#endif

char identifier[24];
#define FIRMWARE_PREFIX "esp8266-open-vindriktning"
#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"
char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

#ifndef DISABLEHA
char MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[128];
#ifdef PM1006K
char MQTT_TOPIC_AUTOCONF_FAN_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PM25_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PM10_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PM100_SENSOR[128];
#endif
#if ( defined(BME680) ||  defined(DS18B20) || defined(DHTSensor) )
char MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR[128];
#endif
#ifdef BME680
char MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PRESSURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_GAS_SENSOR[128];
#endif
#ifdef DHTSensor
char MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR[128];
#endif
#ifdef USE_ADC
char MQTT_TOPIC_AUTOCONF_ADC_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_EV_SENSOR[128];
#endif
#endif

#ifdef PM1006K
constexpr static const uint8_t PIN_FAN = 13; // Fan
uint8_t FAN_STATUS = HIGH;
unsigned long fanPreviousMillis = 0;
const long fanOnInterval = 21000;
const long fanCycle = 30000;
#endif

bool shouldSaveConfig = false;

void saveConfigCallback() {
    shouldSaveConfig = true;
}

boolean isDoNotDisturb(int element) {
    for (int i = 0; i < DoNotDisturbSize; i++) {
        if (DoNotDisturbHour[i] == element) {
            return true;
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200);

#ifdef PM1006K
    SerialCom::setup();
    pinMode(PIN_FAN, OUTPUT);
    digitalWrite(PIN_FAN, FAN_STATUS);
#endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n");
    Serial.println("Hello from esp8266-open-vindriktning");
    Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
    Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
    Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
    Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

    delay(3000);

    snprintf(identifier, sizeof(identifier), "OPEN-VINDRIKTNING-%X", ESP.getChipId());
#ifndef MQTTTOPIC
    snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", FIRMWARE_PREFIX, identifier);
#else
    snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/status", Config::mqtt_topic);
    snprintf(MQTT_TOPIC_STATE, 127, "%s/state", Config::mqtt_topic);
    snprintf(MQTT_TOPIC_COMMAND, 127, "%s/command", Config::mqtt_topic);
#endif

#ifndef DISABLEHA
#ifdef PM1006K
    snprintf(MQTT_TOPIC_AUTOCONF_FAN_SENSOR, 127, "homeassistant/sensor/%s/%s_fan/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_PM25_SENSOR, 127, "homeassistant/sensor/%s/%s_pm25/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_PM10_SENSOR, 127, "homeassistant/sensor/%s/%s_pm10/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_PM100_SENSOR, 127, "homeassistant/sensor/%s/%s_pm100/config", FIRMWARE_PREFIX, identifier);
#endif
#if ( defined(BME680) ||  defined(DS18B20) || defined(DHTSensor) )
    snprintf(MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR, 127, "homeassistant/sensor/%s/%s_temperature/config", FIRMWARE_PREFIX, identifier);
#endif
#if ( defined(BME680) || defined(DHTSensor) )
    snprintf(MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR, 127, "homeassistant/sensor/%s/%s_humidity/config", FIRMWARE_PREFIX, identifier);
#endif
#ifdef BME680
    snprintf(MQTT_TOPIC_AUTOCONF_PRESSURE_SENSOR, 127, "homeassistant/sensor/%s/%s_pressure/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_GAS_SENSOR, 127, "homeassistant/sensor/%s/%s_gas/config", FIRMWARE_PREFIX, identifier);
#endif
#ifdef USE_ADC
    snprintf(MQTT_TOPIC_AUTOCONF_ADC_SENSOR, 127, "homeassistant/sensor/%s/%s_adc/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_EV_SENSOR, 127, "homeassistant/sensor/%s/%s_ev/config", FIRMWARE_PREFIX, identifier);
#endif
    snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);
#endif

    WiFi.hostname(identifier);

#ifndef MANUAL
    Config::load();
#endif

    setupWifi();
    setup_ntp();
    setupOTA();
    mqttClient.setServer(Config::mqtt_server, 1883);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    Serial.printf("Hostname: %s\n", identifier);
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

#ifdef PM1006K
    Serial.println("-- Current GPIO Configuration --");
    Serial.printf("PIN_UART_RX: %d\n", SerialCom::PIN_UART_RX);
    Serial.printf("PIN_UART_TX: %d\n", SerialCom::PIN_UART_TX);
#endif

    mqttReconnect();

#ifdef BME680
    Serial.println("Initializing BME680");
    if (!bme.begin()) {
        Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
        while (1);
    }
    
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
#endif

#ifdef DHTSensor
    Serial.println("Initializing DHT Sensor");
    dht.begin();
#endif
}

void loop() {
    if (!Update.isRunning()) {
        if(shouldReboot){
            Serial.println("Rebooting...");
            delay(100);
            ESP.restart();
        }
#ifdef PM1006K
        SerialCom::handleUart(state);
#endif
        mqttClient.loop();

        const uint32_t currentMillis = millis();
        if (timeStatus() != timeNotSet) {
            if (now() != timestamp) {
                timestamp = now();
            }
        }

#ifdef PM1006K
        if (currentMillis - pm25PreviousMillis >= pm25Interval) {
            pm25PreviousMillis = currentMillis;
            SerialCom::sendRequest();
        }
#endif

#ifndef FANALWAYSON
        if (currentMillis - fanPreviousMillis >= fanCycle) {
            FAN_STATUS = HIGH;
            digitalWrite(PIN_FAN, FAN_STATUS);
            fanPreviousMillis = currentMillis;
        }
        else{
            if (currentMillis - fanPreviousMillis >= fanOnInterval) {
                FAN_STATUS = LOW;
                digitalWrite(PIN_FAN, FAN_STATUS);
            }
        }
#endif

#ifdef BME680
        if (currentMillis - bmePreviousMillis >= bmeInterval) {
            bmePreviousMillis = currentMillis;
            // Tell BME680 to begin measurement.
            unsigned long endTime = bme.beginReading();
            if (endTime == 0) {
                Serial.println(F("Failed to begin reading :("));
                return;
            }
            delay(50); // This represents parallel work.
            if (!bme.endReading()) {
                Serial.println(F("Failed to complete reading :("));
                return;
            }
            // Read temperature
            float newT = bme.temperature;
            if (isnan(newT)) {
                Serial.println("Failed to read temperature from BME sensor!");
            }
            else {
                bmestate.t = newT;
                Serial.printf("temperature: %.2f °C\n", bmestate.t);
            }
            // Read humidity
            float newH = bme.humidity;
            if (isnan(newH)) {
                Serial.println("Failed to read humidity from BME sensor!");
            }
            else {
                bmestate.h = newH;
                Serial.printf("humidity: %.2f %%\n", bmestate.h);
            }
            // Read pressure
            float newP = bme.pressure / 100.0;
            if (isnan(newP)) {
                Serial.println("Failed to read pressure from BME sensor!");
            }
            else {
                bmestate.p = newP;
                Serial.printf("pressure: %.2f hPa\n", bmestate.p);
            }
            // Read gas_resistance
            float newG = bme.gas_resistance;
            if (isnan(newG)) {
                Serial.println("Failed to read gas resistance from BME sensor!");
            }
            else {
                bmestate.g = newG;
                Serial.printf("gas resistance: %.2f Ohms\n", bmestate.g);
            }
        }
#endif      

#ifdef DS18B20
        if (currentMillis - ds18b20PreviousMillis >= ds18b20Interval) {
            // Save the last time a new reading was published
            ds18b20PreviousMillis = currentMillis;
            // New temperature readings
            ds18b20.requestTemperatures(); 
            // Temperature in Celsius degrees
            float newT = ds18b20.getTempCByIndex(0);
            // Temperature in Fahrenheit degrees
            //temp = ds18b20.getTempFByIndex(0);

            if (isnan(newT)) {
                Serial.println("Failed to read temperature from DS18B20 sensor!");
            }
            else {
                ds18b20state.t = newT;
                Serial.printf("temperature: %.2f °C\n", ds18b20state.t);
            }
        }
#endif

#ifdef DHTSensor
        if (currentMillis - dht22PreviousMillis >= dht22Interval) {
            // Save the last time a new reading was published
            dht22PreviousMillis = currentMillis;
            // Read temperature
            float newT = dht.readTemperature();
            if (isnan(newT)) {
                Serial.println("Failed to read temperature from DHT22 sensor!");
            }
            else {
                dht22state.t = newT;
                Serial.printf("temperature: %.2f °C\n", dht22state.t);
            }
            // Read humidity
            float newH = dht.readHumidity();
            if (isnan(newH)) {
                Serial.println("Failed to read humidity from DHT22 sensor!");
            }
            else {
                dht22state.h = newH;
                Serial.printf("humidity: %.2f %%\n", dht22state.h);
            }
        }
#endif

#ifdef USE_ADC
        if (currentMillis - adcPreviousMillis >= adcInterval) {
            // Save the last time a new reading was published
            adcPreviousMillis = currentMillis;
            float newV = analogRead(analogInPin);

            if (isnan(newV)) {
                Serial.println("Failed to value from ADC PIN!");
            }
            else {
                adcstate.v = newV;
                adcstate.ev = max(newV * 13.0 - 387.0, 0.0);
                Serial.printf("ADC value: %d\n", adcstate.v);
                Serial.printf("Estimated illuminance: %f\n", adcstate.ev);
            }
        }
#endif

        if (currentMillis - statusPublishPreviousMillis >= statusPublishInterval && currentMillis >= statusPublishDelay) {
            statusPublishPreviousMillis = currentMillis;
#ifdef PM1006K
            if (state.valid) {
                printf("Publish state\n");
                publishState();
            }
#else
            publishState();
#endif
        }

        if (!mqttClient.connected() && currentMillis - lastMqttConnectionAttempt >= mqttConnectionInterval) {
            lastMqttConnectionAttempt = currentMillis;
            printf("Reconnect mqtt\n");
            mqttReconnect();
        }


        if (!mqttClient.connected()){
            if (currentMillis - ledBlinkPreviousMillis >= ledBlinkInterval) {
                // Save the last time a new reading was published
                ledBlinkPreviousMillis = currentMillis;
                if (ledState == LOW) {
                    ledState = HIGH;
                } else {
                    ledState = LOW;
                }
            }
        }
        else {
            ledState = LOW;
        }


        if (isDoNotDisturb(hour(timestamp))) {
            digitalWrite(LED_PIN, HIGH);
        }
        else {
            digitalWrite(LED_PIN, ledState);
        }
    }
}

void setupWifi() {
#ifndef MANUAL
    wifiManager.setDebugOutput(true);
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    wifiManager.addParameter(&custom_ntp_server);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    WiFi.hostname(identifier);
    wifiManager.autoConnect(identifier);
    mqttClient.setClient(wifiClient);

    strcpy(Config::ntp_server, custom_ntp_server.getValue());
    strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
    strcpy(Config::username, custom_mqtt_user.getValue());
    strcpy(Config::password, custom_mqtt_pass.getValue());

    if (shouldSaveConfig) {
        Config::save();
    } else {
        // For some reason, the read values get overwritten in this function
        // To combat this, we just reload the config
        // This is most likely a logic error which could be fixed otherwise
        Config::load();
    }
#else
    WiFi.mode(WIFI_STA);
    WiFi.hostname(identifier);
    WiFi.begin(Config::wifi_ssid, Config::wifi_password);
    Serial.println("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println(".");
    }
    // Print ESP8266 Local IP Address
    Serial.printf("MAC: %s\n",WiFi.macAddress().c_str());
    Serial.printf("IP: %s\n",WiFi.localIP().toString().c_str());
    Serial.printf("Hostname: %s\n", WiFi.getHostname());

    // Print ESP8266 Local IP Address
    Serial.println(WiFi.localIP());
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    mqttClient.setClient(wifiClient);
#endif
}

#ifndef MANUAL
void resetWifiSettingsAndReboot() {
    wifiManager.resetSettings();
    delay(3000);
    ESP.restart();
}
#endif

void mqttReconnect() {
    Serial.printf("Connecting to MQTT Server %s\n", Config::mqtt_server);
    if (!(Config::username == "")) {
        Serial.printf("MQTT Server Username %s\n", Config::username);
    }
    if (!(Config::password == "")) {
        Serial.printf("MQTT Server Password%s\n", Config::password);
    }
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (mqttClient.connect(identifier, Config::username, Config::password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE)) {
            mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
#ifndef DISABLEHA
            publishAutoConfig();
#endif
            // Make sure to subscribe after polling the status so that we never execute commands with the default data
            mqttClient.subscribe(MQTT_TOPIC_COMMAND);
            break;
        }
        delay(5000);
    }
}

bool isMqttConnected() {
    return mqttClient.connected();
}

void publishState() {
    DynamicJsonDocument wifiJson(192);
    DynamicJsonDocument stateJson(604);
    char payload[256];

    wifiJson["ssid"] = WiFi.SSID();
    wifiJson["ip"] = WiFi.localIP().toString();
    wifiJson["rssi"] = WiFi.RSSI();

#ifdef PM1006K
    stateJson["pm25"] = state.pm25;
    stateJson["pm10"] = state.pm10;
    stateJson["pm100"] = state.pm100;
#endif

#ifdef BME680
    stateJson["temperature"] = bmestate.t;
    stateJson["humidity"] = bmestate.h;
    stateJson["pressure"] = bmestate.p;
    stateJson["gas"] = bmestate.g;
#endif

#ifdef DS18B20
    stateJson["temperature"] = ds18b20state.t;
#endif

#ifdef DHTSensor
    stateJson["temperature"] = dht22state.t;
    stateJson["humidity"] = dht22state.h;
#endif

#ifdef USE_ADC
    stateJson["adc"] = adcstate.v;
    stateJson["ev"] = adcstate.ev;
#endif

#ifdef PM1006K
    stateJson["fan"] = FAN_STATUS;
#endif

    stateJson["wifi"] = wifiJson.as<JsonObject>();

    stateJson["timestamp"] = timestamp;

    serializeJson(stateJson, payload);
    mqttClient.publish(&MQTT_TOPIC_STATE[0], &payload[0], true);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) { }

#ifndef DISABLEHA
void publishAutoConfig() {
    char mqttPayload[2048];
    DynamicJsonDocument device(256);
    DynamicJsonDocument autoconfPayload(1024);
    StaticJsonDocument<64> identifiersDoc;
    JsonArray identifiers = identifiersDoc.to<JsonArray>();

    identifiers.add(identifier);

    device["identifiers"] = identifiers;
    device["manufacturer"] = "Ikea";
    device["model"] = "VINDRIKTNING";
    device["name"] = identifier;
    device["sw_version"] = "2022.05.08";

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" WiFi");
    autoconfPayload["value_template"] = "{{value_json.wifi.rssi}}";
    autoconfPayload["unique_id"] = identifier + String("_wifi");
    autoconfPayload["unit_of_measurement"] = "dBm";
    autoconfPayload["json_attributes_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["json_attributes_template"] = "{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": \"{{value_json.wifi.ip}}\"}";
    autoconfPayload["icon"] = "mdi:wifi";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

#ifdef PM1006K
    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" FAN status");
    autoconfPayload["unit_of_measurement"] = "";
    autoconfPayload["value_template"] = "{{value_json.fan}}";
    autoconfPayload["unique_id"] = identifier + String("_fan");
    autoconfPayload["icon"] = "mdi:fan";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_FAN_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" PM 2.5");
    autoconfPayload["unit_of_measurement"] = "μg/m³";
    autoconfPayload["value_template"] = "{{value_json.pm25}}";
    autoconfPayload["unique_id"] = identifier + String("_pm25");
    autoconfPayload["icon"] = "mdi:air-filter";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PM25_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" PM 1.0");
    autoconfPayload["unit_of_measurement"] = "μg/m³";
    autoconfPayload["value_template"] = "{{value_json.pm10}}";
    autoconfPayload["unique_id"] = identifier + String("_pm10");
    autoconfPayload["icon"] = "mdi:air-filter";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PM10_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" PM 10.0");
    autoconfPayload["unit_of_measurement"] = "μg/m³";
    autoconfPayload["value_template"] = "{{value_json.pm100}}";
    autoconfPayload["unique_id"] = identifier + String("_pm100");
    autoconfPayload["icon"] = "mdi:air-filter";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PM100_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
#endif

#if ( defined(BME680) ||  defined(DS18B20) || defined(DHTSensor) )
    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Temperature");
    autoconfPayload["unit_of_measurement"] = "°C";
    autoconfPayload["value_template"] = "{{value_json.temperature}}";
    autoconfPayload["unique_id"] = identifier + String("_temperature");
    autoconfPayload["icon"] = "mdi:thermometer";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
#endif

#if ( defined(BME680) || defined(DHTSensor) )
    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Humidity");
    autoconfPayload["unit_of_measurement"] = "%";
    autoconfPayload["value_template"] = "{{value_json.humidity}}";
    autoconfPayload["unique_id"] = identifier + String("_humidity");
    autoconfPayload["icon"] = "mdi:water-percent";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
#endif

#ifdef BME680
    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Pressure");
    autoconfPayload["unit_of_measurement"] = "hPa";
    autoconfPayload["value_template"] = "{{value_json.pressure}}";
    autoconfPayload["unique_id"] = identifier + String("_pressure");
    autoconfPayload["icon"] = "mdi:car-brake-low-pressure";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PRESSURE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Gas resistance");
    autoconfPayload["unit_of_measurement"] = "Ohms";
    autoconfPayload["value_template"] = "{{value_json.gas}}";
    autoconfPayload["unique_id"] = identifier + String("_gas");
    autoconfPayload["icon"] = "mdi:gas-cylinder";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_GAS_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
#endif

#ifdef USE_ADC
    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" ADC Value");
    autoconfPayload["unit_of_measurement"] = "";
    autoconfPayload["value_template"] = "{{value_json.adc}}";
    autoconfPayload["unique_id"] = identifier + String("_adc");
    autoconfPayload["icon"] = "mdi:sine-wave";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_ADC_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Estimated illuminance");
    autoconfPayload["unit_of_measurement"] = "lx";
    autoconfPayload["value_template"] = "{{value_json.ev}}";
    autoconfPayload["unique_id"] = identifier + String("_ev");
    autoconfPayload["icon"] = "mdi:lightbulb-on";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_EV_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
#endif
}
#endif
