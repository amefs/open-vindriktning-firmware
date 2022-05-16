# open-vindriktning-firmware

This repository contains an firmware, which designed for [open vindriktning hardware project](https://github.com/OtakuNekoP/open-vindriktning). 

## Key Features

- Temperature sensors support
- MQTT support
- OTA function via web
- Home Assistant Autodiscovery support

[![open-vindriktning-ha.md.png](https://cdn-image.wenzelu.com/2022/05/08/open-vindriktning-ha.md.png)](https://image.wenzelu.com/image/782K)

- WIFI configuration portal by WiFiManager library

  [![open-vindriktning-wifi-portal.md.png](https://cdn-image.wenzelu.com/2022/05/08/open-vindriktning-wifi-portal.md.png)](https://image.wenzelu.com/image/7Mu0)

## Requirements

To build this firmware, you will need following libraries:

- Arduino IDE
- ArduinoJson
- TimeLib
- ESPAsyncTCP

For extend temperature sensors, you will need:

- Adafruit Sensor
- Adafruit BME680 Library (for BME 680)
- DallasTemperature (for DS18B20)
- DHT sensor library (for DHR22)

## Configuration

You can find all settings in `manual_config.hpp` , you can enable functions by uncomment following macros:

- `#define PM1006K`  particle sensor

- `#define BME680` BME680 sensor

- `#define DS18B20` ds18b20 sensor

- `#define DHTSensor` DHT22 sensor

- `#define MANUAL` manually setup WIFI with following information:

  ```
  const char* wifi_ssid = "";
  const char* wifi_password = "";
  const char* ntp_server = "";
  const char* mqtt_server = "";
  const char* username = "";
  const char* password = "";
  ```

- `#define DISABLEHA` disable homeassistant function.

- `#define MQTTTOPIC` set a custom MQTT Topic.

- `#define FANALWAYSON` keep the fan always on, or it will run for 21 seconds then sleep for 9 seconds.

- `#define USE_ADC` enable ADC on ESP8266.

## Webpage

- We provide an OTA webpage, you can upgrade the firmware via http://DEVICEIP/update.
- WIFI settings can be reset with http://DEVICEIP/reset.
- The device can be software restarted via http://DEVICEIP/reset.

## Related Works

- https://github.com/Hypfer/esp8266-vindriktning-particle-sensor
- https://github.com/alanswx/ESPAsyncWiFiManager
- https://github.com/arendst/Tasmota/blob/62983899b626c0c81a998de381471b90d169d648/tasmota/xsns_91_vindriktning.ino
