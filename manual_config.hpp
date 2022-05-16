#include "Config.h"

// Hardware version
//#define HWV10
 #define HWV11

// Sensor variant
#define PM1006K

//#define BME680
//#define DS18B20
//#define DHTSensor

// Manual setup
//#define MANUAL

// disable homeassistant
//#define DISABLEHA

// custom MQTT Topic
//#define MQTTTOPIC

// keep fan always on
#define FANALWAYSON

// enable ADC
#define USE_ADC

// Do not disturb hours 0-23
const int DoNotDisturbHour[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 22, 23};
const int DoNotDisturbSize = std::size(DoNotDisturbHour);


namespace Config {
#ifdef MANUAL
    // Manual setup here
    const char* wifi_ssid = "";
    const char* wifi_password = "";
    const char* ntp_server = "";
    const char* mqtt_server = "";
    const char* username = "";
    const char* password = "";
#endif

#ifdef MQTTTOPIC
    const char* mqtt_topic = "";
#endif

}


