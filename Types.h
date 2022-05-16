#ifndef Types_h
#define Types_h

#ifdef PM1006K
struct particleSensorState_t {
    uint16_t pm25 = 0;
    uint16_t pm10 = 0;
    uint16_t pm100 = 0;
    boolean valid = false;
};
#endif

# ifdef     BME680
struct bmeSensorState_t {
    float t = 0.0;
    float h = 0.0;
    float p = 0.0;
    float g = 0.0;
};
# endif

# ifdef     DS18B20
struct ds18b20SensorState_t {
    float t = 0.0;
};
# endif

# ifdef     DHTSensor
struct dht22SensorState_t {
    float t = 0.0;
    float h = 0.0;
};
# endif

# ifdef     USE_ADC
struct adcState_t {
    uint16_t v = 0;
    float ev = 0.0;
};
# endif

#endif