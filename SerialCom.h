#ifndef SerialCom_h
#define SerialCom_h

#include <SoftwareSerial.h>

#include "Types.h"

namespace SerialCom {
    constexpr static const uint8_t PIN_UART_RX = 14; // receive sensor response
#ifdef HWV10
    constexpr static const uint8_t PIN_UART_TX = 2; // send request
#endif
#ifdef HWV11
    constexpr static const uint8_t PIN_UART_TX = 12; // send request
#endif
    byte init_request[] = {0x11, 0x03, 0x0c, 0x02, 0x1e, 0xc0};
    byte request[] = {0x11, 0x02, 0x0b, 0x01, 0xe1};

    SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

    uint8_t serialRxBuf[255];
    uint8_t rxBufIdx = 0;

    void sendRequest() {
        Serial.println("Request sent...");
        sensorSerial.write(request, sizeof(request));
    }

    void setup() {
        sensorSerial.begin(9600);
        sensorSerial.write(init_request, sizeof(request));
    }

    void clearRxBuf() {
        // Clear everything for the next message
        memset(serialRxBuf, 0, sizeof(serialRxBuf));
        rxBufIdx = 0;
    }

    void parseState(particleSensorState_t& state) {
        // sample data:
        //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
        // 16 11 0b 00 00 00 0c 00 00 03 cb 00 00 00 0c 01 00 00 00 e7
        //               |pm2_5|     |pm1_0|     |pm10 |        | CRC |
        const uint16_t pm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];
        Serial.printf("Received PM 2.5 reading: %d ug/m3\n", pm25);
        state.pm25 = pm25;

        const uint16_t pm10 = (serialRxBuf[9] << 8) | serialRxBuf[10];
        Serial.printf("Received PM 1.0 reading: %d ug/m3\n", pm10);
        state.pm10 = pm10;
        
        const uint16_t pm100 = (serialRxBuf[13] << 8) | serialRxBuf[14];
        Serial.printf("Received PM 10.0 reading: %d ug/m3\n", pm100);
        state.pm100 = pm100;

        state.valid = true;

        clearRxBuf();
    }

    bool isValidHeader() {
        bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

        if (!headerValid) {
            Serial.println("Received message with invalid header.");
        }

        return headerValid;
    }

    bool isValidChecksum() {
        uint8_t checksum = 0;

        for (uint8_t i = 0; i < 20; i++) {
            checksum += serialRxBuf[i];
        }

        if (checksum != 0) {
            Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\n", checksum);
        }

        return checksum == 0;
    }

    void handleUart(particleSensorState_t& state) {
        if (!sensorSerial.available()) {
            return;
        }

        Serial.print("Receiving:");
        while (sensorSerial.available()) {
            serialRxBuf[rxBufIdx++] = sensorSerial.read();
            Serial.print(".");

            // Without this delay, receiving data breaks for reasons that are beyond me
            delay(15);

            if (rxBufIdx >= 64) {
                clearRxBuf();
            }
        }
        Serial.println("Done.");

        if (isValidHeader() && isValidChecksum()) {
            parseState(state);
        } else {
            clearRxBuf();
        }
    }
} // namespace SerialCom

#endif
