#include <stdint.h>

#ifndef _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_
#define _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_

#define WRITE_FIFO_INPUT_BYTE 0x04
#define DISABLE 0x00
#define ENABLE 0x01
#define MODE_ONE 0x01
#define MODE_TWO 0x02
#define APP_MODE 0x00
#define BOOTLOADER_MODE 0x08
#define NO_WRITE 0x00
#define INCORR_PARAM 0xEE

#define CONFIGURATION_REGISTER 0x0A
#define PULSE_MASK 0xFC
#define READ_PULSE_MASK 0x03
#define SAMP_MASK 0xE3
#define READ_SAMP_MASK 0x1C
#define ADC_MASK 0x9F
#define READ_ADC_MASK 0x60

#define ENABLE_CMD_DELAY 45     // Milliseconds
#define ALGO_CMD_DELAY_SHORT 45 // Milliseconds
#define ALGO_CMD_DELAY_LONG 45  // Milliseconds
#define CMD_DELAY 2             // Milliseconds
#define MAXFAST_ARRAY_SIZE 6    // Number of bytes...
#define MAXFAST_EXTENDED_DATA 5
#define MAX30101_LED_ARRAY 12   // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT 0x00
#define READ_FORMAT 0x01            // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD 0x01    // Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

const uint8_t BIO_ADDRESS = 0x55;   // this should be fine...

// the struct that holds all the important data
struct bioData{
    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate; // Least Significant Bit = 0.1bpm
    uint8_t confidence; // 0 -> 100% LSB = 1%
    uint16_t oxygen;    // 0 -> 100% LSB = 1%
    uint8_t status;     // 0: Success, 1: Not Ready, 2: Object Detected, 3: Finger Detected
    float rValue;       // -- Algorithm Mode 2 vv
    uint8_t extStatus;  // --
    uint8_t reserveOne; // --
    uint8_t reserveTwo; // -- Algorithm Mode 2 ^^
};

struct version{
    // 3 bytes total
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
};

#endif // _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_
