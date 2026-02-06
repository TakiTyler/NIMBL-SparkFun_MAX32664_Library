#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "sensor-library.h"

// setting the reset & mfio pins
SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub(uint8_t* resetPort, uint8_t* resetOut, uint16_t resetBit, uint8_t* mfioPort, uint8_t* mfioOut, uint16_t mfioBit, uint8_t address = 0x55){

    // set the necessary pins
    _resetPort = resetPort;
    _resetOut = resetOut;
//    _resetOut = resetPort + 2; // out is always +2 bits offset from port (disabled for now)
    _resetBit = resetBit;

    _mfioPort = mfioPort;
    _mfioOut = mfioOut;
//    _mfioOut = mfioPort + 2; // out is always +2 bits offset from port
    _mfioBit = mfioBit;

    _address = address;

    if(_resetPort != nullptr){
        *_resetPort |= _resetBit; // set direction to output
        *_resetOut |= _resetBit; // set pin high
    }

    if(_mfioPort != nullptr){
        *_mfioPort |= _mfioBit; // set direction to output
        *_mfioOut |= _mfioBit; // set pin high
    }
}

uint8_t SparkFun_Bio_Sensor_Hub::begin(){

    // validate that pins ARE assigned
    if(_resetPort == nullptr || _mfioPort == nullptr){
        return 0xFF; // MAX32664 general error code
    }

    // pull mfio high in reset for 10ms
    *_mfioOut |= _mfioBit;      // write mfio high
    *_resetOut &= ~_resetBit;   // write reset low
    __delay_cycles(10000);      // 10ms delay @ 1MHz

    *_resetOut |= _resetBit;    // write reset high
    __delay_cycles(1000000);     // 1000ms delay @ 1MHz

    // set mfio to input
    // pulling enable is always offset by 0x06 (P(x) is at 0x06, P(y) is at 0x07 (x = odd #, y = even #))
    *_mfioPort &= ~_mfioBit;
    *(_mfioPort + 0x06) |= _mfioBit;
    *_mfioOut |= _mfioBit;

    // verify MAX32664 returned 0x00 (READ_DEVICE_MODE = 0x02)
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00);
    return responseByte;
}

// same as begin, but inverting some pins being held low/high
uint8_t SparkFun_Bio_Sensor_Hub::beginBootloader(){

    // validate that pins ARE assigned
    if(_resetPort == nullptr || _mfioPort == nullptr){
        return 0xFF; // MAX32664 general error code
    }

    // pull mfio high in reset for 10ms
    *_mfioOut &= ~_mfioBit;     // write mfio low
    *_resetOut &= ~_resetBit;   // write reset low
    __delay_cycles(10000);      // 10ms delay @ 1MHz

    *_resetOut |= _resetBit;    // write reset high
    __delay_cycles(50000);     // 50ms delay @ 1MHz for the bootloader

    // set mfio to output
    *_mfioOut |= _mfioBit;

    // verify MAX32664 returned 0x08 (READ_DEVICE_MODE = 0x02)
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00);
    return responseByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::readSensorHubStatus(){
    uint8_t responseByte = readByte(HUB_STATUS, 0x00);
    return responseByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte){

    uint8_t returnByte = 0xFF;
    uint8_t statusByte = 0x00;

    // write the family & index byte
    UCB0I2CSA = _address;           // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;    // transmitter mode & start condition

    while(!(UCB0IFG & UCTXIFG));    // wait for tx buffer
    UCB0TXBUF = _familyByte;        // send family byte

    while(!(UCB0IFG & UCTXIFG));    // wait for tx buffer
    UCB0TXBUF = _indexByte;         // send family byte

    while(UCB0CTLW0 & UCTXSTT);     // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP;           // generate stop condition
    while(UCB0CTLW0 & UCTXSTP);     // wait for stop to finish

    __delay_cycles(2000);           // 2ms delay @ 1MHz

    // read back status bit
    UCB0CTLW0 &= ~UCTR;             // receiver mode
    UCB0CTLW0 |= UCTXSTT;           // start condition

    // read status byte
    while(!(UCB0IFG & UCTXIFG));    // wait until interrupt
    statusByte = UCB0RXBUF;

    // read return byte
    UCB0CTLW0 |= UCTXSTP;           // set stop before reading the last byte
    while(!(UCB0IFG & UCTXIFG));    // wait until interrupt
    returnByte = UCB0RXBUF;

    while(UCB0CTLW0 & UCTXSTP);     // wait for stop to finish

    // check if the statusByte isn't 0x00 to return the error

    if(statusByte != 0x00){
        return statusByte;
    }

    return returnByte;


}

// creating UART for debugging since printing doesn't seem to work
void initUART(){
    // configure uart pins, 1.4 = TX & 1.5 = RX
    P1SEL0 |= BIT4 | BIT5;
//    P1SEL0 &= ~(BIT4 | BIT5);

    // configure uart for 9600 baud
    UCA0CTLW0 |= UCSWRST; // eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // choose smclk

    UCA0BR0 = 104;
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x1100;

    UCA0CTLW0 &= ~UCSWRST; // release from reset
}

void uart_write_char(char c){
    while(!(UCA0IFG & UCTXIFG)); // wait for tx buffer to be empty
    UCA0TXBUF = c;
}

void uart_write_string(const char* str){
    while(*str){
        uart_write_char(*str++);
    }
}

void uart_write_int(int num){
    char buf[10];
    int i = 0;
    if(num == 0){
        uart_write_char('0');
        return;
    }
    while(num > 0 && i < 10){
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while(--i >= 0){
        uart_write_char(buf[i]);
    }
}

int main(void){

    // stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // disable gpio
    PM5CTL0 &= ~LOCKLPM5;

    P1DIR |= BIT0;   // Set P1.0 to output direction
    P1OUT |= BIT0;   // Turn on the LED

    initUART();

    bioData mySensorData;
    version firmwareVersion;

    mySensorData.heartRate = 72;
    mySensorData.confidence = 98;
    mySensorData.oxygen = 99;

    firmwareVersion.major = 1;
    firmwareVersion.minor = 0;
    firmwareVersion.revision = 3;

    uart_write_string("--- Sensor Test ---\n");

    uart_write_string("Heart Rate: ");
    uart_write_int((int) mySensorData.heartRate);
    uart_write_string(" bpm\n");

    uart_write_string("Oxygen: ");
    uart_write_int((int) mySensorData.oxygen);
    uart_write_string("%%\n");

    uart_write_string("Firmware: v");
    uart_write_int((int) firmwareVersion.major);
    uart_write_string(".");
    uart_write_int((int) firmwareVersion.minor);
    uart_write_string(".");
    uart_write_int((int) firmwareVersion.revision);
    uart_write_string("\n");
    P1OUT &= ~BIT0;   // Turn on the LED

//    printf("--- Sensor Test ---\n");
//    printf("Heart Rate: %d bpm\n", (int) mySensorData.heartRate);
//    printf("Oxygen: %d%%\n", (int) mySensorData.oxygen);
//    printf("Firmware: v%d.%d.%d\n", (int) firmwareVersion.major, (int) firmwareVersion.minor, (int) firmwareVersion.revision);


    while(1);
}
