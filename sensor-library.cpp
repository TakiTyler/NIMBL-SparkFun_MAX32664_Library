#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "sensor-library.h"

// setting the reset & mfio pins
SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub(uint8_t* resetPort, uint8_t* resetOut, uint16_t resetBit, uint8_t* mfioPort, uint8_t* mfioOut, uint16_t mfioBit, uint8_t address){

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

uint8_t SparkFun_Bio_Sensor_Hub::setOperatingMode(uint8_t selection){

    // parameter validation (must be 0x00, 0x02, or 0x08)
    if(selection != EXIT_BOOTLOADER && selection != SFE_BIO_RESET && selection != ENTER_BOOTLOADER){
        return INCORR_PARAM; // 0xEE
    }

    // send mode change
    uint8_t statusByte = writeByte(SET_DEVICE_MODE, 0x00, selection);
    if(statusByte != SFE_BIO_SUCCESS){
        return statusByte;
    }

    // wait for a verification
    __delay_cycles(100000);         // 100ms delay @ 1MHz

    // READ_DEVICE_MODE = 0x02
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00);
    return responseByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::configBpm(uint8_t mode){

    if(mode != MODE_ONE && mode != MODE_TWO){
        return 0xEE;
    }

    uint8_t status;

    status = setOutputMode(ALGO_DATA);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = setFifoThreshold(0x01);        // one sample before interrupt is fired
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = agcAlgoControl(ENABLE);        // one sample before interrupt is fired
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = max30101Control(ENABLE);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = maximFastAlgoControl(mode);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    __delay_cycles(1000000);                // 1 second delay @ 1MHz
    return SFE_BIO_SUCCESS;
}

uint8_t SparkFun_Bio_Sensor_Hub::configSensor(){
    uint8_t status;

    status = setOutputMode(SENSOR_DATA);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = setFifoThreshold(0x01);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = max30101Control(ENABLE);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = maximFastAlgoControl(MODE_ONE);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    __delay_cycles(1000000);                // 1 second delay @ 1MHz
    return SFE_BIO_SUCCESS;
}

uint8_t SparkFun_Bio_Sensor_Hub::configSensorBpm(uint8_t mode){

    if(mode != MODE_ONE && mode != MODE_TWO){
        return 0xEE;
    }

    uint8_t status;

    status = setOutputMode(SENSOR_AND_ALGORITHM);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = setFifoThreshold(0x01);        // one sample before interrupt is fired
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = max30101Control(ENABLE);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    status = maximFastAlgoControl(mode);
    if(status != SFE_BIO_SUCCESS){
        return status;
    }

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    __delay_cycles(1000000);                // 1 second delay @ 1MHz
    return SFE_BIO_SUCCESS;
}

bioData SparkFun_Bio_Sensor_Hub::readBpm(){
    bioData libBpm = {0};
    uint8_t status = readSensorHubStatus();

    // if we get a communication error
    if(status == 1){
        libBpm.heartRate = 0;
        libBpm.confidence = 0;
        libBpm.oxygen = 0;
        return libBpm;
    }

    numSamplesOutFifo();

    if(_userSelectedMode == MODE_ONE){
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, bpmArr);
        if(status != SFE_BIO_SUCCESS){
            return libBpm;
        }

        // heart rate formatting
        libBpm.heartRate = (uint16_t(bpmArr[0]) << 8) | bpmArr[1];
        libBpm.heartRate /= 10;

        // confidence formatting
        libBpm.confidence = bpmArr[2];

        // blood oxygen level formatting
        libBpm.oxygen = (uint16_t(bpmArr[3]) << 8) | bpmArr[4];
        libBpm.oxygen /= 10;

        // "machine state" - has a finger been detected?
        libBpm.status = bpmArr[5];

        return libBpm;
    }
    else if(_userSelectedMode == MODE_TWO){
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpmArrTwo);
        if(status != SFE_BIO_SUCCESS){
            return libBpm;
        }

        // heart rate formatting
        libBpm.heartRate = (uint16_t(bpmArrTwo[0]) << 8) | bpmArrTwo[1];
        libBpm.heartRate /= 10;

        // confidence formatting
        libBpm.confidence = bpmArrTwo[2];

        // blood oxygen level formatting
        libBpm.oxygen = (uint16_t(bpmArrTwo[3]) << 8) | bpmArrTwo[4];
        libBpm.oxygen /= 10;

        // "machine state" - has a finger been detected?
        libBpm.status = bpmArrTwo[5];

        // Sp02 r value formatting
        uint16_t tempVal = (uint16_t(bpmArrTwo[6]) << 8) | bpmArrTwo[7];
        libBpm.rValue = tempVal;
        libBpm.rValue /= 10.0;

        // extended machine state formatting
        libBpm.extStatus = bpmArrTwo[8];

        // two additional bytes of data were requested, but not implemented
        return libBpm;
    }

    return libBpm;
}

uint8_t SparkFun_Bio_Sensor_Hub::getMcuType(){
    return readByte(IDENTITY, READ_MCU_TYPE, NO_WRITE);
}

uint8_t SparkFun_Bio_Sensor_Hub::max30101Control(uint8_t enable){
    return writeByte(ENABLE_SENSOR, 0x03, enable);
}

uint8_t SparkFun_Bio_Sensor_Hub::readMAX30101State(){
    return readByte(READ_SENSOR_MODE, READ_ENABLE_MAX30101);
}

uint8_t SparkFun_Bio_Sensor_Hub::setOutputMode(uint8_t outputType){
    return writeByte(OUTPUT_MODE, 0x00, outputType);
}

uint8_t SparkFun_Bio_Sensor_Hub::setFifoThreshold(uint8_t threshold){
    return writeByte(OUTPUT_MODE, 0x01, threshold);
}

uint8_t SparkFun_Bio_Sensor_Hub::numSamplesOutFifo(){
    return readByte(READ_DATA_OUTPUT, NUM_SAMPLES);
}

uint8_t *SparkFun_Bio_Sensor_Hub::getDataOutFifo(uint8_t data[]){
    uint8_t samples = numSamplesOutFifo();
    readFillArray(READ_DATA_OUTPUT, READ_DATA, samples, data);
    return data;
}

uint8_t SparkFun_Bio_Sensor_Hub::numSamplesExternalSensor(){
    return readByte(READ_DATA_INPUT, SAMPLE_SIZE, WRITE_ACCELEROMETER);
}

void SparkFun_Bio_Sensor_Hub::writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal){
    writeByte(WRITE_REGISTER, WRITE_MAX30101, regAddr, regVal);
}

void SparkFun_Bio_Sensor_Hub::writeRegisterAccel(uint8_t regAddr, uint8_t regVal){
    writeByte(WRITE_REGISTER, WRITE_ACCELEROMETER, regAddr, regVal);
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30101(uint8_t regAddr){
    return readByte(READ_REGISTER, READ_MAX30101, regAddr);
}

uint8_t SparkFun_Bio_Sensor_Hub::readAlgoSamples(){
    return readByte(READ_ALGORITHM_CONFIG, 0x00, 0x03);
}

uint8_t SparkFun_Bio_Sensor_Hub::agcAlgoControl(uint8_t enable){
    return writeByte(ENABLE_ALGORITHM, 0x00, enable);
}

uint8_t SparkFun_Bio_Sensor_Hub::maximFastAlgoControl(uint8_t mode){
    return writeByte(ENABLE_ALGORITHM, 0x02, mode);
}

uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte){
    uint8_t statusByte = 0xFF;

    // write the command
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    while(UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while(UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000);           // 2ms delay @ 1MHz

    // read back the single status byte
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP;           // set stop

    while(!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    while(UCB0CTLW0 & UCTXSTP);

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeByteTwo){
    uint8_t statusByte = 0xFF;

    // write sequence: family + index + data1 + data2
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByteTwo;

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000);           // 2ms delay @ 1MHz

    // read status bytes
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP;           // set stop

    while(!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    while(UCB0CTLW0 & UCTXSTP);

    return statusByte;
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

    if(statusByte != SFE_BIO_SUCCESS){
        return statusByte;
    }

    return returnByte;
}

// similar to above, but we just add the write
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte){
    uint8_t statusByte = 0xFF;
    uint8_t returnByte = 0x00;

    // write the three bytes
    UCB0I2CSA = this->_address;
    UCB0CTLW0 = UCTR | UCTXSTT;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    while(UCB0CTLW0 & UCTXSTT);     // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP;           // generate stop condition
    while(UCB0CTLW0 & UCTXSTP);     // wait for stop to finish

    __delay_cycles(2000);           // 2ms delay @ 1MHz

    // read the status + data
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

    if(statusByte != SFE_BIO_SUCCESS){
        return statusByte;
    }

    return returnByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::readFillArray(uint8_t _familyByte, uint8_t _indexByte, uint8_t arraySize, uint8_t array[]){

    // write request
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;
    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;
    while(UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while(UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000);           // 2ms delay @ 1MHz

    // read response
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;

    while(!(UCB0IFG & UCRXIFG));
    uint8_t statusByte = UCB0RXBUF; // first byte is always status

    if(statusByte != SFE_BIO_SUCCESS){
        UCB0CTLW0 |= UCTXSTP;
        return statusByte;
    }

    for(uint8_t i = 0; i < arraySize; i++){
        if(i == (arraySize-1)){
            UCB0CTLW0 |= UCTXSTP;
        }
        while(!(UCB0IFG & UCRXIFG));
        array[i] = UCB0RXBUF;
    }

    return statusByte;
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
