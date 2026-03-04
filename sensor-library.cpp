#include <msp430.h>
#include <msp430fr2476.h>
#include <stdio.h>
#include <string.h>
#include "sensor-library.h"

#define SDA_PIN BIT2
#define SCL_PIN BIT3
#define SDA BIT2
#define SCL BIT3

#define DEBUG false

extern void uart_write_string(const char *str);
extern void uart_write_int(int num);
extern void delay_ms(uint16_t ms);

// wait for transmitter buffer (ready for new data)
inline void waitForTx()
{
    while(!(UCB0IFG & UCTXIFG));
}

// wait for receive buffer (ready to read data)
inline void waitForRx()
{
    while(!(UCB0IFG & UCRXIFG));
}

// wait for stop condition to fully complete
inline void waitForStop()
{
    while(UCB0CTLW0 & UCTXSTT); // wait for start
    while(UCB0CTLW0 & UCTXSTP); // wait for stop
}

// used to stop transmission early
inline void transmitDummyByte()
{
    UCB0CTLW0 |= UCTXSTP;       // send the stop signal
    waitForRx();
    uint8_t dummy = UCB0RXBUF;  // read the rest of buffer
    waitForStop();
}

uint8_t checkResponse()
{
    while(UCB0CTLW0 & UCTXSTT);     // wait for response

    if(UCB0IFG & UCNACKIFG)         // check NACK
    {
        UCB0CTLW0 |= UCTXSTP;       // send stop
        UCB0IFG &= ~UCNACKIFG;      // clear flag
        return ERR_TRY_AGAIN;
    }
    return SFE_BIO_SUCCESS;
}

// consolidates start + address + NACK check
uint8_t startWriteSequence(uint8_t addr)
{
    UCB0I2CSA = addr;
    UCB0CTLW0 |= UCTR | UCTXSTT;    // transmitter mode & start

    return checkResponse();
}

// wait for start & enable read
uint8_t startReadSequence()
{
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;

    return checkResponse();
}


// setting the reset & mfio pins
SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub(volatile uint8_t* resetPort, volatile uint8_t* resetOut, uint16_t resetBit,
                                                 volatile uint8_t* mfioPort, volatile uint8_t* mfioOut, volatile uint8_t* mfioRen, uint16_t mfioBit,
                                                 uint8_t address)
{

    // set the necessary pins
    _resetPort = resetPort;
    _resetOut = resetOut;
    _resetBit = resetBit;

    _mfioPort = mfioPort;
    _mfioOut = mfioOut;
    _mfioRen = mfioRen;
    _mfioBit = mfioBit;

    _address = address;

    if (_resetPort != nullptr)
    {
        *_resetPort |= _resetBit; // set direction to output
        *_resetOut |= _resetBit;  // set pin high
    }

    if (_mfioPort != nullptr)
    {
        *_mfioPort |= _mfioBit; // set direction to output
        *_mfioOut |= _mfioBit;  // set pin high
    }
}

// initializes sensor
// to do this, place sensor into application mode by pulling MFIO high
// while board is in reset
uint8_t SparkFun_Bio_Sensor_Hub::begin()
{
    // validate that pins ARE assigned
    if (_resetPort == nullptr || _mfioPort == nullptr) return ERR_UNKNOWN;

    *_mfioOut |= _mfioBit;    // set mfio high
    *_resetOut &= ~_resetBit; // set reset low
    delay_ms(10);

    *_resetOut |= _resetBit; // set reset high
    delay_ms(1000);

    // set mfio to input
    *_mfioPort &= ~_mfioBit;
    *_mfioRen |= _mfioBit;
    *_mfioOut |= _mfioBit;

    return readByte(READ_DEVICE_MODE, 0x00); // verify sensor returns 0x00 (application)
}

// places sensor in bootloader mode
// to do this, pull MFIO low while board is in reset
uint8_t SparkFun_Bio_Sensor_Hub::beginBootloader()
{
    // validate that pins ARE assigned
    if (_resetPort == nullptr || _mfioPort == nullptr) return ERR_UNKNOWN;

    *_mfioOut &= ~_mfioBit;   // set mfio low
    *_resetOut &= ~_resetBit; // set reset low
    delay_ms(10);

    *_resetOut |= _resetBit; // set reset high
    delay_ms(50);

    *_mfioOut |= _mfioBit; // set mfio high

    return readByte(READ_DEVICE_MODE, 0x00); // verify sensor returned 0x08 (bootloader)
}

// checks the status of the FIFO
uint8_t SparkFun_Bio_Sensor_Hub::readSensorHubStatus()
{
    return readByte(HUB_STATUS, 0x00);
}

// alternate way to set mode of sensor (implemented as the github has)
uint8_t SparkFun_Bio_Sensor_Hub::setOperatingMode(uint8_t selection)
{
    // parameter validation (must be 0x00, 0x02, or 0x08)
    if (selection != EXIT_BOOTLOADER && selection != SFE_BIO_RESET && selection != ENTER_BOOTLOADER) return INCORR_PARAM;

    // send mode change
    uint8_t statusByte = writeByte(SET_DEVICE_MODE, 0x00, selection);
    if (statusByte != SFE_BIO_SUCCESS) return statusByte;

    return readByte(READ_DEVICE_MODE, 0x00); // read device mode (either 0x00 or 0x08 should be returned)
}

// sets basic settings to get sensor & biometric data
// those being: heartrate, confidence, Sp02, and if a finger is detected
uint8_t SparkFun_Bio_Sensor_Hub::configBpm(uint8_t mode)
{

    if (mode != MODE_ONE && mode != MODE_TWO) return ERR_TRY_AGAIN; // make sure we selected a valid mode

    uint8_t status;

    status = setOutputMode(ALGO_DATA); // set the output mode to only data
    if (status != SFE_BIO_SUCCESS) return status;

    status = setFifoThreshold(0x01); // one sample before interrupt is fired
    if (status != SFE_BIO_SUCCESS) return status;

    status = agcAlgoControl(ENABLE); // one sample before interrupt is fired
    if (status != SFE_BIO_SUCCESS) return status;

    status = max30101Control(ENABLE); // enable control
    if (status != SFE_BIO_SUCCESS) return status;

    status = maximFastAlgoControl(mode); // set the algo mode to our choice
    if (status != SFE_BIO_SUCCESS) return status;

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    delay_ms(1000); // 1 second delay
    return SFE_BIO_SUCCESS;
}

// sets basic settings to get LED counts from MAX30101
uint8_t SparkFun_Bio_Sensor_Hub::configSensor()
{
    uint8_t status;

    status = setOutputMode(SENSOR_DATA); // set to sensor data
    if (status != SFE_BIO_SUCCESS) return status;

    status = setFifoThreshold(0x01); // one sample before interrupt is fired to MAX32664
    if (status != SFE_BIO_SUCCESS) return status;

    status = max30101Control(ENABLE); // enable sensor
    if (status != SFE_BIO_SUCCESS) return status;

    status = maximFastAlgoControl(MODE_ONE); // enable algorithm
    if (status != SFE_BIO_SUCCESS) return status;

    delay_ms(1000);
    return SFE_BIO_SUCCESS;
}

// configure both sensor & biometric data
uint8_t SparkFun_Bio_Sensor_Hub::configSensorBpm(uint8_t mode)
{

    if (mode != MODE_ONE && mode != MODE_TWO) return ERR_TRY_AGAIN; // ensure a correct mode was selected

    uint8_t status;

    status = setOutputMode(SENSOR_AND_ALGORITHM); // set data & sensor data
    if (status != SFE_BIO_SUCCESS) return status;

    status = setFifoThreshold(0x01); // one sample before interrupt is fired
    if (status != SFE_BIO_SUCCESS) return status;

    status = max30101Control(ENABLE); // enable sensor
    if (status != SFE_BIO_SUCCESS) return status;

    status = maximFastAlgoControl(mode); // enable algorithm
    if (status != SFE_BIO_SUCCESS) return status;

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    delay_ms(1000);
    return SFE_BIO_SUCCESS;
}

// takes 8 bytes from FIFO buffer: 
// heart rate (uint16_t), confidence (uint8_t), Sp02 (uint16_t), status (uint8_t)
bioData SparkFun_Bio_Sensor_Hub::readBpm()
{
    bioData libBpm;
    uint8_t status = readSensorHubStatus(); // returns 0 if no error

    // if we get a communication error
    if (status == 1)
    {
        libBpm.heartRate = 0;
        libBpm.confidence = 0;
        libBpm.oxygen = 0;
        return libBpm;
    }

    numSamplesOutFifo();

    if (_userSelectedMode == MODE_ONE)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, bpmArr);
        if (status != SFE_BIO_SUCCESS)
        {
            libBpm.heartRate = 0;
            libBpm.confidence = 0;
            libBpm.oxygen = 0;
            return libBpm;
        }

        // heart rate formatting
        libBpm.heartRate = ((uint16_t(bpmArr[0]) << 8) | bpmArr[1]) / 10;

        // confidence formatting
        libBpm.confidence = bpmArr[2];

        // blood oxygen level formatting
        libBpm.oxygen = ((uint16_t(bpmArr[3]) << 8) | bpmArr[4]) / 10;

        // has a finger been detected?
        libBpm.status = bpmArr[5];

        return libBpm;
    }
    else if (_userSelectedMode == MODE_TWO)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpmArrTwo);
        if (status != SFE_BIO_SUCCESS)
        {
            libBpm.heartRate = 0;
            libBpm.confidence = 0;
            libBpm.oxygen = 0;
            return libBpm;
        }

        // heart rate formatting
        libBpm.heartRate = ((uint16_t(bpmArrTwo[0]) << 8) | bpmArrTwo[1]) / 10;

        // confidence formatting
        libBpm.confidence = bpmArrTwo[2];

        // blood oxygen level formatting
        libBpm.oxygen = ((uint16_t(bpmArrTwo[3]) << 8) | bpmArrTwo[4]) / 10;

        // has a finger been detected?
        libBpm.status = bpmArrTwo[5];

        // Sp02 r value formatting
        // uint16_t tempVal = (uint16_t(bpmArrTwo[6]) << 8) | bpmArrTwo[7];
        // libBpm.rValue = tempVal;
        // libBpm.rValue /= 10.0;
        libBpm.rValue = ((uint16_t(bpmArrTwo[6]) << 8) | bpmArrTwo[7]) / 10;

        // extended machine state formatting
        libBpm.extStatus = bpmArrTwo[8];

        // two additional bytes of data were requested, but not implemented
        // with the sensor version we have
        return libBpm;
    }
    else
    {
        libBpm.heartRate = 0;
        libBpm.confidence = 0;
        libBpm.oxygen = 0;
        return libBpm;
    }

    return libBpm;
}

// takes 9 bytes of the LED values (IR, RED, GREEN)
bioData SparkFun_Bio_Sensor_Hub::readSensor()
{
    bioData libLedFifo;

    uint8_t status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAX30101_LED_ARRAY, senArr);

    if (status != SFE_BIO_SUCCESS) return libLedFifo;

    // format IR LED values
    libLedFifo.irLed = (uint32_t)senArr[0] << 16;
    libLedFifo.irLed |= (uint32_t)senArr[1] << 8;
    libLedFifo.irLed |= (uint32_t)senArr[2];

    // format red LED values
    libLedFifo.redLed = (uint32_t)senArr[3] << 16;
    libLedFifo.redLed |= (uint32_t)senArr[4] << 8;
    libLedFifo.redLed |= (uint32_t)senArr[5];

    // format green LED values
    libLedFifo.greenLed = (uint32_t)senArr[6] << 16;
    libLedFifo.greenLed |= (uint32_t)senArr[7] << 8;
    libLedFifo.greenLed |= (uint32_t)senArr[8];

    return libLedFifo;
}

// combines read sensor & read bpm (need to debug)
bioData SparkFun_Bio_Sensor_Hub::readSensorBpm()
{
    bioData libLedBpm;
    uint8_t status;

    if (_userSelectedMode == MODE_ONE)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY, bpmSenArr);
        if (status != SFE_BIO_SUCCESS) return libLedBpm;

        // format IR LED values
        libLedBpm.irLed = (uint32_t)bpmSenArr[0] << 16;
        libLedBpm.irLed |= (uint32_t)bpmSenArr[1] << 8;
        libLedBpm.irLed |= (uint32_t)bpmSenArr[2];

        // format red LED values
        libLedBpm.redLed = (uint32_t)bpmSenArr[3] << 16;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[4] << 8;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[5];

        // format green LED values
        libLedBpm.redLed = (uint32_t)bpmSenArr[6] << 16;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[7] << 8;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[8];

        // heart rate formatting
        libLedBpm.heartRate = ((uint16_t(bpmSenArr[12]) << 8) | bpmSenArr[13]) / 10;

        // confidence formatting
        libLedBpm.confidence = bpmSenArr[14];

        // blood oxygen level formatting
        libLedBpm.oxygen = ((uint16_t(bpmSenArr[15]) << 8) | bpmSenArr[16]) / 10;

        // "machine state" - has a finger been detected?
        libLedBpm.status = bpmSenArr[17];

        return libLedBpm;
    }
    else if (_userSelectedMode == MODE_TWO)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY + MAXFAST_EXTENDED_DATA, bpmSenArrTwo);
        if (status != SFE_BIO_SUCCESS) return libLedBpm;

        // format IR LED values
        libLedBpm.irLed = (uint32_t)bpmSenArrTwo[0] << 16;
        libLedBpm.irLed |= (uint32_t)bpmSenArrTwo[1] << 8;
        libLedBpm.irLed |= (uint32_t)bpmSenArrTwo[2];

        // format red LED values
        libLedBpm.redLed = (uint32_t)bpmSenArrTwo[3] << 16;
        libLedBpm.redLed |= (uint32_t)bpmSenArrTwo[4] << 8;
        libLedBpm.redLed |= (uint32_t)bpmSenArrTwo[5];

        // format green LED values
        libLedBpm.redLed = (uint32_t)bpmSenArr[6] << 16;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[7] << 8;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[8];

        // heart rate formatting
        libLedBpm.heartRate = ((uint16_t(bpmSenArrTwo[12]) << 8) | bpmSenArrTwo[13]) / 10;

        // confidence formatting
        libLedBpm.confidence = bpmSenArrTwo[14];

        // blood oxygen level formatting
        libLedBpm.oxygen = ((uint16_t(bpmSenArrTwo[15]) << 8) | bpmSenArrTwo[16]) / 10;

        // "machine state" - has a finger been detected?
        libLedBpm.status = bpmSenArrTwo[17];

        // Sp02 r value formatting
        uint16_t tempVal = (uint16_t(bpmSenArrTwo[18]) << 8) | bpmSenArrTwo[19];
        libLedBpm.rValue = tempVal;
        libLedBpm.rValue /= 10.0;

        // extended machine state formatting
        libLedBpm.extStatus = bpmSenArrTwo[20];

        // two additional bytes of data were requested, but not implemented
        return libLedBpm;
    }
    else
    {
        libLedBpm.irLed = 0;
        libLedBpm.redLed = 0;
        libLedBpm.heartRate = 0;
        libLedBpm.confidence = 0;
        libLedBpm.oxygen = 0;
        libLedBpm.status = 0;
        libLedBpm.rValue = 0;
        libLedBpm.extStatus = 0;
        return libLedBpm;
    }

    return libLedBpm;
}

// modifies the pulse width of the MAX30101 LEDs
uint8_t SparkFun_Bio_Sensor_Hub::setPulseWidth(uint16_t width)
{
    uint8_t bits, regVal;

    switch(width)
    {
        case 69:    bits = 0; break;    // fastest, least resolution
        case 118:   bits = 1; break;
        case 215:   bits = 2; break;
        case 411:   bits = 3; break;    // slowest, highest resolution
        default:    return INCORR_PARAM;
    }

    // get current register value to not overwrite anything
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= PULSE_MASK; // mask bits to change
    regVal |= bits;       // add bits

    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal);

    return SFE_BIO_SUCCESS;
}

// reads (0x0A) from the MAX30101 sensor
uint16_t SparkFun_Bio_Sensor_Hub::readPulseWidth()
{
    uint8_t regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_PULSE_MASK;

    // corresponds to the values in setPulseWidth()
    switch(regVal)
    {
        case 0: return 69;
        case 1: return 118;
        case 2: return 215;
        case 3: return 411;
        default: return ERR_UNKNOWN;
    }
}

// changes the sample rate of the MAX30101 sensor
uint8_t SparkFun_Bio_Sensor_Hub::setSampleRate(uint16_t sampleRate)
{
    uint8_t bits, regVal;

    // making sure the correct sample rate was selected
    switch(sampleRate)
    {
        case 50:    bits = 0; break;    // lowest sample rate (less power, least accuracy)
        case 100:   bits = 1; break;
        case 200:   bits = 2; break;
        case 400:   bits = 3; break;
        case 800:   bits = 4; break;
        case 1000:  bits = 5; break;
        case 1600:  bits = 6; break;
        case 3200:  bits = 7; break;    // highest sample rate (highest power, best accuracy)
        default:    return INCORR_PARAM;
    }

    // read-modify-write
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= SAMP_MASK;
    regVal |= (bits << 2);

    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal);

    return SFE_BIO_SUCCESS;
}

// returns the current sample rate of the device
uint16_t SparkFun_Bio_Sensor_Hub::readSampleRate()
{
    // read the register and isolate bits
    uint8_t regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_SAMP_MASK;
    regVal = (regVal >> 2);

    // all of the values here correspond to the setSampleRate() function
    switch(regVal)
    {
        case 0: return 50;
        case 1: return 100;
        case 2: return 200;
        case 3: return 400;
        case 4: return 800;
        case 5: return 1000;
        case 6: return 1600;
        case 7: return 3200;
        default: return ERR_UNKNOWN;
    }
}

// sets the dynamic range of the ADC. default = 2048nA
// low adc can clip/saturate signal, high adc lower resolution
uint8_t SparkFun_Bio_Sensor_Hub::setAdcRange(uint16_t adcVal)
{
    uint8_t bits, regVal;

    switch(adcVal)
    {
        case 2048:  bits = 0; break;
        case 4096:  bits = 1; break;
        case 8192:  bits = 2; break;
        case 16384: bits = 3; break;
        default:    return INCORR_PARAM;
    }

    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= ADC_MASK;
    regVal |= (bits << 5);

    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal);

    return SFE_BIO_SUCCESS;
}

// reads the ADC range of the sensor
uint16_t SparkFun_Bio_Sensor_Hub::readAdcRange()
{
    uint8_t regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_ADC_MASK;
    regVal = (regVal >> 5);

    // values here correspond to the values found in setAdcRange()
    switch(regVal)
    {
        case 0: return 2048;
        case 1: return 4096;
        case 2: return 8192;
        case 3: return 16384;
        default: return ERR_UNKNOWN;
    }
}

// returns a byte that signifies if we are talking to the the sensor
uint8_t SparkFun_Bio_Sensor_Hub::getMcuType()
{
    return readByte(IDENTITY, READ_MCU_TYPE);
}

// checks the version number of the bootloader on the chip
int32_t SparkFun_Bio_Sensor_Hub::getBootloaderInf()
{
    const size_t sizeOfRev = 4;
    int32_t bootVers, revNum[sizeOfRev];

    uint8_t status = readMultipleBytes(BOOTLOADER_INFO, BOOTLOADER_VERS, 0x00, sizeOfRev, revNum);

    if(status != SFE_BIO_SUCCESS) return ERR_UNKNOWN; 

    bootVers = ((int32_t)revNum[1] << 16);
    bootVers |= ((int32_t)revNum[2] << 8);
    bootVers |= (int32_t)revNum[3];

    return bootVers;
}

// enables the MAX30101
uint8_t SparkFun_Bio_Sensor_Hub::max30101Control(uint8_t enable)
{
    // if enable != 0 or 1, return
    switch(enable)
    {
        case 0:
        case 1:
            break;
        default:
            return INCORR_PARAM;
    }

    return enableWrite(ENABLE_SENSOR, ENABLE_MAX30101, enable);
}

// enables the accelerometer
uint8_t SparkFun_Bio_Sensor_Hub::accelControl(uint8_t accelSwitch)
{
    switch(accelSwitch)
    {
        case 0:
        case 1:
            break;
        default:
            return INCORR_PARAM;
    }

    return enableWrite(ENABLE_SENSOR, ENABLE_ACCELEROMETER, accelSwitch);
}

// checks if the MAX30101 is enabled or not
uint8_t SparkFun_Bio_Sensor_Hub::readMAX30101State()
{
    return readByte(READ_SENSOR_MODE, READ_ENABLE_MAX30101);
}

// sets the output mode of the sensor
uint8_t SparkFun_Bio_Sensor_Hub::setOutputMode(uint8_t outputType)
{
    if(outputType > SENSOR_ALGO_COUNTER) return INCORR_PARAM;
    return writeByte(OUTPUT_MODE, SET_FORMAT, outputType);
}

// changes the threshold for the FIFO interrupt
uint8_t SparkFun_Bio_Sensor_Hub::setFifoThreshold(uint8_t threshold)
{
    return writeByte(OUTPUT_MODE, WRITE_SET_THRESHOLD, threshold);
}

// returns the number of samples available in the FIFO
uint8_t SparkFun_Bio_Sensor_Hub::numSamplesOutFifo()
{
    return readByte(READ_DATA_OUTPUT, NUM_SAMPLES);
}

// returns the data in the FIFO
uint8_t *SparkFun_Bio_Sensor_Hub::getDataOutFifo(uint8_t data[])
{
    uint8_t samples = numSamplesOutFifo();
    readFillArray(READ_DATA_OUTPUT, READ_DATA, samples, data);
    return data;
}

// adds support for the accelerometer that is not included w/ sensor
uint8_t SparkFun_Bio_Sensor_Hub::numSamplesExternalSensor()
{
    return readByte(READ_DATA_INPUT, SAMPLE_SIZE, WRITE_ACCELEROMETER);
}

// writes the given register value to the register address for the MAX30101
void SparkFun_Bio_Sensor_Hub::writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal)
{
    writeByte(WRITE_REGISTER, WRITE_MAX30101, regAddr, regVal);
}

// writes the given register value to the register address for the accelerometer
void SparkFun_Bio_Sensor_Hub::writeRegisterAccel(uint8_t regAddr, uint8_t regVal)
{
    writeByte(WRITE_REGISTER, WRITE_ACCELEROMETER, regAddr, regVal);
}

// reads the given register address for the MAX30101 sensor & returns the value
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30101(uint8_t regAddr)
{
    return readByte(READ_REGISTER, READ_MAX30101, regAddr);
}

// reads the given register address for the accelerometer & returns the value
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterAccel(uint8_t regAddr)
{
    return readByte(READ_REGISTER, READ_ACCELEROMETER, regAddr);
}

// retrieves the attributes of the analog-front-end (AFE) of the MAX30101
sensorAttr SparkFun_Bio_Sensor_Hub::getAfeAttributesMAX30101()
{
    sensorAttr maxAttr;
    uint8_t tempArray[2]{};

    uint8_t status = readFillArray(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30101, 2, tempArray);

    if(status == SFE_BIO_SUCCESS)
    {
        maxAttr.byteWord = tempArray[0];
        maxAttr.availRegisters = tempArray[1];
    }

    return maxAttr;
}

// retrieves the attributes of the analog-front-end (AFE) of the accelerometer
sensorAttr SparkFun_Bio_Sensor_Hub::getAfeAttributesAccelerometer()
{
    sensorAttr accelAttr;
    uint8_t tempArray[2]{};

    uint8_t status = readFillArray(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_ACCELEROMETER, 2, tempArray);

    if(status == SFE_BIO_SUCCESS)
    {
        accelAttr.byteWord = tempArray[0];
        accelAttr.availRegisters = tempArray[1];
    }

    return accelAttr;
}

// returns all registers and register values of the MAX30101
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterMAX30101(uint8_t regArray[])
{
    return readFillArray(DUMP_REGISTERS, DUMP_REGISTER_MAX30101, 36, regArray);
}

// returns all registers and register values of the accelerometer
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterAccelerometer(uint8_t numReg, uint8_t regArray[])
{
    return readFillArray(DUMP_REGISTERS, DUMP_REGISTER_ACCELEROMETER, numReg, regArray);
}

uint8_t SparkFun_Bio_Sensor_Hub::setAlgoRange(uint8_t perc)
{
    if(perc > 100)
    {
        return INCORR_PARAM;
    }

    return writeByte(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_GAIN_ID, perc);
}

uint8_t SparkFun_Bio_Sensor_Hub::setAlgoStepSize(uint8_t step)
{
    if(step > 100)
    {
        return INCORR_PARAM;
    }

    return writeByte(CHANGE_ALGORITHM_CONFIG, SET_STEP_SIZE, AGC_STEP_SIZE_ID, step);
}

uint8_t SparkFun_Bio_Sensor_Hub::setAlgoSensitivity(uint8_t sense)
{
    if(sense > 100)
    {
        return INCORR_PARAM;
    }

    return writeByte(CHANGE_ALGORITHM_CONFIG, SET_SENSITIVITY, AGC_SENSITIVITY_ID, sense);
}

uint8_t SparkFun_Bio_Sensor_Hub::setAlgoSamples(uint8_t avg)
{
    return writeByte(CHANGE_ALGORITHM_CONFIG, SET_AVG_SAMPLES, AGC_NUM_SAMP_ID, avg);
}

uint8_t SparkFun_Bio_Sensor_Hub::setMaximFastCoef(int32_t coef1, int32_t coef2, int32_t coef3)
{
    const size_t numCoefVals = 3;
    int32_t coefArr[3] = {coef1, coef2, coef3};

    return writeLongBytes(CHANGE_ALGORITHM_CONFIG, SET_PULSE_OX_COEF, MAXIMFAST_COEF_ID, coefArr, numCoefVals);
}

uint8_t SparkFun_Bio_Sensor_Hub::readAlgoRange()
{
    return readByte(READ_ALGORITHM_CONFIG, READ_AGC_PERCENTAGE, READ_AGC_PERC_ID);
}

uint8_t SparkFun_Bio_Sensor_Hub::readAlgoStepSize()
{
    return readByte(READ_ALGORITHM_CONFIG, READ_AGC_STEP_SIZE, READ_AGC_STEP_SIZE_ID);
}

uint8_t SparkFun_Bio_Sensor_Hub::readAlgoSensitivity()
{
    return readByte(READ_ALGORITHM_CONFIG, READ_AGC_SENSITIVITY, READ_AGC_SENSITIVITY_ID);
}

uint8_t SparkFun_Bio_Sensor_Hub::readAlgoSamples()
{
    return readByte(READ_ALGORITHM_CONFIG, READ_AGC_NUM_SAMPLES, READ_AGC_NUM_SAMPLES_ID);
}

uint8_t SparkFun_Bio_Sensor_Hub::readMaximFastCoef(int32_t coefArr[3])
{
    const size_t numOfReads = 3;
    uint8_t status = readMultipleBytes(READ_ALGORITHM_CONFIG, READ_MAX_FAST_COEF, READ_MAX_FAST_COEF_ID, numOfReads, coefArr);

    // original code does mult by 100,000 but that is usually for display formatting
    // for now, we shall save performance, but just in case:
    // coefArr[0] *= 100000;
    // coefArr[1] *= 100000;
    // coefArr[2] *= 100000;
    return status;
}

uint8_t SparkFun_Bio_Sensor_Hub::agcAlgoControl(uint8_t enable)
{
    switch(enable)
    {
        case 0:
        case 1:
            break;
        default:
            return INCORR_PARAM;
    }

    return enableWrite(ENABLE_ALGORITHM, ENABLE_AGC_ALGO, enable);
}

uint8_t SparkFun_Bio_Sensor_Hub::maximFastAlgoControl(uint8_t mode)
{
    switch(mode)
    {
        case 0:
        case 1:
        case 2:
            break;
        default:
            return INCORR_PARAM;
    }

    return enableWrite(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
}

bool SparkFun_Bio_Sensor_Hub::setNumPages(uint8_t totalPages)
{
    return writeByte(BOOTLOADER_FLASH, SET_NUM_PAGES, 0x00, totalPages);
}

bool SparkFun_Bio_Sensor_Hub::eraseFlash()
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = BOOTLOADER_FLASH;
    waitForTx();
    UCB0TXBUF = ERASE_FLASH;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();          // wait until stop condition
    delay_ms(10);           // processing delay

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
    
    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return (statusByte == SFE_BIO_SUCCESS);
}

version SparkFun_Bio_Sensor_Hub::readBootloaderVers()
{
    version booVers = {0, 0, 0}; // BOO! (originally from the github, funny comment)
    uint8_t versionArray[3] = {0};

    // the logic from the original is changed with this. we were requesting and reading 3 bytes
    // readMultipleBytes does exactly as the above comment says.
    uint8_t status = readMultipleBytes(BOOTLOADER_INFO, BOOTLOADER_VERS, 0x00, 3, versionArray);

    if(status == SFE_BIO_SUCCESS)
    {
        booVers.major = versionArray[0];
        booVers.minor = versionArray[1];
        booVers.revision = versionArray[2];
    }

    return booVers;
}

version SparkFun_Bio_Sensor_Hub::readSensorHubVersion()
{
    version bioHubVers = {0, 0, 0};
    uint8_t versionArray[3] = {0};

    uint8_t status = readMultipleBytes(IDENTITY, READ_SENSOR_HUB_VERS, 3, versionArray);

    if(status == SFE_BIO_SUCCESS)
    {
        bioHubVers.major = versionArray[0];
        bioHubVers.minor = versionArray[1];
        bioHubVers.revision = versionArray[2];
    }

    return bioHubVers;
}

version SparkFun_Bio_Sensor_Hub::readAlgorithmVersion()
{
    version libAlgoVersion = {0, 0, 0};
    uint8_t versionArray[3] = {0};

    uint8_t status = readMultipleBytes(IDENTITY, READ_ALGO_VERS, 0x00, 3, versionArray);

    if(status == SFE_BIO_SUCCESS)
    {
        libAlgoVersion.major = versionArray[0];
        libAlgoVersion.minor = versionArray[1];
        libAlgoVersion.revision = versionArray[2];
    }

    return libAlgoVersion;
}

//// Functions below are for Blood Pressure version, putting them in JUST IN CASE ////

uint8_t SparkFun_Bio_Sensor_Hub::isPatientBPMedication(uint8_t medication)
{
    switch(medication)
    {
        case 0:
        case 1:
            break;
        default:
            return INCORR_PARAM;
    }

    return writeByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_MEDICATION, medication);
}

uint8_t SparkFun_Bio_Sensor_Hub::isPatientBPMedication()
{
    return readByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_MEDICATION);
}

uint8_t SparkFun_Bio_Sensor_Hub::writeDiastolicVals(uint8_t diasVal1, uint8_t diasVal2, uint8_t diasVal3)
{
    const size_t numDiasVals = 3;
    uint8_t diasVals[3] = {diasVal1, diasVal2, diasVal3};

    return writeBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, DIASTOLIC_VALUE, diasVals, numDiasVals);
}

uint8_t SparkFun_Bio_Sensor_Hub::readDiastolicVals(uint8_t userArray[])
{
    const size_t numDiasVals = 3;
    return readMultipleBytes(READ_ALGORITHM_CONFIG, BPT_CONFIG, DIASTOLIC_VALUE, numDiasVals, userArray);
}

uint8_t SparkFun_Bio_Sensor_Hub::writeSystolicVals(uint8_t sysVal1, uint8_t sysVal2, uint8_t sysVal3)
{
    const size_t numSysVals = 3;
    uint8_t sysVals[3] = {sysVal1, sysVal2, sysVal3};

    return writeBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, SYSTOLIC_VALUE, sysVals, numSysVals);
}

uint8_t SparkFun_Bio_Sensor_Hub::readSystolicVals(uint8_t userArray[])
{
    const size_t numSysVals = 3;

    return readMultipleBytes(READ_ALGORITHM_CONFIG, BPT_CONFIG, SYSTOLIC_VALUE, numSysVals, userArray);
}

uint8_t SparkFun_Bio_Sensor_Hub::writeBPTAlgoData(uint8_t bptCalibData[])
{
    const size_t numCalibVals = 824;
    return writeBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_CALIB_DATA, bptCalibData, numCalibVals);
}

uint8_t SparkFun_Bio_Sensor_Hub::readBPTAlgoData(uint8_t userArray[])
{
    const size_t numCalibVals = 824;
    return readMultipleBytes(READ_ALGORITHM_CONFIG, BPT_CONFIG, BPT_CALIB_DATA, numCalibVals, userArray);
}

uint8_t SparkFun_Bio_Sensor_Hub::isPatientResting(uint8_t resting)
{
    switch(resting)
    {
        case 0:
        case 1:
            break;
        default:
            return INCORR_PARAM;
    }

    return writeByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, PATIENT_RESTING, resting);
}

uint8_t SparkFun_Bio_Sensor_Hub::isPatientResting()
{
    return readByte(READ_ALGORITHM_CONFIG, BPT_CONFIG, PATIENT_RESTING);
}

uint8_t SparkFun_Bio_Sensor_Hub::writeSP02AlgoCoef(int32_t intA, int32_t intB, int32_t intC)
{
    const size_t numCoefVals = 3;
    int32_t coefVals[numCoefVals] = {intA, intB, intC};

    return writeLongBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, AGC_SP02_COEFS, coefVals, numCoefVals);
}

uint8_t SparkFun_Bio_Sensor_Hub::readSP02AlgoCoef(int32_t userArray[])
{
    return readMultipleBytes(READ_ALGORITHM_CONFIG, BPT_CONFIG, AGC_SP02_COEFS, 3, userArray);
}

//// DRIVER FUNCTIONS ////
uint8_t SparkFun_Bio_Sensor_Hub::enableWrite(uint8_t _familyByte, uint8_t _indexByte, uint8_t _enableByte)
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();
    
    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _enableByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();          // wait until stop condition
    delay_ms(51);            // processing delay (extra for enable)

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
    
    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return statusByte;
}

// sending one write byte
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();          // wait until stop condition
    delay_ms(6);            // processing delay

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
    
    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return statusByte;
}

// sending two write bytes
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal)
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    UCB0TXBUF = _writeVal;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
    
    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return statusByte;
}

// same as above, but instead writes a uint16_t
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val)
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    // set buffer to first half of val (MSB)
    UCB0TXBUF = (uint8_t)(_val >> 8);
    waitForTx();

    // set buffer to second half of val (LSB)
    UCB0TXBUF = (uint8_t)(_val & 0xFF);
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return statusByte;
}

// write uint32_t byte
uint8_t SparkFun_Bio_Sensor_Hub::writeLongBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, int32_t _writeVal[], const size_t _len)
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    for(size_t i = 0; i < _len; i++)
    {
        for(int8_t shift_by = 24; shift_by != -8; shift_by -= 8)
        {
            // send 8 bits
            UCB0TXBUF = (uint8_t)(_writeVal[i] >> shift_by);

            // wait for transmit interrupt
            waitForTx();
        }
    }

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return statusByte;
}

// write an array of bytes
uint8_t SparkFun_Bio_Sensor_Hub::writeBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal[], const size_t _len)
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    for(size_t i = 0; i != _len; i++)
    {
        UCB0TXBUF = _writeVal[i];   // set buffer to data byte
        waitForTx();
    }

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// READ STATUS BYTE ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    statusByte = UCB0RXBUF;
    waitForStop();
    return statusByte;
}

// reads a status byte (or return byte) from the device
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte)
{
    uint8_t returnByte, statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
    
    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF; // set status to buffer
    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    returnByte = UCB0RXBUF; // set return to buffer
    waitForStop();

    if (statusByte != SFE_BIO_SUCCESS) return statusByte;
    else return returnByte;
}

// similar to above, but we just add the write byte
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    uint8_t returnByte, statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
    
    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF; // set status to buffer
    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForRx();
    returnByte = UCB0RXBUF; // set return to buffer
    waitForStop();

    if (statusByte != SFE_BIO_SUCCESS) return statusByte;
    else return returnByte;
}

// reads a uint16_t value
uint16_t SparkFun_Bio_Sensor_Hub::readIntByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    uint16_t returnByte;
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF;

    if(statusByte != SFE_BIO_SUCCESS)
    {
        transmitDummyByte();
        return statusByte;
    }

    waitForRx();
    returnByte = (uint16_t)(UCB0RXBUF << 8);    // set half of return to buffer
    UCB0CTLW0 |= UCTXSTP;                       // send the stop signal
    waitForRx();
    returnByte |= UCB0RXBUF;                    // set other half of return
    waitForStop();
    return returnByte;
}

// reads the bytes in an arr
uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, const size_t _len, uint8_t userArray[])
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF;

    if(statusByte != SFE_BIO_SUCCESS)
    {
        transmitDummyByte();
        return statusByte;
    }

    // loop for len indexes (i is set to size_t to match _len type)
    for(size_t i = 0; i != _len; i++)
    {
        if(i == (_len - 1)) UCB0CTLW0 |= UCTXSTP;   // send the stop signal
        waitForRx();
        userArray[i] = UCB0RXBUF;
    }

    waitForStop();
    return statusByte;
}

// same as above, but also adds a write byte
uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _len, uint8_t userArray[])
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF;

    if(statusByte != SFE_BIO_SUCCESS)
    {
        transmitDummyByte();
        return statusByte;
    }
    
    for (size_t i = 0; i != _len; i++)
    {
        if(i == (_len - 1)) UCB0CTLW0 |= UCTXSTP;   // send the stop signal
        waitForRx();
        userArray[i] = UCB0RXBUF;
    }
    
    waitForStop();
    return statusByte;
}

// same as above, but user array is of uint32_t
uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _len, int32_t userArray[])
{
    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0TXBUF = _writeByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF;

    if(statusByte != SFE_BIO_SUCCESS)
    {
        transmitDummyByte();
        return statusByte;
    }
    
    for (size_t i = 0; i != _len; i++)
    {
        userArray[i] = 0;

        for(int8_t shift_by = 24; shift_by != -8; shift_by -= 8)
        {
            if(i == (_len - 1) && shift_by == 0) UCB0CTLW0 |= UCTXSTP;   // send the stop signal
            waitForRx();
            userArray[i] = ((uint32_t)UCB0RXBUF << shift_by);
        }
    }
    
    waitForStop();
    return statusByte;
}

// write to an array passed through
uint8_t SparkFun_Bio_Sensor_Hub::readFillArray(uint8_t _familyByte, uint8_t _indexByte, uint8_t arraySize, uint8_t array[])
{
    if(arraySize == 0) return ERR_UNKNOWN;

    uint8_t statusByte;

    //// START A WRITE ////
    if(startWriteSequence(this->_address) != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;
   
    UCB0TXBUF = _familyByte;
    waitForTx();

    UCB0TXBUF = _indexByte;
    waitForTx();

    UCB0CTLW0 |= UCTXSTP;   // send the stop signal
    waitForStop();
    delay_ms(6);

    //// START A READ ////
    if(startReadSequence() != SFE_BIO_SUCCESS) return ERR_TRY_AGAIN;

    waitForRx();
    statusByte = UCB0RXBUF;

    if(statusByte != SFE_BIO_SUCCESS)
    {
        transmitDummyByte();
        return statusByte;
    }

    for (uint8_t i = 0; i != arraySize; i++)
    {
        if (i == (arraySize - 1)) UCB0CTLW0 |= UCTXSTP;   // send the stop signal
        waitForRx();
        array[i] = UCB0RXBUF;
    }

    waitForStop();
    return statusByte;
}

void delay_ms(uint16_t ms)
{
    // timer B0, 32kHz clock, count up, clock divider = 1
    TB0CTL = (TBSSEL__ACLK | MC__UP | ID__1 | TBCLR);

    // set the count value, about 32 counts per ms
    TB0CCR0 = 33 * ms;

    // clear the timer interrupt
    TB0CCTL0 &= ~CCIFG;

    // enable clock interrupt
    TB0CCTL0 |= CCIE;

    // enable low power mode & interrupts
    __bis_SR_register(LPM3_bits + GIE);

    // disable timer after we wake
    TB0CTL = MC__STOP;
}

#pragma vector = TIMER0_B0_VECTOR;
__interrupt void Timer_B0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits); // wake CPU & exit low-power
}

// creating UART for debugging since printing doesn't seem to work
void initUART()
{
    // configure uart pins, 1.4 = TX & 1.5 = RX
    P1SEL0 |= BIT4 | BIT5;

    // configure uart for 115.2k baud
    UCA0CTLW0 |= UCSWRST;       // eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // choose smclk

    UCA0BR0 = 8;
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0xD600;

    UCA0CTLW0 &= ~UCSWRST; // release from reset
}

void uart_write_char(char c)
{
    while (!(UCA0IFG & UCTXIFG)); // wait for tx buffer to be empty
    UCA0TXBUF = c;
}

void uart_write_string(const char *str)
{
    while (*str)
    {
        if (*str == '\n') uart_write_char('\r');
        uart_write_char(*str++);
    }
}

void uart_write_int(int num)
{
    char buf[10];
    int i = 0;
    if (num == 0)
    {
        uart_write_char('0');
        return;
    }
    while (num > 0 && i < 10)
    {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while (--i >= 0)
    {
        uart_write_char(buf[i]);
    }
}

// \x1B = ESC, ESC [2J clears the screen, ESC [H moves cursor to top left
void uart_clear_screen()
{
    uart_write_string("\x1B[2J\x1B[H"); 
}

uint8_t runI2CScanner() {

    uart_write_string("Hardware Scan Starting...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        UCB0I2CSA = addr;
        UCB0CTLW0 |= UCTR | UCTXSTT; // start & write

        uint32_t timeout = 5000;
        while ((UCB0CTLW0 & UCTXSTT) && --timeout); // Wait for Address

        // bus is stuck
        if (timeout == 0)
        {
            UCB0CTLW0 |= UCSWRST;
            UCB0CTLW0 &= ~UCSWRST;
            continue;
        }

        // device ACK'd
        if (!(UCB0IFG & UCNACKIFG))
        {
            uart_write_string("Found i2c device at: ");
            uart_write_int(addr);
            uart_write_string("\r\n");

            // stop communication & return
            UCB0CTLW0 |= UCTXSTP;   // send the stop signal

            uint32_t stopTimeout = 1000;
            while((UCB0CTLW0 & UCTXSTP) && --stopTimeout);

            UCB0IFG &= ~UCNACKIFG;

            if(stopTimeout == 0)
            {
                UCB0CTLW0 |= UCSWRST;
                UCB0CTLW0 &= ~UCSWRST;
            }

            return 1;
        }

        UCB0CTLW0 |= UCTXSTP;   // send the stop signal      // Always send STOP to release bus
        while (UCB0CTLW0 & UCTXSTP);
        UCB0IFG &= ~UCNACKIFG;     // Clear NACK flag for next loop
    }

    return 0;
}

int main(void)
{
    // stop watchdog & gpio high-impedance
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // init uart
    initUART();
    uart_clear_screen();    // clear the screen so we can properly see all the messages
    delay_ms(100);          // wait a little bit
    uart_write_string("\r\n--- SYSTEM STARTUP ---\n");

    // ensure i2c lines are high
    P1SEL0 &= ~(BIT2 | BIT3);
    P1SEL1 &= ~(BIT2 | BIT3);
    P1DIR &= ~(BIT2 | BIT3); // inputs
    P1REN &= ~(BIT2 | BIT3); // external resistors

    // toggle scl to clear
    P1DIR |= BIT3;
    for(int i = 0; i < 9; i++)
    {
        P1OUT &= ~BIT3;
        delay_ms(1);
        P1OUT |= BIT3;
        delay_ms(1);
    }

    uart_write_string("I2C Bus Recovered (ideally).\n");

    // switch to hardware i2c mode
    P1SEL0 |= (BIT2 | BIT3);
    P1SEL1 &= ~(BIT2 | BIT3);   // double-checking
    UCB0CTLW0 = UCSWRST; // enable reset
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC | UCSSEL__SMCLK | UCSWRST;
    UCB0BRW = 10;
    UCB0CTLW0 &= ~UCSWRST;

    uint8_t tester = runI2CScanner();

    // init library
    uart_write_string("\n--- Initializing Bio Sensor ---\n");

    // need to disable PSELX to allow pins to be used as GPIO
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 &= ~(BIT0 | BIT1);

    // create sensor instance
    SparkFun_Bio_Sensor_Hub bioHub(&P2DIR, &P2OUT, BIT0,
                                   &P2DIR, &P2OUT, &P2REN, BIT1,
                                   BIO_ADDRESS);
    // instance of bioData
    bioData body;

    uart_write_string("Created a sensor instance. Beginning sensor.\n");

    uint8_t result = bioHub.begin();

    // check if we got a response
    if(result == 0x00)
    {
        uart_write_string("SUCCESS: Bio Sensor Online!\r\n");

        // get mcu type
        uint8_t mcuType = bioHub.getMcuType();
        uart_write_string("MCU Type (0x01 expected): 0x");
        uart_write_int(mcuType);
        uart_write_string("\r\n");

        // get sensor hub version
        version hubVer = bioHub.readSensorHubVersion();
        uart_write_string("Hub Version: ");
        uart_write_int(hubVer.major);
        uart_write_string(".");
        uart_write_int(hubVer.minor);
        uart_write_string(".");
        uart_write_int(hubVer.revision);
        uart_write_string("\r\n");

        uint8_t error = bioHub.configSensorBpm(MODE_ONE);

        if(error == SFE_BIO_SUCCESS) uart_write_string("Sensor Configured!\n");
        else uart_write_string("Error in configuring sensor.\n");

        while(1)
        {
            body = bioHub.readSensorBpm();
            uart_write_string("\n---- SENSOR READINGS ----\n");
            uart_write_string("Heartrate: ");
            uart_write_int(body.heartRate);
            uart_write_string("\n");
            uart_write_string("Confidence: ");
            uart_write_int(body.confidence);
            uart_write_string("\n");
            uart_write_string("Oxygen: ");
            uart_write_int(body.oxygen);
            uart_write_string("\n");
            uart_write_string("Status: ");
            uart_write_int(body.status);
            uart_write_string("\n");
            uart_write_string("Green LED: ");
            uart_write_int(body.greenLed);
            uart_write_string("\n");
            uart_write_string("\n");
            delay_ms(2000); // slow down the output
        }
    }
    else
    {
        uart_write_string("Error: Initialization failed. Code: ");
        uart_write_int(result);
        uart_write_string("\r\n");
    }

    while(1); // End of test
}