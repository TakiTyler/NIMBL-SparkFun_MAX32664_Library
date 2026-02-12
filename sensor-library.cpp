#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "sensor-library.h"

#define SDA_PIN BIT2
#define SCL_PIN BIT3
#define SDA BIT2
#define SCL BIT3

#define DEBUG true

extern void uart_write_string(const char *str);
extern void uart_write_int(int num);

// setting the reset & mfio pins
SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub(volatile uint8_t* resetPort, volatile uint8_t* resetOut, uint16_t resetBit,
                                                 volatile uint8_t* mfioPort, volatile uint8_t* mfioOut, volatile uint8_t* mfioRen, uint16_t mfioBit,
                                                 uint8_t address)
{

    // set the necessary pins
    _resetPort = resetPort;
    _resetOut = resetOut;
    //    _resetOut = resetPort + 2; // out is always +2 bits offset from port (disabled for now)
    _resetBit = resetBit;

    _mfioPort = mfioPort;
    _mfioOut = mfioOut;
    _mfioRen = mfioRen;
    //    _mfioOut = mfioPort + 2; // out is always +2 bits offset from port
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

uint8_t SparkFun_Bio_Sensor_Hub::begin()
{
    if(DEBUG) uart_write_string("\tbegin part 1\n");

    // validate that pins ARE assigned
    if (_resetPort == nullptr || _mfioPort == nullptr)
    {
        return 0xFF; // MAX32664 general error code
    }

    if(DEBUG) uart_write_string("\tbegin part 2\n");

    // pull mfio high in reset for 10ms
    *_mfioOut |= _mfioBit;    // write mfio high
    *_resetOut &= ~_resetBit; // write reset low
    __delay_cycles(10000);    // 10ms delay @ 1MHz

    if(DEBUG) uart_write_string("\tbegin part 3\n");

    *_resetOut |= _resetBit; // write reset high
    __delay_cycles(1000000); // 1000ms delay @ 1MHz

    if(DEBUG) uart_write_string("\tbegin part 4\n");

    // set mfio to input
    // pulling enable is always offset by 0x06 (P(x) is at 0x06, P(y) is at 0x07 (x = odd #, y = even #))
    *_mfioPort &= ~_mfioBit;
    *_mfioRen |= _mfioBit;
    *_mfioOut |= _mfioBit;

    if(DEBUG) uart_write_string("\tbegin part 5\n");

    // verify MAX32664 returned 0x00 (READ_DEVICE_MODE = 0x02)
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00);

    if(DEBUG) uart_write_string("\tbegin part 6\n");

    return responseByte;
}

// same as begin, but inverting some pins being held low/high
uint8_t SparkFun_Bio_Sensor_Hub::beginBootloader()
{

    // validate that pins ARE assigned
    if (_resetPort == nullptr || _mfioPort == nullptr)
    {
        return 0xFF; // MAX32664 general error code
    }

    // pull mfio high in reset for 10ms
    *_mfioOut &= ~_mfioBit;   // write mfio low
    *_resetOut &= ~_resetBit; // write reset low
    __delay_cycles(10000);    // 10ms delay @ 1MHz

    *_resetOut |= _resetBit; // write reset high
    __delay_cycles(50000);   // 50ms delay @ 1MHz for the bootloader

    // set mfio to output
    *_mfioOut |= _mfioBit;

    // verify MAX32664 returned 0x08 (READ_DEVICE_MODE = 0x02)
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00);
    return responseByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::readSensorHubStatus()
{
    uint8_t responseByte = readByte(HUB_STATUS, 0x00);
    return responseByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::setOperatingMode(uint8_t selection)
{

    // parameter validation (must be 0x00, 0x02, or 0x08)
    if (selection != EXIT_BOOTLOADER && selection != SFE_BIO_RESET && selection != ENTER_BOOTLOADER)
    {
        return INCORR_PARAM; // 0xEE
    }

    // send mode change
    uint8_t statusByte = writeByte(SET_DEVICE_MODE, 0x00, selection);
    if (statusByte != SFE_BIO_SUCCESS)
    {
        return statusByte;
    }

    // wait for a verification
    __delay_cycles(100000); // 100ms delay @ 1MHz

    // READ_DEVICE_MODE = 0x02
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00);
    return responseByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::configBpm(uint8_t mode)
{

    if(DEBUG) uart_write_string("\tconfigBpm part 1\n");

    if (mode != MODE_ONE && mode != MODE_TWO)
    {
        if(DEBUG) uart_write_string("\tconfigBpm (INPUT CORRECT MODE)\n");
        return 0xEE;
    }

    if(DEBUG) uart_write_string("\tconfigBpm part 2\n");

    uint8_t status;

    if(DEBUG) uart_write_string("\tconfigBpm part 3\n");

    status = setOutputMode(ALGO_DATA);
    if (status != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\tconfigBpm (FAILED @ setOutputMode)\n");
        return status;
    }

    if(DEBUG) uart_write_string("\tconfigBpm part 4\n");

    status = setFifoThreshold(0x01); // one sample before interrupt is fired
    if (status != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\tconfigBpm (FAILED @ setFifoThreshold)\n");
        return status;
    }

    if(DEBUG) uart_write_string("\tconfigBpm part 5\n");

    status = agcAlgoControl(ENABLE); // one sample before interrupt is fired
    if (status != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\tconfigBpm (FAILED @ acgAlgoControl)\n");
        return status;
    }

    if(DEBUG) uart_write_string("\tconfigBpm part 6\n");

    status = max30101Control(ENABLE);
    if (status != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\tconfigBpm (FAILED @ max30101Control)\n");
        return status;
    }

    if(DEBUG) uart_write_string("\tconfigBpm part 7\n");

    status = maximFastAlgoControl(mode);
    if (status != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\tconfigBpm (FAILED @ maximFastAlgoControl)\n");
        return status;
    }

    if(DEBUG) uart_write_string("\tconfigBpm part 8\n");

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    if(DEBUG) uart_write_string("\tconfigBpm part 9\n");

    __delay_cycles(1000000); // 1 second delay @ 1MHz
    return SFE_BIO_SUCCESS;
}

uint8_t SparkFun_Bio_Sensor_Hub::configSensor()
{
    uint8_t status;

    status = setOutputMode(SENSOR_DATA);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    status = setFifoThreshold(0x01);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    status = max30101Control(ENABLE);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    status = maximFastAlgoControl(MODE_ONE);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    __delay_cycles(1000000); // 1 second delay @ 1MHz
    return SFE_BIO_SUCCESS;
}

uint8_t SparkFun_Bio_Sensor_Hub::configSensorBpm(uint8_t mode)
{

    if (mode != MODE_ONE && mode != MODE_TWO)
    {
        return 0xEE;
    }

    uint8_t status;

    status = setOutputMode(SENSOR_AND_ALGORITHM);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    status = setFifoThreshold(0x01); // one sample before interrupt is fired
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    status = max30101Control(ENABLE);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    status = maximFastAlgoControl(mode);
    if (status != SFE_BIO_SUCCESS)
    {
        return status;
    }

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    __delay_cycles(1000000); // 1 second delay @ 1MHz
    return SFE_BIO_SUCCESS;
}

bioData SparkFun_Bio_Sensor_Hub::readBpm()
{
    bioData libBpm = {0};
    uint8_t status = readSensorHubStatus();

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
    else if (_userSelectedMode == MODE_TWO)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpmArrTwo);
        if (status != SFE_BIO_SUCCESS)
        {
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
    else
    {
        libBpm.heartRate = 0;
        libBpm.confidence = 0;
        libBpm.oxygen = 0;
        return libBpm;
    }

    return libBpm;
}

bioData SparkFun_Bio_Sensor_Hub::readSensor()
{
    bioData libLedFifo = {0}; // init everything to 0

    uint8_t status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAX30101_LED_ARRAY, senArr);

    if (status != SFE_BIO_SUCCESS)
    {
        return libLedFifo;
    }

    // format IR LED values
    libLedFifo.irLed = (uint32_t)senArr[0] << 16;
    libLedFifo.irLed |= (uint32_t)senArr[1] << 8;
    libLedFifo.irLed |= (uint32_t)senArr[2];

    // format red LED values
    libLedFifo.redLed = (uint32_t)senArr[3] << 16;
    libLedFifo.redLed |= (uint32_t)senArr[4] << 8;
    libLedFifo.redLed |= (uint32_t)senArr[5];

    // if our sensor has green, we'd do the same stuff but increase the senArr value
    // wrote the code down for that JUST IN CASE
    // libLedFifo.greenLed = (uint32_t)senArr[6] << 16;
    // libLedFifo.greenLed |= (uint32_t)senArr[7] << 8;
    // libLedFifo.greenLed |= (uint32_t)senArr[8];

    return libLedFifo;
}

bioData SparkFun_Bio_Sensor_Hub::readSensorBpm()
{
    bioData libLedBpm = {0};
    uint8_t status;

    if (_userSelectedMode == MODE_ONE)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY, bpmSenArr);
        if (status != SFE_BIO_SUCCESS)
        {
            return libLedBpm;
        }

        // format IR LED values
        libLedBpm.irLed = (uint32_t)bpmSenArr[0] << 16;
        libLedBpm.irLed |= (uint32_t)bpmSenArr[1] << 8;
        libLedBpm.irLed |= (uint32_t)bpmSenArr[2];

        // format red LED values
        libLedBpm.redLed = (uint32_t)bpmSenArr[3] << 16;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[4] << 8;
        libLedBpm.redLed |= (uint32_t)bpmSenArr[5];

        // heart rate formatting
        libLedBpm.heartRate = (uint16_t(bpmSenArr[12]) << 8) | bpmSenArr[13];
        libLedBpm.heartRate /= 10;

        // confidence formatting
        libLedBpm.confidence = bpmSenArr[14];

        // blood oxygen level formatting
        libLedBpm.oxygen = (uint16_t(bpmSenArr[15]) << 8) | bpmSenArr[16];
        libLedBpm.oxygen /= 10;

        // "machine state" - has a finger been detected?
        libLedBpm.status = bpmSenArr[17];

        return libLedBpm;
    }
    else if (_userSelectedMode == MODE_TWO)
    {
        status = readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY + MAXFAST_EXTENDED_DATA, bpmSenArrTwo);
        if (status != SFE_BIO_SUCCESS)
        {
            return libLedBpm;
        }

        // format IR LED values
        libLedBpm.irLed = (uint32_t)bpmSenArrTwo[0] << 16;
        libLedBpm.irLed |= (uint32_t)bpmSenArrTwo[1] << 8;
        libLedBpm.irLed |= (uint32_t)bpmSenArrTwo[2];

        // format red LED values
        libLedBpm.redLed = (uint32_t)bpmSenArrTwo[3] << 16;
        libLedBpm.redLed |= (uint32_t)bpmSenArrTwo[4] << 8;
        libLedBpm.redLed |= (uint32_t)bpmSenArrTwo[5];

        // heart rate formatting
        libLedBpm.heartRate = (uint16_t(bpmSenArrTwo[12]) << 8) | bpmSenArrTwo[13];
        libLedBpm.heartRate /= 10;

        // confidence formatting
        libLedBpm.confidence = bpmSenArrTwo[14];

        // blood oxygen level formatting
        libLedBpm.oxygen = (uint16_t(bpmSenArrTwo[15]) << 8) | bpmSenArrTwo[16];
        libLedBpm.oxygen /= 10;

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

uint8_t SparkFun_Bio_Sensor_Hub::setPulseWidth(uint16_t width)
{
    uint8_t bits;
    uint8_t regVal;

    switch(width)
    {
        case 69:    bits = 0; break;
        case 118:   bits = 1; break;
        case 215:   bits = 2; break;
        case 411:   bits = 3; break;
        default:    return INCORR_PARAM;
    }

    // get current register value to not overwrite anything
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= PULSE_MASK; // mask bits to change
    regVal |= bits;       // add bits

    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal);

    return SFE_BIO_SUCCESS;
}

uint16_t SparkFun_Bio_Sensor_Hub::readPulseWidth()
{
    uint8_t regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_PULSE_MASK;

    switch(regVal)
    {
        case 0: return 69;
        case 1: return 118;
        case 2: return 215;
        case 3: return 411;
        default: return ERR_UNKNOWN;
    }
}

uint8_t SparkFun_Bio_Sensor_Hub::setSampleRate(uint16_t sampleRate)
{
    uint8_t bits;
    uint8_t regVal;

    // making sure the correct sample rate was selected

    switch(sampleRate)
    {
        case 50:    bits = 0; break;
        case 100:   bits = 1; break;
        case 200:   bits = 2; break;
        case 400:   bits = 3; break;
        case 800:   bits = 4; break;
        case 1000:  bits = 5; break;
        case 1600:  bits = 6; break;
        case 3200:  bits = 7; break;
        default:    return INCORR_PARAM;
    }

    // read-modify-write
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= SAMP_MASK;
    regVal |= (bits << 2);

    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal);

    return SFE_BIO_SUCCESS;
}

uint16_t SparkFun_Bio_Sensor_Hub::readSampleRate()
{
    // read the register and isolate bits
    uint8_t regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_SAMP_MASK;
    regVal = (regVal >> 2);

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

uint8_t SparkFun_Bio_Sensor_Hub::setAdcRange(uint16_t adcVal)
{
    uint8_t bits;
    uint8_t regVal;

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

uint16_t SparkFun_Bio_Sensor_Hub::readAdcRange()
{
    uint8_t regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_ADC_MASK;
    regVal = (regVal >> 5);

    switch(regVal)
    {
        case 0: return 2048;
        case 1: return 4096;
        case 2: return 8192;
        case 3: return 16384;
        default: return ERR_UNKNOWN;
    }
}

uint8_t SparkFun_Bio_Sensor_Hub::getMcuType()
{
    return readByte(IDENTITY, READ_MCU_TYPE, NO_WRITE);
}

int32_t SparkFun_Bio_Sensor_Hub::getBootloaderInf()
{
    int32_t bootVers = 0;
    const size_t sizeOfRev = 4;
    int32_t revNum[sizeOfRev] = {0};

    uint8_t status = readMultipleBytes(BOOTLOADER_INFO, BOOTLOADER_VERS, 0x00, sizeOfRev, revNum);

    if(status != SFE_BIO_SUCCESS)
    {
        return ERR_UNKNOWN;
    }

    bootVers |= ((int32_t)revNum[1] << 16);
    bootVers |= ((int32_t)revNum[2] << 8);
    bootVers |= (int32_t)revNum[3];

    return bootVers;
}

uint8_t SparkFun_Bio_Sensor_Hub::max30101Control(uint8_t enable)
{
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

uint8_t SparkFun_Bio_Sensor_Hub::readMAX30101State()
{
    return readByte(READ_SENSOR_MODE, READ_ENABLE_MAX30101);
}

uint8_t SparkFun_Bio_Sensor_Hub::setOutputMode(uint8_t outputType)
{
    if(outputType > SENSOR_ALGO_COUNTER)
    {
        return INCORR_PARAM;
    }

    return writeByte(OUTPUT_MODE, SET_FORMAT, outputType);
}

uint8_t SparkFun_Bio_Sensor_Hub::setFifoThreshold(uint8_t threshold)
{
    return writeByte(OUTPUT_MODE, WRITE_SET_THRESHOLD, threshold);
}

uint8_t SparkFun_Bio_Sensor_Hub::numSamplesOutFifo()
{
    return readByte(READ_DATA_OUTPUT, NUM_SAMPLES);
}

uint8_t *SparkFun_Bio_Sensor_Hub::getDataOutFifo(uint8_t data[])
{
    uint8_t samples = numSamplesOutFifo();
    readFillArray(READ_DATA_OUTPUT, READ_DATA, samples, data);
    return data;
}

uint8_t SparkFun_Bio_Sensor_Hub::numSamplesExternalSensor()
{
    return readByte(READ_DATA_INPUT, SAMPLE_SIZE, WRITE_ACCELEROMETER);
}

void SparkFun_Bio_Sensor_Hub::writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal)
{
    writeByte(WRITE_REGISTER, WRITE_MAX30101, regAddr, regVal);
}

void SparkFun_Bio_Sensor_Hub::writeRegisterAccel(uint8_t regAddr, uint8_t regVal)
{
    writeByte(WRITE_REGISTER, WRITE_ACCELEROMETER, regAddr, regVal);
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30101(uint8_t regAddr)
{
    return readByte(READ_REGISTER, READ_MAX30101, regAddr);
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterAccel(uint8_t regAddr)
{
    return readByte(READ_REGISTER, READ_ACCELEROMETER, regAddr);
}

sensorAttr SparkFun_Bio_Sensor_Hub::getAfeAttributesMAX30101()
{
    sensorAttr maxAttr = {0};
    uint8_t tempArray[2]{};

    uint8_t status = readFillArray(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30101, 2, tempArray);

    if(status == SFE_BIO_SUCCESS)
    {
        maxAttr.byteWord = tempArray[0];
        maxAttr.availRegisters = tempArray[1];
    }

    return maxAttr;
}

sensorAttr SparkFun_Bio_Sensor_Hub::getAfeAttributesAccelerometer()
{
    sensorAttr accelAttr = {0};
    uint8_t tempArray[2]{};

    uint8_t status = readFillArray(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_ACCELEROMETER, 2, tempArray);

    if(status == SFE_BIO_SUCCESS){
        accelAttr.byteWord = tempArray[0];
        accelAttr.availRegisters = tempArray[1];
    }

    return accelAttr;
}

uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterMAX30101(uint8_t regArray[])
{
    return readFillArray(DUMP_REGISTERS, DUMP_REGISTER_MAX30101, 36, regArray);
}

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
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = BOOTLOADER_FLASH;
    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = ERASE_FLASH;

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(10000); // 10ms @ 1MHz

    // read back the single status byte
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    uint8_t statusByte = UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP);

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

    uint8_t status = readMultipleBytes(IDENTITY, READ_SENSOR_HUB_VERS, 0x00, 3, versionArray);

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
    // write sequence
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _enableByte;

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    // specific hub processing delays
    if(_familyByte == ENABLE_SENSOR && _indexByte == ENABLE_MAX30101)
    {
        __delay_cycles(45000); // 45ms @ 1MHz
    }
    else if(_familyByte == ENABLE_ALGORITHM && _indexByte == ENABLE_AGC_ALGO)
    {
        __delay_cycles(10000); // 10ms @ 1MHz
    }
    else if(_familyByte == ENABLE_ALGORITHM && _indexByte == ENABLE_WHRM_ALGO)
    {
        __delay_cycles(50000); // 50ms @ 1MHz
    }
    
    // read back the single status byte
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    uint8_t statusByte = UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP);

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    uint8_t statusByte = 0xFF;

    // write the command
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read back the single status byte
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP);

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeByteTwo)
{
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

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read status bytes
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP);

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val)
{
    uint8_t statusByte = 0xFF;

    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    // split the 16-bit value into two 8-bit chunks
    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = (uint8_t)(_val >> 8); // MSB
    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = (uint8_t)(_val & 0xFF); // LSB

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read status bytes
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::writeLongBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, int32_t _writeVal[], const size_t _len)
{
    uint8_t statusByte = 0xFF;

    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    for (size_t i = 0; i < _len; i++)
    {
        for (uint8_t shift = 24; shift >= 0; shift -= 8)
        {
            while (!(UCB0IFG & UCTXIFG));
            UCB0TXBUF = (uint8_t)(_writeVal[i] >> shift);
        }
    }

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read status bytes
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::writeBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal[], const size_t _len)
{
    uint8_t statusByte = 0xFF;

    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    for (size_t i = 0; i < _len; i++)
    {
        while (!(UCB0IFG & UCTXIFG))
            ;
        UCB0TXBUF = _writeVal[i];
    }

    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read status bytes
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP; // set stop

    while (!(UCB0IFG & UCRXIFG));
    statusByte = UCB0RXBUF;

    return statusByte;
}

// 2 byte version
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte)
{
    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 1\n");

    if (UCB0STATW & UCBBUSY)
    {
        UCB0CTLW0 |= UCSWRST;   // put in reset
        UCB0CTLW0 &= ~UCSWRST;  // release reset
    }
    
    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 2\n");

    uint8_t returnByte = 0xFF;
    uint8_t statusByte = 0x00;

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 3\n");

    // write the family & index byte
    UCB0I2CSA = this->_address;     // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;    // transmitter mode & start condition

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 4\n");

    while(UCB0CTLW0 & UCTXSTT);
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTLW0 |= UCTXSTP;
        UCB0IFG &= ~UCNACKIFG;  // clear flag
        return 0xEE;
    }
    
    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 5\n");

    while (!(UCB0IFG & UCTXIFG));   // wait for tx buffer
    UCB0TXBUF = _familyByte;        // send family byte

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 6 (transmitted family byte)\n");

    while (!(UCB0IFG & UCTXIFG));   // wait for tx buffer
    UCB0TXBUF = _indexByte;         // send index byte

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 7 (transmitted index byte)\n");

    while (UCB0CTLW0 & UCTXSTT);    // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP;           // generate stop condition
    while (UCB0CTLW0 & UCTXSTP);    // wait for stop to finish

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 8\n");

    __delay_cycles(2000); // 2ms delay @ 1MHz

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 9\n");

    // read back status bit
    UCB0CTLW0 &= ~UCTR;   // receiver mode
    UCB0CTLW0 |= UCTXSTT; // start condition

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 10\n");

    while(UCB0CTLW0 & UCTXSTT);
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTLW0 |= UCTXSTP;
        UCB0IFG &= ~UCNACKIFG;  // clear flag
        return 0xEE;
    }
    
    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 11\n");

    // read status byte
    while (!(UCB0IFG & UCTXIFG));   // wait until interrupt
    statusByte = UCB0RXBUF;         // obtain status byte

    if(DEBUG)
    {
        uart_write_string("\treadByte (2 bytes) part 12 statusByte = ");
        uart_write_int(statusByte);
        uart_write_string("\n");
    }

    // read return byte
    UCB0CTLW0 |= UCTXSTP;           // set stop before reading the last byte
    while (!(UCB0IFG & UCTXIFG));   // wait until interrupt
    returnByte = UCB0RXBUF;         // obtain return byte

    if(DEBUG)
    {
        uart_write_string("\treadByte (2 bytes) part 13 returnByte = ");
        uart_write_int(returnByte);
        uart_write_string("\n");
    }


    while (UCB0CTLW0 & UCTXSTP);    // wait for stop to finish

    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 14\n");

    // check if the statusByte isn't 0x00 to return the error

    if (statusByte != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\treadByte (2 bytes) part 15 (ERROR)\n");
        return statusByte;
    }
    
    if(DEBUG) uart_write_string("\treadByte (2 bytes) part 15 (SUCCESS)\n");

    return returnByte;
}

// similar to above, but we just add the write
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 1\n");

    if (UCB0STATW & UCBBUSY)
    {
        UCB0CTLW0 |= UCSWRST;   // put in reset
        UCB0CTLW0 &= ~UCSWRST;  // release reset
    }

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 2\n");    
    
    uint8_t statusByte = 0xFF;
    uint8_t returnByte = 0x00;

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 3\n");

    // write the three bytes
    UCB0I2CSA = this->_address;     // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;    // transmitter mode & start condition

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 4\n");

    while(UCB0CTLW0 & UCTXSTT);
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTLW0 |= UCTXSTP;
        UCB0IFG &= ~UCNACKIFG;  // clear flag
        return 0xEE;
    }

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 5\n");

    while (!(UCB0IFG & UCTXIFG));   // wait for tx buffer
    UCB0TXBUF = _familyByte;        // send family byte

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 6 (transmitted family byte)\n");

    while (!(UCB0IFG & UCTXIFG));   // wait for tx buffer
    UCB0TXBUF = _indexByte;         // send index byte

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 7 (transmitted index byte)\n");

    while (!(UCB0IFG & UCTXIFG));   // wait for tx buffer
    UCB0TXBUF = _writeByte;         // send write byte

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 8 (transmitted write byte)\n");

    while (!(UCB0IFG & UCTXIFG));    // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP;           // generate stop condition
    while (UCB0CTLW0 & UCTXSTP);    // wait for stop to finish

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 9\n");

    __delay_cycles(2000); // 2ms delay @ 1MHz

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 10\n");

    // read the status + data
    UCB0CTLW0 &= ~UCTR;     // receiver mode
    UCB0CTLW0 |= UCTXSTT;   // start condition

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 11\n");

    while(UCB0CTLW0 & UCTXSTT);
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTLW0 |= UCTXSTP;
        UCB0IFG &= ~UCNACKIFG;  // clear flag
        return 0xEE;
    }

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 12\n");

    // read status byte
    while (!(UCB0IFG & UCRXIFG));   // wait until interrupt
    statusByte = UCB0RXBUF;         // obtain status byte

    if(DEBUG)
    {
        uart_write_string("\treadByte (3 bytes) part 13 statusByte = ");
        uart_write_int(statusByte);
        uart_write_string("\n");
    }

    // read return byte
    UCB0CTLW0 |= UCTXSTP;           // set stop before reading the last byte
    while (!(UCB0IFG & UCRXIFG));   // wait until interrupt
    returnByte = UCB0RXBUF;         // obtain return byte
    
    if(DEBUG)
    {
        uart_write_string("\treadByte (3 bytes) part 14 returnByte = ");
        uart_write_int(returnByte);
        uart_write_string("\n");
    }
    
    while (UCB0CTLW0 & UCTXSTP);    // wait for stop to finish

    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 15\n");

    // check if the statusByte isn't 0x00 to return the error

    if (statusByte != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\treadByte (3 bytes) part 16 (ERROR)\n");
        return statusByte;
    }
    
    if(DEBUG) uart_write_string("\treadByte (3 bytes) part 16 (SUCCESS)\n");

    return returnByte;
}

uint16_t SparkFun_Bio_Sensor_Hub::readIntByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{

    uint16_t returnVal = 0;
    uint8_t statusByte = 0xFF;

    // write the three bytes
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    while (UCB0CTLW0 & UCTXSTT);                 // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP; // generate stop condition
    while (UCB0CTLW0 & UCTXSTP); // wait for stop to finish

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read the status + data
    UCB0CTLW0 &= ~UCTR;   // receiver mode
    UCB0CTLW0 |= UCTXSTT; // start condition

    // read status byte
    while (!(UCB0IFG & UCTXIFG)); // wait until interrupt
    statusByte = UCB0RXBUF;

    // check for an error
    if (statusByte != SFE_BIO_SUCCESS)
    {
        UCB0CTLW0 |= UCTXSTP;
        return 0;
    }

    // read high byte
    while (!(UCB0IFG & UCRXIFG));
    returnVal = (uint16_t)UCB0RXBUF << 8;

    // read low byte
    UCB0CTLW0 |= UCTXSTP;
    while (!(UCB0IFG & UCRXIFG));
    returnVal |= UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP);

    return returnVal;
}

// uses uint8_t user array
uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _len, uint8_t userArray[])
{
    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 1\n");

    uint8_t statusByte = 0xFF;

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 2\n");

    // write the three bytes
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 3\n");

    while(UCB0CTLW0 & UCTXSTT);
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTLW0 |= UCTXSTP;
        UCB0IFG &= ~UCNACKIFG;  // clear flag
        return 0xEE;
    }

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 4 (transmitted family byte)\n");

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 5 (transmitted index byte)\n");

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 6 (transmitted write byte)\n");

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 7\n");

    while (!(UCB0IFG & UCTXIFG));    // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP;           // generate stop condition
    while (UCB0CTLW0 & UCTXSTP);    // wait for stop to finish

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 8\n");

    __delay_cycles(2000); // 2ms delay @ 1MHz

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 9\n");

    // read the status + data
    UCB0CTLW0 &= ~UCTR;   // receiver mode
    UCB0CTLW0 |= UCTXSTT; // start condition

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 10\n");

    while(UCB0CTLW0 & UCTXSTT);
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTLW0 |= UCTXSTP;
        UCB0IFG &= ~UCNACKIFG;  // clear flag
        return 0xEE;
    }

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 11\n");

    // read status byte
    while (!(UCB0IFG & UCRXIFG)); // wait until interrupt
    statusByte = UCB0RXBUF;

    if(DEBUG)
    {
        uart_write_string("\treadMultipleBytes (uint8_t) part 12 statusByte = ");
        uart_write_int(statusByte);
        uart_write_string("\n");
    }

    // check for an error
    if (statusByte != SFE_BIO_SUCCESS)
    {
        if(DEBUG) uart_write_string("\treadMultipleBytes (ERROR) statusByte != SFE_BIO_SUCCESS\n");
        UCB0CTLW0 |= UCTXSTP;
        while(!(UCB0IFG & UCRXIFG));
        uint8_t dummy = UCB0RXBUF;
        while(UCB0CTLW0 & UCTXSTP);
        return statusByte;
    }

    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 13\n");
    
    for (size_t i = 0; i < _len; i++)
    {
        if (i == (_len - 1))
        {
            UCB0CTLW0 |= UCTXSTP; // stop before last byte
        }
        while (!(UCB0IFG & UCRXIFG));
        userArray[i] = UCB0RXBUF;
        if(DEBUG)
        {
            uart_write_string("\treadMultipleBytes (uint8_t) finished loop userArray[i] = ");
            uart_write_int(userArray[i]);
            uart_write_string("\n");
        }
    }
    
    if(DEBUG) uart_write_string("\treadMultipleBytes (uint8_t) part 14\n");

    while(UCB0CTLW0 & UCTXSTP);
    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _len, int32_t userArray[])
{
    uint8_t statusByte = 0xFF;

    // write the three bytes
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _writeByte;

    while (UCB0CTLW0 & UCTXSTT);                 // wait for start bit to clear
    UCB0CTLW0 |= UCTXSTP; // generate stop condition
    while (UCB0CTLW0 & UCTXSTP); // wait for stop to finish

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read the status + data
    UCB0CTLW0 &= ~UCTR;   // receiver mode
    UCB0CTLW0 |= UCTXSTT; // start condition

    // read status byte
    while (!(UCB0IFG & UCTXIFG)); // wait until interrupt
    statusByte = UCB0RXBUF;

    // check for an error
    if (statusByte != SFE_BIO_SUCCESS)
    {
        UCB0CTLW0 |= UCTXSTP;
        return statusByte;
    }

    // each 32-bit int is sent as 4 bytes big-endian
    for (size_t i = 0; i < _len; i++)
    {
        userArray[i] = 0;
        for (uint8_t j = 0; j < 4; j++)
        {
            // check if its the last byte
            if (i == (_len - 1) && j == 3)
            {
                UCB0CTLW0 |= UCTXSTP;
            }
            while (!(UCB0IFG & UCRXIFG));
            userArray[i] = (userArray[i] << 8) | UCB0RXBUF;
        }
    }

    return statusByte;
}

uint8_t SparkFun_Bio_Sensor_Hub::readFillArray(uint8_t _familyByte, uint8_t _indexByte, uint8_t arraySize, uint8_t array[])
{

    // write request
    UCB0I2CSA = this->_address;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _familyByte;
    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = _indexByte;
    while (UCB0CTLW0 & UCTXSTT);
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);

    __delay_cycles(2000); // 2ms delay @ 1MHz

    // read response
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;

    while (!(UCB0IFG & UCRXIFG));
    uint8_t statusByte = UCB0RXBUF; // first byte is always status

    if (statusByte != SFE_BIO_SUCCESS)
    {
        UCB0CTLW0 |= UCTXSTP;
        return statusByte;
    }

    for (uint8_t i = 0; i < arraySize; i++)
    {
        if (i == (arraySize - 1))
        {
            UCB0CTLW0 |= UCTXSTP;
        }
        while (!(UCB0IFG & UCRXIFG))
            ;
        array[i] = UCB0RXBUF;
    }

    return statusByte;
}

// creating UART for debugging since printing doesn't seem to work
void initUART()
{
    // configure uart pins, 1.4 = TX & 1.5 = RX
    P1SEL0 |= BIT4 | BIT5;
    //    P1SEL0 &= ~(BIT4 | BIT5);

    // configure uart for 9600 baud
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
        if (*str == '\n') {
            uart_write_char('\r');
        }
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

void uart_clear_screen() {
    // ESC [ 2 J  = Clear entire screen
    // ESC [ H    = Move cursor to home (top-left)
    uart_write_string("\x1B[2J\x1B[H"); 
}

// Helper to set pins to a "High" state using internal pull-ups
void b_high(uint8_t pin) {
    P1DIR &= ~pin; // Set to input
    P1OUT |= pin;  // Ensure pull-up is active
}

// Helper to set pins to a "Low" state
void b_low(uint8_t pin) {
    P1OUT &= ~pin; // Write 0
    P1DIR |= pin;  // Set to output
}

void b_delay() { __delay_cycles(200); } // ~2.5kHz for maximum stability with weak pull-ups

void b_start() {
    b_high(SDA); b_high(SCL); b_delay();
    b_low(SDA); b_delay();
    b_low(SCL); b_delay();
}

void b_stop() {
    b_low(SDA); b_delay();
    b_high(SCL); b_delay();
    b_high(SDA); b_delay();
}

bool b_write(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) b_high(SDA);
        else b_low(SDA);
        b_delay();
        b_high(SCL); b_delay();
        b_low(SCL); b_delay();
        data <<= 1;
    }
    // Check for ACK
    b_high(SDA); b_delay();
    b_high(SCL); b_delay();
    bool ack = !(P1IN & SDA); // If SDA is pulled low, we have an ACK
    b_low(SCL); b_delay();
    return ack;
}

uint8_t runI2CScanner() {

    uart_write_string("Hardware Scan Starting...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        UCB0I2CSA = addr;
        UCB0CTLW0 |= UCTR | UCTXSTT; // Start + Write

        uint32_t timeout = 5000;
        while ((UCB0CTLW0 & UCTXSTT) && --timeout); // Wait for Address

        if (timeout == 0) {
            // Bus is physically stuck - Reset the whole module
            UCB0CTLW0 |= UCSWRST;
            UCB0CTLW0 &= ~UCSWRST;
            continue;
        }

        if (!(UCB0IFG & UCNACKIFG)) { // SUCCESS: Device ACKed
            uart_write_string("Found i2c device at: ");
            uart_write_int(addr);
            uart_write_string("\r\n");

            // clean-up
            UCB0CTLW0 |= UCTXSTP;

            uint32_t stopTimeout = 1000;
            while((UCB0CTLW0 & UCTXSTP) && --stopTimeout);

            UCB0IFG &= ~UCNACKIFG;

            if(stopTimeout == 0)
            {
                UCB0CTLW0 |= UCSWRST;
                UCB0CTLW0 &= ~UCSWRST;
            }

            uart_write_string("Returning 1\n");
            return 1;
        }

        UCB0CTLW0 |= UCTXSTP;      // Always send STOP to release bus
        while (UCB0CTLW0 & UCTXSTP);
        UCB0IFG &= ~UCNACKIFG;     // Clear NACK flag for next loop
    }

    return 0;
}

void runI2CScannerBitBang()
{
    uart_write_string("Starting Bit-Bang I2C Scan...\r\n");

    // Ensure pins are GPIO and have internal pull-ups enabled
    P1SEL0 &= ~(SDA | SCL);
    P1SEL1 &= ~(SDA | SCL);
    P1REN |= (SDA | SCL);
    P1OUT |= (SDA | SCL);

    P1OUT |= BIT0; // Turn on LED

    for(uint8_t addr = 1; addr < 127; addr++)
    {
        // I2C Write Address = (addr << 1) + 0
        b_start();
        bool found = b_write(addr << 1);
        b_stop();

        if(found)
        {
            uart_write_string("Found bit-bang device at: ");
            uart_write_int(addr);
            uart_write_string("\r\n");
        }

        __delay_cycles(1000); // Small rest between pings
    }

    P1OUT &= ~BIT0; // Turn off LED
    uart_write_string("Scan Complete.\r\n");
}

int main(void)
{
    // stop watchdog & gpio high-impedance
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // init uart
    initUART();
    uart_clear_screen();
    __delay_cycles(100000);
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
        __delay_cycles(100);
        P1OUT |= BIT3;
        __delay_cycles(100);
    }

    uart_write_string("I2C Bus Recovered (ideally).\n");

    // switch to hardware i2c mode
    P1SEL0 |= (BIT2 | BIT3);
    UCB0CTLW0 = UCSWRST; // enable reset
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC | UCSSEL__SMCLK | UCSWRST;
    UCB0BRW = 10;
    UCB0CTLW0 &= ~UCSWRST;

    uint8_t tester = runI2CScanner();

    // hardware scan
    if(tester == 0)
    {
        uart_write_string("CRITICAL: Hardware I2C could not find sensor.\n");
        runI2CScannerBitBang(); // last resort
        return;
    }
    else
    {
        uart_write_string("We exited the scanner!\n");
    }

    // init library

    uart_write_string("\n--- Initializing Bio Sensor ---\n");

    // create sensor instance
    SparkFun_Bio_Sensor_Hub bioHub(&P2DIR, &P2OUT, BIT0,
                                   &P2DIR, &P2OUT, &P2REN, BIT1,
                                   BIO_ADDRESS);

    uart_write_string("Created a sensor instance. Beginning sensor.\n");

    uint8_t result = bioHub.begin();

    // check if we got a response
    if(result == 0x00) {
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

        uint8_t error = bioHub.configBpm(MODE_ONE);

        if(error == SFE_BIO_SUCCESS) uart_write_string("Sensor Configured!");
        else uart_write_string("Error in configuring sensor.");
    }
    else {
        uart_write_string("Error: Initialization failed. Code: ");
        uart_write_int(result);
        uart_write_string("\r\n");
    }

    while(1); // End of test
}
