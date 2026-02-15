#include <stdint.h>
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//         THESE VALUES COME FROM THE SPARKFUN MAX32664 USER GUIDE           //
//                                                                           //
//  ADDITIONALLY, THIS IS HEAVILY BASED ON:                                  //
//  https://github.com/sparkfun/SparkFun_Bio_Sensor_Hub_Library/tree/master  //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

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

#define MAXFAST_ARRAY_SIZE 6 // Number of bytes...
#define MAXFAST_EXTENDED_DATA 5
#define MAX30101_LED_ARRAY 12 // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT 0x00
#define READ_FORMAT 0x01         // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD 0x01 // Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

const uint8_t BIO_ADDRESS = 0x55; // address for I2C

// the struct that holds all the important data
struct bioData
{
    uint32_t irLed;
    uint32_t redLed;
    // uint32_t greenLed;
    uint16_t heartRate; // Least Significant Bit = 0.1bpm
    uint8_t confidence; // 0 -> 100% LSB = 1%
    uint16_t oxygen;    // 0 -> 100% LSB = 1%
    uint8_t status;     // 0: Success, 1: Not Ready, 2: Object Detected, 3: Finger Detected
    float rValue;       // -- Algorithm Mode 2 vv
    uint8_t extStatus;  // --
    uint8_t reserveOne; // --
    uint8_t reserveTwo; // -- Algorithm Mode 2 ^^
};

struct version
{
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
};

struct sensorAttr
{
    uint8_t byteWord;
    uint8_t availRegisters;
};

// these error codes & names come from the MAX32664 user guide,
enum READ_STATUS_BYTE_VALUE
{
    SFE_BIO_SUCCESS = 0x00,   // success!
    ERR_UNAVAIL_CMD,          // illegal Family Byte and/or Command Byte was used
    ERR_UNAVAIL_FUNC,         // function is not implemented
    ERR_DATA_FORMAT,          // incorrect number of bytes sent for the requested Family Byte
    ERR_INPUT_VALUE,          // illegal configuration value was attempted to be set
    ERR_TRY_AGAIN,            // device is busy, try again
    ERR_BTLDR_GENERAL = 0x80, // general error while receiving/flashing a page during the bootloader sequence
    ERR_BTLDR_CHECKSUM,       // checksum error while decrypting/checking page data
    ERR_BTLDR_AUTH,           // authorization error
    ERR_BTLDR_INVALID_APP,    // application not valid
    ERR_UNKNOWN = 0xFF        // unknown error
};

// common bytes used to tell the MAX32664 what operation we're performing
enum FAMILY_REGISTER_BYTES
{
    HUB_STATUS = 0x00,              // read sensor hub status
    SET_DEVICE_MODE,                // select the device operating mode. (app must implement this) (see enum DEVICE_MODE_WRITE_BYTES below)
    READ_DEVICE_MODE,               // read the device operating mode
    OUTPUT_MODE = 0x10,             // set the output format of the sensor hub (see enum OUTPUT_MODE_WRITE_BYTE below)
    READ_OUTPUT_MODE,               // read the device output mode
    READ_DATA_OUTPUT,               // get the number of samples available in FIFO (see enum FIFO_OUTPUT_INDEX_BYTE below)
    READ_DATA_INPUT,                // read input for external sensor (see enum FIFO_EXTERNAL_INDEX_BYTE below)
    WRITE_INPUT,                    // index byte is 0x04
    WRITE_REGISTER = 0x40,          // (see enum WRITE_REGISTER_INDEX_BYTE below)
    READ_REGISTER,                  // (see enum READ_REGISTER_INDEX_BYTE below)
    READ_ATTRIBUTES_AFE,            // (see enum GET_AFE_INDEX_BYTE below)
    DUMP_REGISTERS,                 // (see enum DUMP_REGISTER_INDEX_BYTE below)
    ENABLE_SENSOR,                  // (see enum SENSOR_ENABLE_INDEX_BYTE below)
    READ_SENSOR_MODE,               // (see enum READ_SENSOR_ENABLE_INDEX_BYTE below)
    CHANGE_ALGORITHM_CONFIG = 0x50, // (see enum ALGORITHM_CONFIG_INDEX_BYTE, enum ALGO_AGC_WRITE_BYTE, and enum ALGO_BPT_WRITE_BYTE below)
    READ_ALGORITHM_CONFIG,          // (see enum READ_ALGORITHM_INDEX_BYTE and enum READ_AGC_ALGO_WRITE_BYTE below)
    ENABLE_ALGORITHM,               // (see enum ALGORITHM_MODE_ENABLE_INDEX_BYTE below)
    BOOTLOADER_FLASH = 0x80,        // (see enum BOOTLOADER_FLASH_INDEX_BYTE below)
    BOOTLOADER_INFO,                // (see enum BOOTLOADER_INFO_INDEX_BYTE below)
    IDENTITY = 0xFF                 // (see enum IDENTITY_INDEX_BYTES below)
};

// index bytes for SET_DEVICE_MODE (0x01)
enum DEVICE_MODE_WRITE_BYTES
{
    EXIT_BOOTLOADER = 0x00,
    SFE_BIO_RESET = 0x02,
    ENTER_BOOTLOADER = 0x08
};

// index bytes for OUTPUT_MODE (0x10)
enum OUTPUT_MODE_WRITE_BYTE
{
    PAUSE = 0x00, // (no data)
    SENSOR_DATA,
    ALGO_DATA,
    SENSOR_AND_ALGORITHM, // combine the above two
    PAUSE_TWO,            // (no data two)
    SENSOR_COUNTER_BYTE,  // sample counter byte
    ALGO_COUNTER_BYTE,    // sample counter byte
    SENSOR_ALGO_COUNTER   // combine the above two
};

// index bytes for READ_DATA_OUTPUT (0x12)
enum FIFO_OUTPUT_INDEX_BYTE
{
    NUM_SAMPLES = 0x00, // get number of available samples
    READ_DATA           // read data stored in output
};

// index bytes for READ_DATA_INPUT (0x13)
enum FIFO_EXTERNAL_INDEX_BYTE
{
    SAMPLE_SIZE = 0x00,     // sensor sample size
    READ_INPUT_DATA,        // read input FIFO size for the maximum number of samples that the input can hold
    READ_SENSOR_DATA,       // read sensor FIFO size for the maximum number of samples that the sensor can hold (for external accelerometer)
    READ_NUM_SAMPLES_INPUT, // read the number of samples currently in the input (for external accelerometer)
    READ_NUM_SAMPLES_SENSOR // read the number of samples currently in the sensor
};

// index bytes for WRITE_REGISTER (0x40) (not all max sensors needed for this specific project)
enum WRITE_REGISTER_INDEX_BYTE
{
    WRITE_MAX86140 = 0x00,
    WRITE_MAX30205,
    WRITE_MAX30001,
    WRITE_MAX30101,
    WRITE_ACCELEROMETER
};

// index bytes for READ_REGISTER (0x41)
enum READ_REGISTER_INDEX_BYTE
{
    READ_MAX86140 = 0x00,
    READ_MAX30205,
    READ_MAX30001,
    READ_MAX30101,
    READ_ACCELEROMETER
};

// index bytes for READ_ATTRIBUTES_AFE (0x42)
enum GET_AFE_INDEX_BYTE
{
    RETRIEVE_AFE_MAX86140 = 0x00,
    RETRIEVE_AFE_MAX30205,
    RETRIEVE_AFE_MAX30001,
    RETRIEVE_AFE_MAX30101,
    RETRIEVE_AFE_ACCELEROMETER
};

// index bytes for DUMP_REGISTERS (0x43)
enum DUMP_REGISTER_INDEX_BYTE
{
    DUMP_REGISTER_MAX86140 = 0x00,
    DUMP_REGISTER_MAX30205,
    DUMP_REGISTER_MAX30001,
    DUMP_REGISTER_MAX30101,
    DUMP_REGISTER_ACCELEROMETER
};

// index byte for ENABLE_SENSOR (0x44)
// 0x00 = disable, 0x01 = enable for the write bytes
enum SENSOR_ENABLE_INDEX_BYTE
{
    ENABLE_MAX86140 = 0x00,
    ENABLE_MAX30205,
    ENABLE_MAX30001,
    ENABLE_MAX30101,
    ENABLE_ACCELEROMETER
};

// index byte for READ_SENSOR_MODE (0x45)
// these don't actually appear in the user guide
enum READ_SENSOR_ENABLE_INDEX_BYTE
{
    READ_ENABLE_MAX86140 = 0x00,
    READ_ENABLE_MAX30205,
    READ_ENABLE_MAX30001,
    READ_ENABLE_MAX30101,
    READ_ENABLE_ACCELEROMETER
};

// index byte for CHANGE_ALGORITHM_CONFIG (0x50)
enum ALGORITHM_CONFIG_INDEX_BYTE
{
    SET_TARG_PERC = 0x00, // each of these have write bytes associated
    SET_STEP_SIZE = 0x00,
    SET_SENSITIVITY = 0x00,
    SET_AVG_SAMPLES = 0x00,
    SET_PULSE_OX_COEF = 0x02,
    BPT_CONFIG = 0x04
};

// extension of CHANGE_ALGORITHM_CONFIG (0x50) for SET_TARG_PERC -> SET_AVG_SAMPLES
enum ALGO_AGC_WRITE_BYTE
{

    AGC_GAIN_ID = 0x00,      // 0 -> 100
    AGC_STEP_SIZE_ID,        // 0 -> 100
    AGC_SENSITIVITY_ID,      // 0 -> 100
    AGC_NUM_SAMP_ID,         // 0 -> 255
    MAXIMFAST_COEF_ID = 0x0B // 3, signed, 32-bit integers
};

// extension of CHANGE_ALGORITHM_CONFIG (0x50)
enum ALGO_BPT_WRITE_BYTE
{
    BPT_MEDICATION = 0x00,  // 0x00, 0x00: not using blood pressure. 0x00, 0x01: using blood pressure
    DIASTOLIC_VALUE,        // returns 3 values
    SYSTOLIC_VALUE,         // returns 3 values
    BPT_CALIB_DATA,         // Index + 824 bytes of calibration data
    PATIENT_RESTING = 0x05, // 0x05, 0x00: resting. 0x05, 0x01: not resting
    AGC_SP02_COEFS = 0x0B
};

// index byte for READ_ALGORITHM_CONFIG (0x51)
enum READ_ALGORITHM_INDEX_BYTE
{

    READ_AGC_PERCENTAGE = 0x00,
    READ_AGC_STEP_SIZE = 0x00,
    READ_AGC_SENSITIVITY = 0x00,
    READ_AGC_NUM_SAMPLES = 0x00,
    READ_MAX_FAST_COEF = 0x02
};

// extension of READ_ALGORITHM_CONFIG (0x51) for READ_AGC_PERCENTAGE -> READ_AGC_NUM_SAMPLES
enum READ_AGC_ALGO_WRITE_BYTE
{

    READ_AGC_PERC_ID = 0x00,
    READ_AGC_STEP_SIZE_ID,
    READ_AGC_SENSITIVITY_ID,
    READ_AGC_NUM_SAMPLES_ID,
    READ_MAX_FAST_COEF_ID = 0x0B
};

// index byte for NABLE_ALGORITHM (0x52)
enum ALGORITHM_MODE_ENABLE_INDEX_BYTE
{
    ENABLE_AGC_ALGO = 0x00,
    ENABLE_WHRM_ALGO = 0x02
};

// index byte for BOOTLOADER_FLASH (0x80)
enum BOOTLOADER_FLASH_INDEX_BYTE
{
    SET_INIT_VECTOR_BYTES = 0x00, // not required for a non-secure bootloader
    SET_AUTH_BYTES,               // not required for a non-secure bootloader
    SET_NUM_PAGES,
    ERASE_FLASH,
    SEND_PAGE_VALUE
};

// index byte for BOOTLOADER_INFO (0x81)
enum BOOTLOADER_INFO_INDEX_BYTE
{
    BOOTLOADER_VERS = 0x00,
    PAGE_SIZE
};

// index byte for IDENTITY (0xFF)
enum IDENTITY_INDEX_BYTES
{
    READ_MCU_TYPE = 0x00,
    READ_SENSOR_HUB_VERS = 0x03,
    READ_ALGO_VERS = 0x07
};

class SparkFun_Bio_Sensor_Hub
{

public:
    //// public variables ////

    uint8_t bpmArr[MAXFAST_ARRAY_SIZE]{};
    uint8_t bpmArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA]{};
    uint8_t senArr[MAX30101_LED_ARRAY]{};
    uint8_t bpmSenArr[MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY]{};
    uint8_t bpmSenArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA + MAX30101_LED_ARRAY]{};

    //// class constructor ////

    SparkFun_Bio_Sensor_Hub(volatile uint8_t *resetPort, volatile uint8_t *resetOut, uint16_t resetBit,
                            volatile uint8_t *mfioPort, volatile uint8_t *mfioOut, volatile uint8_t *mfioRen, uint16_t mfioBit,
                            uint8_t address);

    //// public functions ////

    // initializes the sensor, placing the max32664 into application mode
    uint8_t begin();

    // places the max32664 into bootloader mode
    uint8_t beginBootloader();

    // checks the status of FIFO
    uint8_t readSensorHubStatus();

    // switch the device state & verify
    uint8_t setOperatingMode(uint8_t selection);

    // sets very basic settings to get sensor & biometric data
    uint8_t configBpm(uint8_t mode);

    // sets very basic settings to get LED count values from MAX30101
    uint8_t configSensor();

    // does both config bpm & config sensor
    uint8_t configSensorBpm(uint8_t mode);

    // main logic to read the bpm of user
    bioData readBpm();

    // retrieving raw LED data
    bioData readSensor();

    // combines the two above functions
    bioData readSensorBpm();

    // modified the pulse width of the MAX30101 LEDs (all are set to the same width)
    uint8_t setPulseWidth(uint16_t width);

    // reads the pulse width of MAX30101 LEDs
    uint16_t readPulseWidth();

    // sets the sample rate of the MAX30101
    uint8_t setSampleRate(uint16_t sampleRate);

    // reads the sample rate of the MAX30101
    uint16_t readSampleRate();

    // sets the dynamic range of MAX30101's ADC
    uint8_t setAdcRange(uint16_t adcVal);

    // returns the ADC range
    uint16_t readAdcRange();

    // helper function, just calls readByte
    uint8_t getMcuType();

    // checks the version number of the bootloader on the chip
    int32_t getBootloaderInf();

    // helper function, just calls enableWrite
    uint8_t max30101Control(uint8_t enable);

    // helper function, just calls readByte
    uint8_t readMAX30101State();

    // helper function, just calls enableWrite
    uint8_t accelControl(uint8_t accelSwitch);

    // helper function, just calls writeByte
    uint8_t setOutputMode(uint8_t outputType);

    // helper function, just calls writeByte
    uint8_t setFifoThreshold(uint8_t threshold);

    // helper function, just calls readByte
    uint8_t numSamplesOutFifo();

    // returns the data in fifo
    uint8_t *getDataOutFifo(uint8_t data[]);

    // adds support for the accelerometor
    uint8_t numSamplesExternalSensor();

    // writes the given register value at the given register address
    void writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal);

    // writes the given register value at the given register address
    void writeRegisterAccel(uint8_t regAddr, uint8_t regVal);

    // reads the given register address for the MAX30101 sensor & returns the value
    uint8_t readRegisterMAX30101(uint8_t regAddr);

    // reads the given register address for the MAX30101 sensor
    uint8_t readRegisterAccel(uint8_t regAddr);

    // retrieves the attributes of the analog front end on MAX30101
    sensorAttr getAfeAttributesMAX30101();

    // retrieves attributes of the analog front end on accel
    sensorAttr getAfeAttributesAccelerometer();

    // returns all registers and register values sequentially of the MAX30101
    uint8_t dumpRegisterMAX30101(uint8_t regArray[]);

    // returns all registers and register values sequentially of the accel
    uint8_t dumpRegisterAccelerometer(uint8_t numReg, uint8_t regArray[]);

    // sets the target percentage of full-scale ADC
    uint8_t setAlgoRange(uint8_t perc);

    // changes the step size toward the target for AGC algorithm
    uint8_t setAlgoStepSize(uint8_t step);

    // changes the sensitivity of the agc algo
    uint8_t setAlgoSensitivity(uint8_t sense);

    // changes the number of samples that are averaged
    uint8_t setAlgoSamples(uint8_t avg);

    // takes 3 values, used as the Sp02 coefficients
    uint8_t setMaximFastCoef(int32_t coef1, int32_t coef2, int32_t coef3);

    // reads and returns the currently set target percentage
    uint8_t readAlgoRange();

    // returns the step size toward the target for AGC algo
    uint8_t readAlgoStepSize();

    // returns the sensitivity (percentage) of the automatic gain control
    uint8_t readAlgoSensitivity();

    // helper function, just calls readByte
    uint8_t readAlgoSamples();

    // reads the maximum age for the wrist heart rate monitor
    uint8_t readMaximFastCoef(int32_t coefArr[3]);

    // helper function, just calls enableWrite
    uint8_t agcAlgoControl(uint8_t enable);

    // helper function, just calls enableWrite
    uint8_t maximFastAlgoControl(uint8_t mode);

    // tells the bootloader how many pages of memory to prepare for a flash write
    bool setNumPages(uint8_t totalPages);

    bool eraseFlash();

    version readBootloaderVers();

    version readSensorHubVersion();

    version readAlgorithmVersion();

    // writes medication value
    uint8_t isPatientBPMedication(uint8_t medication);

    // checks for medication value
    uint8_t isPatientBPMedication();

    uint8_t writeDiastolicVals(uint8_t diasVal1, uint8_t diasVal2, uint8_t diasVal3);

    uint8_t readDiastolicVals(uint8_t userArray[]);

    uint8_t writeSystolicVals(uint8_t sysVal1, uint8_t sysVal2, uint8_t sysVal3);

    uint8_t readSystolicVals(uint8_t userArray[]);

    uint8_t writeBPTAlgoData(uint8_t bptCalibData[]);

    uint8_t readBPTAlgoData(uint8_t userArray[]);

    uint8_t isPatientResting(uint8_t resting);

    uint8_t isPatientResting();

    uint8_t writeSP02AlgoCoef(int32_t intA, int32_t intB, int32_t intC);

    uint8_t readSP02AlgoCoef(int32_t userArray[]);

private:
    //// private variables ////

    volatile uint8_t *_resetPort; // port direction register (ex: P1DIR)
    volatile uint8_t *_resetOut;  // port output register (ex: P1OUT)
    uint16_t _resetBit;           // the specific bit (ex: BIT4)

    volatile uint8_t *_mfioPort;
    volatile uint8_t *_mfioOut;
    volatile uint8_t *_mfioRen;
    uint16_t _mfioBit;

    uint8_t _address;

    uint32_t _writeCoefArr[3]{};
    uint8_t _userSelectedMode;
    uint8_t _sampleRate = 100;

    //// private functions ////

    // enables write
    uint8_t enableWrite(uint8_t _familyByte, uint8_t _indexByte, uint8_t _enableByte);

    // writes 3 bytes
    uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);

    // writes 4 bytes (last one being uint8_t)
    uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal);

    // writes 4 bytes (last one being uint16_t)
    uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val);

    // used for Sp02 coefficients
    uint8_t writeLongBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, int32_t _writeVal[], const size_t _len);

    // 8-bit version of previous
    uint8_t writeBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal[], const size_t _len);

    // reads 2 bytes
    uint8_t readByte(uint8_t _familyByte, uint8_t _indexByte);

    // reads 3 bytes
    uint8_t readByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);

    // used when a register holds a 16-bit value
    uint16_t readIntByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);

    // reads a variable number of 8-bit bytes into a buffer, 2 byte sending version
    uint8_t readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, const size_t _len, uint8_t userArray[]);

    // reads a variable number of 8-bit bytes into a buffer
    uint8_t readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _len, uint8_t userArray[]);

    // reads a variable number of 32-bit bytes into a buffer, designed for MaximFast Algorithm Coefficients
    uint8_t readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _len, int32_t userArray[]);

    // reads array & fills the array passed through
    uint8_t readFillArray(uint8_t _familyByte, uint8_t _indexByte, uint8_t arraySize, uint8_t array[]);
};

#endif // _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_