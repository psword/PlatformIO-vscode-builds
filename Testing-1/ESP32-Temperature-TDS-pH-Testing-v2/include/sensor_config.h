// sensor_config.h

#ifndef SENSOR_CONFIG
#define SENSOR_CONFIG

// Pin definitions
#define ONE_WIRE_BUS 2   // Data wire is plugged into port 2
#define TEMP_SENSOR_POWER_PIN 17 // Power Port is GPIO PIN 17
#define TDS_SENSOR_BUS 4 // Data Wire is plugged into port 4
#define TDS_SENSOR_POWER_PIN 16  // Power Port is GPIO PIN 16
#define PH_SENSOR_POWER_PIN 15   // Power Port is GPIO PIN 15
#define PH_SENSOR_BUS 13 // The ADC Pin for the ADS1115 (must be 0, 1, 2, 3)

// Global variables
extern float adjustedTemp;        // Variable to store temperature
// float adjustedTds = 0;         // Variable to store TDS
// float adjustedpH = 0;          // Variable to store pH
// float sendFloatTempValue;      // Float value to send
// float sendFloatTdsValue;       // Float value to send
// float sendFloatpHValue;        // Float value to send
extern bool codeExecuted;     // Flag for code execution
// extern bool sendMessageFlag1 = false; // Flag to indicate relay message
// extern bool sendMessageFlag2 = false; // Flag to indicate relay message
// extern bool sendMessageFlag3 = false; // Flag to indicate relay message


// Define sampling intervals for sensors
#define SAMPLING_INTERVAL 60000 // Interval for sampling (milliseconds)
#define READING_DURATION 12000  // Duration for sampling (milliseconds)
#define uS_TO_S_FACTOR 1000000  // Conversion factor for microseconds to seconds
#define S_TO_MIN_FACTOR 60  // Conversion factor for seconds to minutes
#define TIME_TO_SLEEP 2  // Amount of time to sleep

// Define message types
extern const uint8_t messageType1;   // Message type identifier for Temperature
extern const uint8_t messageType2;   // Message type identifier for TDS
extern const uint8_t messageType3;   // Message type identifier for pH
extern const uint8_t messageLength;  // Length of message to transmit
extern const uint8_t slaveAddress;   // I2C Slave address

#endif