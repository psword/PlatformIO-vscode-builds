/*
* Written for ESP32 dev controller
*/

#include <Arduino.h>
#include <Wire.h>
#include "esp_sleep.h"

// Definitions
#define DEBUGPRINT_INTERVAL 30000 // Interval to print data received from I2C slave (debug print)
#define uS_TO_S_FACTOR 1000000    // Conversion factor for microseconds to seconds
#define S_TO_MIN_FACTOR 60        // Conversion factor for seconds to minutes
#define TIME_TO_SLEEP 2           // Amount of time to sleep in minutes
#define AWAKE_TIME 90             // Time to stay awake (in seconds)
#define BOOT_THRESHOLD 24         // Number of boots before data transmission

// Define variables
uint8_t messageType1 = 0x01;      // Message type identifier
uint8_t messageType2 = 0x02;      // Message type identifier
uint8_t messageType3 = 0x03;      // Message type identifier
uint8_t messageType4 = 0x04;      // Message type identifier
uint8_t messageLength = 8;        // Length of message to transmit
uint8_t slaveAddress = 0x08;      // I2C Slave address
bool flag = false;                // Cloud transmit flag
RTC_DATA_ATTR float receivedFloatTemp; // Float value to receive
RTC_DATA_ATTR float receivedFloatTds;  // Float value to receive
RTC_DATA_ATTR float receivedFloatpH;   // Float value to receive
RTC_DATA_ATTR bool firstBoot = true;   // To check if it's the first boot after deep sleep
RTC_DATA_ATTR uint16_t bootCounter = 0;   // To count the number of boots


// Function prototypes
void receiveData(int bytes);
float readFloatFromWire();
void transmitDataToCloud();

void setup()
{
    Serial.begin(9600); // Start the serial port for debugging
    while (!Serial)
    {
        ; // Wait for the serial port to connect
    }

    // Initialize I2C as a slave on specified pins
    Wire.begin(slaveAddress); // Slave address 0x08
    Wire.onReceive(receiveData); // Register function to handle data reception

    if (!firstBoot) {
        Serial.println("Woke up from deep sleep.");
        bootCounter++; // Increment the boot counter
        Serial.print("Boot Counter : ");
        Serial.println(bootCounter);

    }

    if (firstBoot) {
        firstBoot = false;
        Serial.println("First boot. Entering deep sleep...");
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_MIN_FACTOR *uS_TO_S_FACTOR);
        esp_deep_sleep_start(); // Enter deep sleep
    }
}

void loop()
{
    if(!flag && bootCounter == BOOT_THRESHOLD) {
        transmitDataToCloud();
        bootCounter = 0;
    }
}

void receiveData(int bytes)
{
    Serial.print("Data received. Bytes: ");
    Serial.println(bytes);

    while (Wire.available() >= 2) // Ensure there are at least 2 bytes to read (type and length)
    {
        uint8_t messageType = Wire.read();   // Read message type byte
        uint8_t messageLength = Wire.read(); // Read message length byte

        Serial.print("Message type: ");
        Serial.print(messageType);
        Serial.print(", Message length: ");
        Serial.println(messageLength);

        if (messageType == messageType4) // Check for sleep trigger message
        {
            Serial.println("Received sleep trigger. Going back to deep sleep...");
            esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_MIN_FACTOR *uS_TO_S_FACTOR);
            esp_deep_sleep_start(); // Enter deep sleep
        }

        if (messageLength == 8 && Wire.available() >= 4)
        {
            float receivedValue = readFloatFromWire(); // Read float value
            switch (messageType)
            {
            case 0x01:
                receivedFloatTemp = receivedValue; // Read temperature value
                Serial.print("Temperature received: ");
                Serial.println(receivedFloatTemp);
                break;
            case 0x02:
                receivedFloatTds = receivedValue; // Read TDS value
                Serial.print("TDS received: ");
                Serial.println(receivedFloatTds);
                break;
            case 0x03:
                receivedFloatpH = receivedValue; // Read pH value
                Serial.print("pH received: ");
                Serial.println(receivedFloatpH);
                break;
            default:
                Serial.println("Unknown message type");
                break;
            }
        }
    }
}

// Function to read a float from I2C bus
float readFloatFromWire()
{
    uint8_t byteArray[sizeof(float)]; // Create byte array to store float bytes
    for (int i = 0; i < sizeof(float); i++)
    {
        byteArray[i] = Wire.read(); // Read bytes into array
    }
    float value;
    memcpy(&value, byteArray, sizeof(float)); // Copy bytes to float
    return value;                             // Return reconstructed float value
}

// Placeholder function for transmitting data to the cloud
void transmitDataToCloud()
{
    Serial.println("Transmitting data to cloud...");
    // Add your cloud transmission code here
    // For now, this is just a placeholder
    flag = true;
}