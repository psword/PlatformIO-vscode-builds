// ph_sensor.h

#ifndef pHSensor_h
#define pHSensor_h

#include "sensor.h"
#include <DFRobot_ESP_PH.h> // DFRobot pH Library v2.0
#include <EEPROM.h>         // EEPROM library

/**
 * pHSensor class, derived from Sensor, to handle pH sensor operations.
 */
class pHSensor : public Sensor 
{
private:
    int SENSOR_INPUT_PIN;          // Define the input PIN to read
    const int pHSenseIterations;   // Number of pH measurements to take
    const float referenceTemp;     // Reference temperature for the pH sensor
    const float maxADCValue;       // ADC bits
    float VC;                      // The voltage on the pin powering the sensor (in 1000 units)
    float *analogBuffer;           // Dynamic array for buffer
    int analogBufferIndex;         // Index for circular buffer
    unsigned long lastReadTime;    // To store the last read time
    const unsigned long readDelay; // Delay between reads (milliseconds)

    DFRobot_ESP_PH ph; // pH sensor object
    float averageVoltage;          // Store the average voltage

public:
    /**
     * Constructor to initialize the pH sensor with given parameters.
     * @param voltageConstant The voltage constant for the sensor.
     * @param refTemp The reference temperature for the sensor.
     * @param maxADC The maximum ADC value for the sensor.
     * @param inputPin The input pin for the sensor.
     * @param iterations The number of pH measurements to take (default: 40).
     * @param readGap The delay between reads in milliseconds (default: 250).
     */
    pHSensor(float voltageConstant, float refTemp, float maxADC, int inputPin, int iterations = 40, unsigned long readGap = 250);

    /**
     * Destructor to clean up resources.
     */
    ~pHSensor();

    /**
     * Initializes the pH sensor object.
     */
    void beginSensors();

    /**
     * Returns the average voltage.
     * @return The average voltage.
     */
    float getAverageVoltage() const;

    /**
     * Calibrates the sensor with given voltage and temperature.
     * @param voltage The voltage for calibration.
     * @param temperature The temperature for calibration.
     */
    void calibrateSensors(float voltage, float temperature);

    /**
     * Returns the read delay value.
     * @return The read delay value.
     */
    unsigned long getReadDelay() const;

    /**
     * Reads analog value from pH sensor and stores in buffer.
     */
    void analogReadAction();

    /**
     * Computes the median reading from the buffer.
     * @return The median reading.
     */
    float computeMedian();

    /**
     * Adjusts pH based on temperature.
     * @param voltage The voltage to adjust.
     * @param temperature The temperature to adjust with.
     * @return The adjusted pH value.
     */
    float adjustpH(float voltage, float temperature);

    /**
     * Computes the pH value.
     * @param temperature The temperature to compute with.
     * @return The computed pH value.
     */
    float read(float temperature) override;

    /**
     * Initializes the sensor, if needed.
     */
    void init() override;

    /**
     * Shuts down the sensor, no operation.
     */
    void shutdown() override;

    /**
     * Stabilizes the sensor with a timer, possible for future operation.
     */
    void stabilize() override;
};

#endif // pHSensor_h