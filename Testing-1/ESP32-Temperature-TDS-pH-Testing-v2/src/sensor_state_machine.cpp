// sensor_state_machine.cpp

#include "sensor_state_machine.h"
#include "sensor_config.h"
#include "transmit_functions.h"
#include <Arduino.h>

const uint8_t messageType1 = 0x01;   // Message type identifier
const uint8_t messageType2 = 0x02;   // Message type identifier
const uint8_t messageType3 = 0x03;   // Message type identifier
float adjustedTemp = 0;              // Variable to store temperature

template <typename SensorType>
SensorStateMachine<SensorType>::SensorStateMachine(SensorType& sensor, int powerPin)
    : sensor_(sensor), state_(SENSOR_OFF), stateStartTime_(0), powerPin_(powerPin) {}

template <typename SensorType>
void SensorStateMachine<SensorType>::start() {
    state_ = SENSOR_INIT;
}

template <typename SensorType>
void SensorStateMachine<SensorType>::operate() {
    switch (state_) {
        case SENSOR_OFF:
            // Do nothing until instructed to turn on the sensor
            break;
        case SENSOR_INIT:
            init();
            break;
        case SENSOR_STABILIZE:
            stabilize();
            break;
        case SENSOR_READ:
            read();
            break;
        case SENSOR_SHUTDOWN:
            shutdown();
            break;
    }
}

template <typename SensorType>
void SensorStateMachine<SensorType>::init() {
    stateStartTime_ = millis();
    digitalWrite(powerPin_, HIGH); // Power on the sensor
    state_ = SENSOR_STABILIZE;
}

template <typename SensorType>
void SensorStateMachine<SensorType>::stabilize() {
    if (millis() - stateStartTime_ >= 2000) { // Wait for 2 seconds
        state_ = SENSOR_READ;
    }
}

template <typename SensorType>
void SensorStateMachine<SensorType>::read() {
    float sensorValue = sensor_.read(adjustedTemp);
    Serial.println(sensorValue); // Uncomment for debugging
    sendFloatValue_ = sensorValue;
    sendMessageFlag_ = true;
    if (millis() - stateStartTime_ >= READING_DURATION) { // Waiting period
        state_ = SENSOR_SHUTDOWN;
    }
}

// TemperatureSensor specialization
template <>
void SensorStateMachine<TemperatureSensor>::shutdown() {
    digitalWrite(powerPin_, LOW); // Power off the sensor
    if (sendMessageFlag_) {
        transmitSlave(messageType1, sendFloatValue_); // Transmit using messageType1
        sendMessageFlag_ = false;
    }
    state_ = SENSOR_OFF;
}

// TdsSensor specialization
template <>
void SensorStateMachine<TdsSensor>::shutdown() {
    digitalWrite(powerPin_, LOW); // Power off the sensor
    if (sendMessageFlag_) {
        transmitSlave(messageType2, sendFloatValue_); // Transmit using messageType2
        sendMessageFlag_ = false;
    }
    state_ = SENSOR_OFF;
}

// pHSensor specialization
template <>
void SensorStateMachine<pHSensor>::shutdown() {
    digitalWrite(powerPin_, LOW); // Power off the sensor
    if (sendMessageFlag_) {
        transmitSlave(messageType3, sendFloatValue_); // Transmit using messageType3
        sendMessageFlag_ = false;
    }
    state_ = SENSOR_OFF;
}

template <typename SensorType>
bool SensorStateMachine<SensorType>::isOff() const {
    return state_ == SENSOR_OFF;
}

// Instantiate the templates for each sensor type
template class SensorStateMachine<TemperatureSensor>;
template class SensorStateMachine<TdsSensor>;
template class SensorStateMachine<pHSensor>;
