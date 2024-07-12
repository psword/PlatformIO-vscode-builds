#include <Ticker.h>

// Motor 1 pins
const int motor1Pin1 = 23;
const int motor1Pin2 = 33;

// Motor 2 pins
const int motor2Pin1 = 21;
const int motor2Pin2 = 25;

// Timer intervals in seconds
const float directionInterval = 5.0; // Change direction every 5 seconds
const float runInterval = 5.0; // Run motors for 5 seconds

Ticker directionTimer1;
Ticker directionTimer2;
Ticker runTimer1;
Ticker runTimer2;

// Variables to keep track of motor states
bool motor1Forward = true;
bool motor2Forward = true;

void setup() {
  // Initialize motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Attach the direction toggle functions to the timers
  directionTimer1.attach(directionInterval, toggleMotor1Direction);
  directionTimer2.attach(directionInterval, toggleMotor2Direction);

  // Initially start motors in one direction
  startMotor1();
  startMotor2();
}

void loop() {
  // The Ticker library handles the timing, so the loop can be empty
}

void startMotor1() {
  if (motor1Forward) {
    // Set motor 1 to move forward
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    // Set motor 1 to move backward
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  runTimer1.once(runInterval, stopMotor1); // Run the motor for 5 seconds
}

void startMotor2() {
  if (motor2Forward) {
    // Set motor 2 to move forward
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    // Set motor 2 to move backward
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  runTimer2.once(runInterval, stopMotor2); // Run the motor for 5 seconds
}

void stopMotor1() {
  // Stop motor 1
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
}

void stopMotor2() {
  // Stop motor 2
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void toggleMotor1Direction() {
  motor1Forward = !motor1Forward; // Toggle the direction
  startMotor1();
}

void toggleMotor2Direction() {
  motor2Forward = !motor2Forward; // Toggle the direction
  startMotor2();
}
