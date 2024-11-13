#include <Arduino.h>    // Include the Arduino library for basic functions and definitions

// Define pins for pedestrian lights
#define pedestrianRedPin 10    // Pedestrian red light pin
#define pedestrianGreenPin 9   // Pedestrian green light pin

// Define timing constants (in milliseconds)
const unsigned long redTime = 5000;         // Red light duration for pedestrian light
const unsigned long yellowTime = 2000;      // Yellow light duration (unused in this code)
const unsigned long greenTime = 5000;       // Green light duration for pedestrian light
const unsigned long redYellowTime = 1000;   // Red-yellow duration (unused in this code)
const unsigned long pedestrianGreenTime = redTime;   // Pedestrian green light duration

void setup() {
  Serial.begin(9600);    // Initialize serial communication at 9600 bps
}

// Class to handle pedestrian light functions
class PedestrianLight {
private:
  uint8_t redPin, greenPin;           // Pins for red and green lights
  bool active;                        // Flag indicating if the green light is active
  unsigned long previousMillis;       // Timestamp for timing control
  byte ReceavedMessage=0;               // Stores received serial message

public:
  // Constructor for PedestrianLight class, sets pin modes and initializes the light to red
  PedestrianLight(uint8_t rPin, uint8_t gPin)
    : redPin(rPin), greenPin(gPin), active(false), previousMillis(0) {
    pinMode(rPin, OUTPUT);            // Set red pin as output
    pinMode(gPin, OUTPUT);            // Set green pin as output
    redLight();                       // Initialize pedestrian light to red
  }

  // Method to update the pedestrian light state based on the current time
  void update(unsigned long currentMillis) {
    // Check if a serial message is available
    if (Serial.available() > 0)
      ReceavedMessage = Serial.read();    // Read the message from serial

    // Check for message with value 100 and if light is not already active
    if (ReceavedMessage == 100 && !active) {
      activate(currentMillis);            // Activate green light timing
      ReceavedMessage = 0;                // Reset the message
      greenLight();                       // Turn on green light
    }

    // If the light is active and green duration has passed, turn off green light
    if (active && currentMillis - previousMillis >= pedestrianGreenTime) {
      active = false;                     // Reset active flag
      redLight();                         // Turn on red light
      Serial.write(200);                  // Send response message indicating green light has ended
    }
  }

  // Turn on the red light, turn off the green light
  void redLight() {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
  }

  // Turn on the green light, turn off the red light
  void greenLight() {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
  }

  // Activate green light duration tracking
  void activate(unsigned long currentMillis) {
    active = true;                        // Set active flag
    previousMillis = currentMillis;       // Store the current time for timing reference
  }

  // Returns whether the pedestrian light is currently active
  bool isActive() {
    return active;
  }
};

// Instantiate a PedestrianLight object with specified red and green pins
PedestrianLight pedestrian(pedestrianRedPin, pedestrianGreenPin);

void loop() {
  unsigned long currentMillis = millis();   // Get the current time
  pedestrian.update(currentMillis);         // Update pedestrian light based on the time
}
