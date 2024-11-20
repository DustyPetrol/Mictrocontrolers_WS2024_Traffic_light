#include <Arduino.h>    // Include Arduino library for basic functions

// Define pins for button and lights
#define buttonPin 8                // Button to trigger pedestrian crossing
#define trafficRedPin 13           // Traffic red light pin
#define trafficYellowPin 12        // Traffic yellow light pin
#define trafficGreenPin 11         // Traffic green light pin
#define pedestrianRedPin 10        // Pedestrian red light pin
#define pedestrianGreenPin 9       // Pedestrian green light pin

// Define timing constants (in milliseconds)
const unsigned long redTime = 5000;        // Duration of red light
const unsigned long yellowTime = 2000;     // Duration of yellow light
const unsigned long greenTime = 5000;      // Duration of green light
const unsigned long redYellowTime = 1000;  // Duration of red + yellow light
const unsigned long pedestrianGreenTime = redTime;   // Duration of pedestrian green light

// Define states for traffic light control
#define GREEN_LIGHT 0
#define YELLOW_LIGHT 1
#define RED_LIGHT 2
#define RED_YELLOW_LIGHT 3
#define PEDESTRIAN_CROSSING 4
#define YELLOW_TO_RED 5

// Traffic light class to control the light states and transitions
class TrafficLight {
  private:
    int redPin, yellowPin, greenPin;          // Pins for red, yellow, green lights
    unsigned long previousMillis;             // Timestamp for timing control
    int state;                                // Current state of the traffic light
    bool pedestrianWaiting;                   // Flag for pedestrian request
    uint8_t ReceavedMessage;                  // Stores received serial message
    bool IsThereAMessage = false;             // Flag indicating if a message is to be sent
    
  public:
    // Constructor to initialize traffic light pins and initial state
    TrafficLight(int rPin, int yPin, int gPin) : 
      redPin(rPin), 
      yellowPin(yPin), 
      greenPin(gPin), 
      state(RED_LIGHT), 
      previousMillis(millis()), 
      pedestrianWaiting(false) {
      pinMode(redPin, OUTPUT);
      pinMode(yellowPin, OUTPUT);
      pinMode(greenPin, OUTPUT);
      redLight();   // Initialize with red light on
    }

    // Update function to manage state transitions based on timing and pedestrian request
    void update(unsigned long currentMillis) {
      // Check if a serial message is available
      if (Serial.available() > 0) {
        ReceavedMessage = Serial.read();
      } else {
        ReceavedMessage = 0;
      }

      // Handle pedestrian crossing request
      if (pedestrianWaiting && state != PEDESTRIAN_CROSSING && state != YELLOW_TO_RED) {
        if (state == RED_LIGHT) {
          state = PEDESTRIAN_CROSSING;        // Switch to pedestrian crossing state
          previousMillis = currentMillis;
          pedestrianWaiting = false;
        } else {
          state = YELLOW_TO_RED;              // Transition to yellow-to-red before pedestrian crossing
          previousMillis = currentMillis;
          pedestrianWaiting = false;
          yellowLight();
        }
      }
      

      // State machine for traffic light control
      switch (state) {
        case GREEN_LIGHT: 
          if (currentMillis - previousMillis >= greenTime) {
            state = YELLOW_LIGHT;
            previousMillis = currentMillis;
            yellowLight();
          }
          break;
        case YELLOW_LIGHT: 
          if (currentMillis - previousMillis >= yellowTime) {
            state = RED_LIGHT;
            previousMillis = currentMillis;
            redLight();
          }
          break;
        case RED_LIGHT:
          if (currentMillis - previousMillis >= redTime) {
            state = RED_YELLOW_LIGHT;
            previousMillis = currentMillis;
            redYellowLight();
          }
          break;
        case RED_YELLOW_LIGHT: 
          if (currentMillis - previousMillis >= redYellowTime) {
            state = GREEN_LIGHT;
            previousMillis = currentMillis;
            greenLight();
          }
          break;
        case YELLOW_TO_RED: 
          if (currentMillis - previousMillis >= yellowTime) {
            state = PEDESTRIAN_CROSSING;
            previousMillis = currentMillis;
            redLight();
          }
          break;
        case PEDESTRIAN_CROSSING:
          SendMessage();
          if (ReceavedMessage == 200) {
            ReceavedMessage = 0;
            state = RED_YELLOW_LIGHT;
            previousMillis = currentMillis;
            redYellowLight();
            pedestrianWaiting = false;
          }
          break;
      }
    }

    // Function to set message flag when a message needs to be sent
    void ThereIsAMessage() {
      IsThereAMessage = true;
    }

    // Send a message through Serial if the flag is set
    void SendMessage() {
      if (IsThereAMessage) {
        Serial.write(100);
        IsThereAMessage = false;
      }
    }

    // Functions to set the traffic light color by controlling respective pins
    void redLight() {
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, LOW);
      digitalWrite(greenPin, LOW);
    }

    void yellowLight() {
      digitalWrite(redPin, LOW);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, LOW);
    }

    void greenLight() {
      digitalWrite(redPin, LOW);
      digitalWrite(yellowPin, LOW);
      digitalWrite(greenPin, HIGH);
    }

    void redYellowLight() {
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, LOW);
    }

    // Trigger pedestrian crossing request
    void triggerPedestrianCrossing() {
      pedestrianWaiting = true;
    }

    // Check if the light is red
    bool isRed() {
      return state == RED_LIGHT;
    }

    // Check if pedestrian crossing state is active
    bool isPedestrianCrossing() {
      return state == PEDESTRIAN_CROSSING;
    }
};

// Traffic system class to handle the traffic light and button for pedestrian crossing
class TFSystem {
  private:
    TrafficLight traffic;

  public:
    TFSystem(int trafficRed, int trafficYellow, int trafficGreen) 
    : traffic(trafficRed, trafficYellow, trafficGreen) {
      pinMode(buttonPin, INPUT_PULLUP);  // Set button pin as input with pull-up resistor
    }

    void update(unsigned long currentMillis) {
      if (digitalRead(buttonPin) == LOW && !traffic.isPedestrianCrossing()) {
        traffic.ThereIsAMessage();          // Set message flag
        traffic.triggerPedestrianCrossing(); // Trigger pedestrian crossing
      }

      traffic.update(currentMillis);        // Update traffic light based on current time
    }
};

// Instantiate the traffic system with defined traffic light pins
TFSystem trafficSystem(trafficRedPin, trafficYellowPin, trafficGreenPin);

void setup() {
  Serial.begin(9600);    // Initialize serial communication
}

void loop() {
  unsigned long currentMillis = millis();    // Get current time
  trafficSystem.update(currentMillis);       // Update traffic system based on time
}
