// Constants for pin assignments
#define PinButton = 8;       
#define PinRedTraffic = 13;   
#define PinYellowTraffic = 12;
#define PinGreenTraffic = 11; 
#define PinGreenPed = 10;
#define PinRedPed = 9;
#define green 0
#define yellow 1
#define red 2
#define yellowred 3  

#define Timegreen 3000
#define Timered 3000
#define Timeyellow 3000
#define TimeyellowRed 3000  
const unsigned long pedestrianGreenTime = 5000; 


 


class TrafficLight {
  private:
    int redPin, yellowPin, greenPin;
    unsigned long previousMillis;
    int state; // To keep track of the current light state
    bool pedestrianWaiting; // Flag for pedestrian button press

  public:
    TrafficLight(int rPin, int yPin, int gPin) : redPin(rPin), yellowPin(yPin), greenPin(gPin), state(RED_LIGHT), previousMillis(0), pedestrianWaiting(false) {
      pinMode(redPin, OUTPUT);
      pinMode(yellowPin, OUTPUT);
      pinMode(greenPin, OUTPUT);
      redLight();  // Start with the red light on
    }

    void update(unsigned long currentMillis) {
      if (pedestrianWaiting) return;  // Don't change traffic lights if the pedestrian light is active

      switch (state) {
        case GREEN_LIGHT: // Green light
          if (currentMillis - previousMillis >= greenTime) {
            state = YELLOW_LIGHT;
            previousMillis = currentMillis;
            yellowLight();
          }
          break;
        case YELLOW_LIGHT: // Yellow light
          if (currentMillis - previousMillis >= yellowTime) {
            state = RED_LIGHT;
            previousMillis = currentMillis;
            redLight();
          }
          break;
        case RED_LIGHT: // Red light
          if (currentMillis - previousMillis >= redTime) {
            state = RED_YELLOW_LIGHT;  // Transition to Red + Yellow state
            previousMillis = currentMillis;
            redYellowLight();
          }
          break;
        case RED_YELLOW_LIGHT: // Red + Yellow light
          if (currentMillis - previousMillis >= redYellowTime) {
            state = GREEN_LIGHT;  // Transition to Green light
            previousMillis = currentMillis;
            greenLight();
          }
          break;
      }
    }

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

    void setPedestrianWaiting(bool waiting) {
      pedestrianWaiting = waiting;
    }

    bool isRed() {
      return state == RED_LIGHT; // Return true if the current light is red
    }
};

// Pedestrian light class with its own independent behavior
class PedestrianLight {
  private:
    int redPin, greenPin;
    bool active;
    unsigned long previousMillis;

  public:
    PedestrianLight(int rPin, int gPin) : redPin(rPin), greenPin(gPin), active(false), previousMillis(0) {
      pinMode(redPin, OUTPUT);
      pinMode(greenPin, OUTPUT);
      redLight();  // Start with the red light on
    }

    void update(unsigned long currentMillis) {
      if (active && currentMillis - previousMillis >= pedestrianGreenTime) {
        active = false;  // End pedestrian crossing
        redLight();
      }
    }

    void redLight() {
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
    }

    void greenLight() {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
    }

    void activate(unsigned long currentMillis) {
      active = true;
      previousMillis = currentMillis;
      greenLight();  // Start pedestrian crossing
    }

    bool isActive() {
      return active;
    }
};

// Instances of TrafficLight and PedestrianLight
TrafficLight traffic(trafficRedPin, trafficYellowPin, trafficGreenPin);
PedestrianLight pedestrian(pedestrianRedPin, pedestrianGreenPin);

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // Setup button with internal pull-up resistor
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // Check if the pedestrian button is pressed
  if (digitalRead(buttonPin) == LOW && !pedestrian.isActive() && traffic.isRed()) {
    pedestrian.activate(currentMillis);      // Activate pedestrian light
    traffic.setPedestrianWaiting(true);      // Notify traffic light of pedestrian crossing
  }

  // Update the traffic light behavior
  traffic.update(currentMillis);

  // Update the pedestrian light behavior
  pedestrian.update(currentMillis);

  // After pedestrian cycle, return traffic lights to normal operation
  if (!pedestrian.isActive()) {
    traffic.setPedestrianWaiting(false);  // Allow traffic lights to resume their cycle
  }
}