#define buttonPin  8        
#define trafficRedPin  13    
#define trafficYellowPin  12
#define trafficGreenPin  11 
#define pedestrianRedPin  10 
#define pedestrianGreenPin  9 


const unsigned long redTime = 5000;        
const unsigned long yellowTime = 2000;     
const unsigned long greenTime = 5000;      
const unsigned long redYellowTime = 1000;  
const unsigned long pedestrianGreenTime = redTime;

#define GREEN_LIGHT 0
#define YELLOW_LIGHT 1
#define RED_LIGHT 2
#define RED_YELLOW_LIGHT 3  // New state for Red + Yellow combination
#define PEDESTRIAN_CROSSING 4

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
      if (pedestrianWaiting && state != PEDESTRIAN_CROSSING) {
        state = PEDESTRIAN_CROSSING;
        pedestrianWaiting = false;
        previousMillis = currentMillis;
        redLight();  // Traffic light turns red for pedestrian crossing
      }

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
        case PEDESTRIAN_CROSSING:
          if (currentMillis - previousMillis >= pedestrianGreenTime) {
            state = RED_YELLOW_LIGHT;  // Resume to Red + Yellow
            previousMillis = currentMillis;
            redYellowLight();
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

    void triggerPedestrianCrossing() {
      pedestrianWaiting = true;
    }

    bool isRed() {
      return state == RED_LIGHT; // Return true if the current light is red
    }

    bool isPedestrianCrossing() {
      return state == PEDESTRIAN_CROSSING;  // Check if in pedestrian crossing state
    }
};

class PedestrianLight {
  private:
    int redPin, greenPin;
    bool active;
    unsigned long previousMillis;

  public:
    PedestrianLight(int rPin, int gPin) : redPin(rPin), greenPin(gPin), active(false), previousMillis(0) {
      pinMode(rPin, OUTPUT);
      pinMode(gPin, OUTPUT);
      redLight();  // Start with the red light on
    }

    void update(unsigned long currentMillis) {
      if (active && currentMillis - previousMillis >= pedestrianGreenTime) {
        active = false;
        redLight();  // Pedestrian crossing ends, red light for pedestrians
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
      greenLight();  // Pedestrian green light on
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
  if (digitalRead(buttonPin) == LOW && !pedestrian.isActive()) {
    traffic.triggerPedestrianCrossing();    // Interrupt traffic light for pedestrian crossing
    pedestrian.activate(currentMillis);     // Activate pedestrian light
  }

  // Update the traffic light behavior
  traffic.update(currentMillis);

  // Update the pedestrian light behavior
  pedestrian.update(currentMillis);
}