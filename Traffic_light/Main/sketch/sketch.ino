#include <Arduino.h>
#define buttonPin 8        
#define trafficRedPin 13    
#define trafficYellowPin 12
#define trafficGreenPin 11 
#define pedestrianRedPin 10 
#define pedestrianGreenPin 9 


const unsigned long redTime = 5000;        
const unsigned long yellowTime = 2000;     
const unsigned long greenTime = 5000;      
const unsigned long redYellowTime = 1000;  
const unsigned long pedestrianGreenTime = redTime;

#define GREEN_LIGHT 0
#define YELLOW_LIGHT 1
#define RED_LIGHT 2
#define RED_YELLOW_LIGHT 3 
#define PEDESTRIAN_CROSSING 4
#define YELLOW_TO_RED 5  

class TrafficLight {
  private:
    int redPin, yellowPin, greenPin;
    unsigned long previousMillis;
    int state; 
    bool pedestrianWaiting; 
    int ReceavedMessage;

  public:
    TrafficLight(int rPin, int yPin, int gPin) : 
    redPin(rPin), 
    yellowPin(yPin), 
    greenPin(gPin), 
    state(RED_LIGHT), 
    previousMillis(0), 
    pedestrianWaiting(false) {
      pinMode(redPin, OUTPUT);
      pinMode(yellowPin, OUTPUT);
      pinMode(greenPin, OUTPUT);
      redLight();  
    }

    void update(unsigned long currentMillis) {
     

      if (Serial.available() > 0) {
      ReceavedMessage=Serial.read();
      }
      else
      ReceavedMessage=0;
      if (pedestrianWaiting && state != PEDESTRIAN_CROSSING && state != YELLOW_TO_RED) {
        if (state == RED_LIGHT) {
          state = PEDESTRIAN_CROSSING;
          previousMillis = currentMillis;
          pedestrianWaiting = false;
        } else {
          state = YELLOW_TO_RED;
          previousMillis = currentMillis;
          pedestrianWaiting = false;
          yellowLight();
        }
      }

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
        case RED_LIGHT: // Red light
          if (currentMillis - previousMillis >= redTime) {
            state = RED_YELLOW_LIGHT; 
            previousMillis = currentMillis;
            redYellowLight();
          }
          break;
        case RED_YELLOW_LIGHT: // Red + Yellow light
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
         // if (currentMillis - previousMillis >= pedestrianGreenTime) {old thing}
          if (ReceavedMessage==200) {
            ReceavedMessage=0;
            state = RED_YELLOW_LIGHT;  
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
      return state == RED_LIGHT; 
    }

    bool isPedestrianCrossing() {
      return state == PEDESTRIAN_CROSSING;  
    }
};




class TFSystem {
  private:
    TrafficLight traffic;

  public:
    TFSystem(int trafficRed, 
    int trafficYellow, 
    int trafficGreen) 
    : traffic(trafficRed, trafficYellow, trafficGreen) {
      pinMode(buttonPin, INPUT_PULLUP);  
    }

    void update(unsigned long currentMillis) {
      if (digitalRead(buttonPin) == LOW  && !traffic.isPedestrianCrossing()) {
        Serial.print(100);
        traffic.triggerPedestrianCrossing();        
      }

      
      traffic.update(currentMillis);

    }
};


TFSystem trafficSystem(trafficRedPin, trafficYellowPin, trafficGreenPin);

void setup() {
 
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();
  trafficSystem.update(currentMillis);    
}