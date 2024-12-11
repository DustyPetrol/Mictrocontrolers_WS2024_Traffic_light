#include <Arduino.h>    

#define buttonPin 8                
#define trafficRedPin 13           
#define trafficYellowPin 12        
#define trafficGreenPin 11        


const unsigned long redTime = 5000;        
const unsigned long yellowTime = 2000;    
const unsigned long greenTime = 5000;     
const unsigned long redYellowTime = 1000;  
const unsigned long pedestrianGreenTime = redTime;   
// Define states for traffic light control
#define GREEN_LIGHT 0
#define YELLOW_LIGHT 1
#define RED_LIGHT 2
#define RED_YELLOW_LIGHT 3
#define PEDESTRIAN_CROSSING 4
#define YELLOW_TO_RED 5
#define MessageToSend 100
#define MessageToRecieve 200


class TrafficLight {
  private:
    int redPin, yellowPin, greenPin;         
    unsigned long previousMillis;             
    int state;                               
    bool pedestrianWaiting;                  
    uint8_t ReceavedMessage=0;                  
    bool IsThereAMessage = false;             
    
  public:
   
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
      redLight();   
    }

    
    void update(unsigned long currentMillis) {
      
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
          if (ReceavedMessage == MessageToRecieve) {
            ReceavedMessage = 0;
            state = RED_YELLOW_LIGHT;
            previousMillis = currentMillis;
            redYellowLight();
            pedestrianWaiting = false;
          }
          break;
      }
    }
//workin dis
   
    void ThereIsAMessage() {
      IsThereAMessage = true;
    }

   void recieveMessage(uint8_t message){
    ReceavedMessage=message;
   }



   
    void SendMessage() {
      if (IsThereAMessage) {
        Serial.write(MessageToSend);
        IsThereAMessage = false;
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

    

   
    bool isPedestrianCrossing() {
      return state == PEDESTRIAN_CROSSING;
    }
};


class TFSystem {
  private:
    TrafficLight traffic;
    uint8_t MessageToPass;

  public:
    TFSystem(int trafficRed, int trafficYellow, int trafficGreen) 
    : traffic(trafficRed, trafficYellow, trafficGreen) {
      pinMode(buttonPin, INPUT_PULLUP); 
    }

    void update(unsigned long currentMillis) {
      if (digitalRead(buttonPin) == LOW && !traffic.isPedestrianCrossing()) {
        traffic.ThereIsAMessage();          
        traffic.triggerPedestrianCrossing(); 
      }
       while (Serial.available() > 0) {
        MessageToPass=Serial.read();
        traffic.recieveMessage(MessageToPass);
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
