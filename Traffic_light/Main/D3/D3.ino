#include <Arduino.h>    
#include <Wire.h>

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
#define MessageToSendPedestrian 100
#define MessageToRecievePedestrian 200
#define MessageToSendD3 203
#define MessageToRecieveD3 103
#define D1D2Adress 1
#define PedestrianAdress 2
#define D3Adress 3



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
          if  (ReceavedMessage == MessageToRecieveD3) {
            state = RED_YELLOW_LIGHT;
            previousMillis = currentMillis;
            redYellowLight();
          }
           SendMessageD3(); // might be problematic, if is - put this in the end of yellow light and figure out the flag
          break;
        case RED_YELLOW_LIGHT: 
          if (currentMillis - previousMillis >= redYellowTime) {
            state = GREEN_LIGHT;
            previousMillis = currentMillis;
            greenLight();
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




    void SendMessageD3() {
      if (IsThereAMessage) {
         Wire.beginTransmission(D1D2Adress); // transmit to device #4
         Wire.write(MessageToSendD3);        
         Wire.endTransmission();    
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
    void update(unsigned long currentMillis){
    if (MessageToPass!=0) {
        traffic.recieveMessage(MessageToPass);
        traffic.ThereIsAMessage(); 
        MessageToPass=0;
      }  


      traffic.update(currentMillis);        
    }
    void WeGotMessage(int8_t MessageWeGot){
    MessageToPass=MessageWeGot;
    }
};


TFSystem trafficSystem(trafficRedPin, trafficYellowPin, trafficGreenPin);

void setup() {
  Wire.begin(D3Adress);    
  Wire.onReceive(receiveEvent);
}

void loop() {
  unsigned long currentMillis = millis();   
  trafficSystem.update(currentMillis);       
}

void receiveEvent()
{
  int8_t Message = Wire.read();    
  trafficSystem.WeGotMessage(Message);
  Message=0;
}
