#include <Arduino.h>   
#include <Wire.h>

#define pedestrianRedPin 10    
#define pedestrianGreenPin 9  
#define MessageToSendPedestrian 200
#define MessageToRecievePedestrian 100
#define MessageToSendD3 203
#define MessageToRecieveD3 103
#define D1D2Adress 1
#define PedestrianAdress 2
#define D3Adress 3


// Define timing constants (in milliseconds)
const unsigned long redTime = 5000;         
const unsigned long yellowTime = 2000;      
const unsigned long greenTime = 5000;      
const unsigned long redYellowTime = 1000;   
const unsigned long pedestrianGreenTime = redTime;  

void setup() {
  Wire.begin(D1D2Adress);    
  Wire.onReceive(receiveEvent);   
}


class PedestrianLight {
private:
  uint8_t redPin, greenPin;          
  bool active;                        
  unsigned long previousMillis;      
  byte ReceavedMessage=0;               

public:
  
  PedestrianLight(uint8_t rPin, uint8_t gPin)
    : redPin(rPin), greenPin(gPin), active(false), previousMillis(0) {
    pinMode(rPin, OUTPUT);           
    pinMode(gPin, OUTPUT);            
    redLight();                       
  }

  
  void update(unsigned long currentMillis) {
  
    
  
    if (ReceavedMessage == MessageToRecievePedestrian && !active) {
      activate(currentMillis);            
      ReceavedMessage = 0;               
      greenLight();                      
    }

    
    if (active && currentMillis - previousMillis >= pedestrianGreenTime) {
      SendMessagePedestrian();
      active = false;                    
      redLight();                        
      Serial.write(MessageToSendPedestrian);                 
    }
  }
   void SendMessagePedestrian() {
      
         Wire.beginTransmission(D1D2Adress); // transmit to device #4
         Wire.write(MessageToSendPedestrian);        
         Wire.endTransmission();    
      
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
  }

 
  bool isActive() {
    return active;
  }
     void WeGotMessage(int8_t MessageWeGot){
    ReceavedMessage=MessageWeGot;
    }
};


PedestrianLight pedestrian(pedestrianRedPin, pedestrianGreenPin);

void loop() {
  unsigned long currentMillis = millis();   
  pedestrian.update(currentMillis);        
}

void receiveEvent()
{
  int8_t Message = Wire.read();    
  pedestrian.WeGotMessage(Message);
  Message=0;
}
