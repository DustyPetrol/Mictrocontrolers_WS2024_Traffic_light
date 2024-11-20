#include <Arduino.h>   


#define pedestrianRedPin 10    
#define pedestrianGreenPin 9  

// Define timing constants (in milliseconds)
const unsigned long redTime = 5000;         
const unsigned long yellowTime = 2000;      
const unsigned long greenTime = 5000;      
const unsigned long redYellowTime = 1000;   
const unsigned long pedestrianGreenTime = redTime;  

void setup() {
  Serial.begin(9600);    
}

// Class to handle pedestrian light functions
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
  
    while (Serial.available() > 0){
      ReceavedMessage = Serial.read();   
      }
  
    if (ReceavedMessage == 100 && !active) {
      while
      activate(currentMillis);            
      ReceavedMessage = 0;               
      greenLight();                      
    }

    
    if (active && currentMillis - previousMillis >= pedestrianGreenTime) {
      active = false;                    
      redLight();                        
      Serial.write(200);                 
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
    active = true;                        // Set active flag
    previousMillis = currentMillis;       // Store the current time for timing reference
  }

 
  bool isActive() {
    return active;
  }
};


PedestrianLight pedestrian(pedestrianRedPin, pedestrianGreenPin);

void loop() {
  unsigned long currentMillis = millis();   
  pedestrian.update(currentMillis);        
}
