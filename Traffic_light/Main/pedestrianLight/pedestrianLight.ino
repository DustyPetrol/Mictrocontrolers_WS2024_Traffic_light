#include <Arduino.h>
#define pedestrianRedPin 10 
#define pedestrianGreenPin 9 
const unsigned long redTime = 5000;        
const unsigned long yellowTime = 2000;     
const unsigned long greenTime = 5000;      
const unsigned long redYellowTime = 1000;  
const unsigned long pedestrianGreenTime = redTime;

void setup() {
 Serial.begin(9600);
}

class PedestrianLight {
  private:
    uint8_t redPin, greenPin;
    bool active;
    unsigned long previousMillis;
    int ReceavedMessage;

  public:
    PedestrianLight(uint8_t rPin, uint8_t gPin) : redPin(rPin), greenPin(gPin), active(false), previousMillis(0) {
      pinMode(rPin, OUTPUT);
      pinMode(gPin, OUTPUT);
      redLight();  
    }

    void update(unsigned long currentMillis) {
      if (Serial.available() > 0) {
      ReceavedMessage=Serial.read();
      }
      else
      ReceavedMessage=0;
      if (ReceavedMessage==100) {
        activate(currentMillis);
        ReceavedMessage=0;
        greenLight();
      } else {
        redLight();
      }

      if (active && currentMillis - previousMillis >= pedestrianGreenTime) {
        active = false;
        redLight(); 
        Serial.print(200);
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
    }

    bool isActive() {
      return active;
    }
};
PedestrianLight pedestrian(pedestrianRedPin,pedestrianGreenPin);



void loop() {
  unsigned long currentMillis = millis();
  pedestrian.update(currentMillis);
}
