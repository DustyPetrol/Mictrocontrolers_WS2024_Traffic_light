#include <Arduino.h>


class TrafficLight{
private:
unsigned long TimeGreen = 3000;
unsigned long TimeYellow = 3000;
unsigned long TimeRed = 3000;
unsigned long TimeYellowRed = 3000;
uint8_t PinRed = 11;
uint8_t PinYellow = 12; 
uint8_t PinGreen = 13;
uint8_t NewState=0;
uint8_t CurrentState=0;
unsigned long CurrentStateTime=0;

void NextState()
{
  if (this->CurrentState < 3)
  this->NewState=this->NewState+1;
  else
  this->NewState=0;
}
void TurnOnGreen()
{
  digitalWrite(this->PinGreen, HIGH); 
  digitalWrite(this->PinRed, LOW);
  digitalWrite(this->PinYellow, LOW);
}
void TurnOnRed()
{
  digitalWrite(PinGreen, LOW); 
  digitalWrite(PinRed, HIGH);
  digitalWrite(PinYellow, LOW);
}
void TurnOnYellow()
{
  digitalWrite(PinGreen, LOW); 
  digitalWrite(PinRed, LOW);
  digitalWrite(PinYellow, HIGH);
}
void TurnOnYellowRed()
{
  digitalWrite(PinGreen, LOW); 
  digitalWrite(PinRed, HIGH);
  digitalWrite(PinYellow, HIGH);
}
unsigned long GetTime()
{
  switch(CurrentState)
  {
    case 0:
    return TimeGreen;
    case 1:
    return TimeYellow;
    case 2:
    return TimeRed;
    case 3: 
    return TimeYellowRed;
  }
}
void ChangeColour()
{
  if (NewState != CurrentState)
  {  this->CurrentState=NewState;
    switch (CurrentState){
    case 0:
    TurnOnGreen();
    break;
    case 1:
    TurnOnYellow();
    break;
    case 2:
    TurnOnRed();
    break;
    case 3:
    TurnOnYellowRed();
    break;
    }
  }
}
checkTime()
{
 if(millis()-this->CurrentStateTime>=GetTime())
  {
  NextState();
  this->CurrentStateTime=millis();
 }
}

public:
void GoTrafficLight(){ 
checkTime();
ChangeColour();
}

uint8_t GetNewState()
{return this->NewState;}
uint8_t GetCurrentState()
{return this->CurrentState;}

void SetTimeGreen(uint8_t Time) {
       this->TimeGreen = Time;
    }
    void SetTimeYellow(uint8_t Time) {
       this->TimeYellow = Time;
    }
    void SetTimeRed(uint8_t Time) {
       this->TimeRed = Time;
    }
    void SetYellowRed(uint8_t Time) {
       this->TimeYellowRed = Time;
    }

    uint8_t GetTimeGreen() {
       return this->TimeGreen;
    }
    uint8_t GetTimeYellow() {
     return  this->TimeYellow;
    }
    uint8_t GetTimeRed() {
     return  this->TimeRed;
    }
    uint8_t GetYellowRed() {
     return  this->TimeYellowRed;
    }
  uint8_t GetPinGreen() {
       return this->PinGreen;
    }
    uint8_t GetPinYellow() {
     return  this->PinYellow;
    }
    uint8_t GetPinRed() {
     return  this->PinRed;
    }
    void SetPinGreen(uint8_t Pin) {
       this->PinGreen = Pin;
    }
    void SetPinYellow(uint8_t Pin) {
       this->PinYellow = Pin;
    }
    void SetPinRed(uint8_t Pin) {
       this->PinRed = Pin;
    }
};

TrafficLight Traffic;
void setup() {
 Serial.begin(9600);
 pinMode(Traffic.GetPinGreen(), OUTPUT);
pinMode(Traffic.GetPinYellow(), OUTPUT);
pinMode(Traffic.GetPinRed(), OUTPUT);
}

void loop() {
Serial.print(Traffic.GetNewState());
Serial.println(Traffic.GetCurrentState());
Traffic.GoTrafficLight();
}
