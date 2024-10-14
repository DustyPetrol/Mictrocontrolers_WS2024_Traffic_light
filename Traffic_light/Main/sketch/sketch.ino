#include <Arduino.h>
#include <header.h>
#define green 0
#define yellow 1
#define red 2
#define yellowred 3
#define Timegreen 3000
#define Timered 3000
#define Timeyellow 3000
#define TimeyellowRed 3000
#define PinRedTraffic 13
#define PinYellowTraffic 12
#define PinGreenTraffic 11
#define PinGreenPed 10
#define PinRedPed 9



class TrafficLight {
protected:
  unsigned long TimeGreen = 3000;
  unsigned long TimeYellow = 3000;
  unsigned long TimeRed = 3000;
  unsigned long TimeYellowRed = 3000;
  uint8_t PinRed = 13;
  uint8_t PinYellow = 12;
  uint8_t PinGreen = 11;
  uint8_t NewState = 0;
  uint8_t CurrentState = 3;
  unsigned long CurrentStateTime = 0;
  bool PedestrianLightIsThere;

  void NextState() {
    if (this->CurrentState < 3)
      this->NewState = this->NewState + 1;
    else
      this->NewState = 0;
  }
  void TurnOnGreen() {
    digitalWrite(this->PinGreen, HIGH);
    digitalWrite(this->PinRed, LOW);
    digitalWrite(this->PinYellow, LOW);
  }
  void TurnOnRed() {
    digitalWrite(PinGreen, LOW);
    digitalWrite(PinRed, HIGH);
    digitalWrite(PinYellow, LOW);
  }
  void TurnOnYellow() {
    digitalWrite(PinGreen, LOW);
    digitalWrite(PinRed, LOW);
    digitalWrite(PinYellow, HIGH);
  }
  void TurnOnYellowRed() {
    digitalWrite(PinGreen, LOW);
    digitalWrite(PinRed, HIGH);
    digitalWrite(PinYellow, HIGH);
  }
  unsigned long GetTime() {
    switch (CurrentState) {
      case green:
        return TimeGreen;
      case yellow:
        return TimeYellow;
      case red:
        return TimeRed;
      case yellowred:
        return TimeYellowRed;
    }
  }
  void ChangeColour() {
    if (NewState != CurrentState) {
      this->CurrentState = NewState;
      switch (CurrentState) {
        case green:
          TurnOnGreen();
          break;
        case yellow:
          TurnOnYellow();
          break;
        case red:
          TurnOnRed();
          break;
        case yellowred:
          TurnOnYellowRed();
          break;
      }
    }
  }
  checkTime() {
    if (millis() - this->CurrentStateTime >= GetTime()) {
      NextState();
      this->CurrentStateTime = millis();
    }
  }

public:
  TrafficLight(
    unsigned long timegreen,
    unsigned long timered,
    unsigned long timeyellow,
    unsigned long timeyellowred,
    uint8_t pinred,
    uint8_t pingreen,
    uint8_t pingyellow,
    uint8_t startstate) {
    this->TimeGreen = timegreen;
    this->TimeYellow = timered;
    this->TimeRed = timeyellow;
    this->TimeYellowRed = timeyellowred;
    this->PinRed = pinred;
    this->PinYellow = pinyellow;
    this->PinGreen = pingreen;
    this->NewState = startstate;
    this->PedestrianLightIsThere = 0;
  }


  void GoTrafficLight() {
    checkTime();
    ChangeColour();
  }

  uint8_t GetNewState() {
    return this->NewState;
  }
  uint8_t GetCurrentState() {
    return this->CurrentState;
  }

  class PedestrianLight : TrafficLight {
  protected:

  }

  void
  SetTimeGreen(uint8_t Time) {
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
    return this->TimeYellow;
  }
  uint8_t GetTimeRed() {
    return this->TimeRed;
  }
  uint8_t GetYellowRed() {
    return this->TimeYellowRed;
  }
  uint8_t GetPinGreen() {
    return this->PinGreen;
  }
  uint8_t GetPinYellow() {
    return this->PinYellow;
  }
  uint8_t GetPinRed() {
    return this->PinRed;
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
  pinMode(Traffic.GetPinGreen(), OUTPUT);
  pinMode(Traffic.GetPinYellow(), OUTPUT);
  pinMode(Traffic.GetPinRed(), OUTPUT);
}

void loop() {

  Traffic.GoTrafficLight();
}
