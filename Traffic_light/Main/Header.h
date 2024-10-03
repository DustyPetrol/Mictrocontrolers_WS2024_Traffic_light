#include <Arduino.h>
class guessedNumber {
private:
uint8_t TimeGreen = 15000;
uint8_t TimeYellow = 3000;
uint8_t TimeRed = 15000;
uint8_t TimeYellowRed = 3000;
uint8_t PinRed = 13;
uint8_t PinYellow = 12; 
uint8_t PinGreen = 11;
uint8_t NewState;
uint8_t CurrentState;
uint8_t CurrentStateTime;

void 
public:
void GoTrafficLight(){

}
void SetTimeGreen(uint8_t Time) {
       TimeGreen = Time;
    }
    void SetTimeYellow(uint8_t Time) {
       TimeYellow = Time;
    }
    void SetTimeRed(uint8_t Time) {
       TimeRed = Time;
    }
    void SetYellowRed(uint8_t Time) {
       YellowRed = Time;
    }

    uint8_t GetTimeGreen() {
       return TimeGreen;
    }
    uint8_t GetTimeYellow() {
     return  TimeYellow;
    }
    uint8_t GetTimeRed() {
     return  TimeRed;
    }
    uint8_t GetYellowRed() {
     return  YellowRed;
    }
}