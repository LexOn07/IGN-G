#include <Arduino.h>

static int setPoint;
static int pin_PV;
static int hysteresis;
static bool PV_ON = false;
extern long unsigned engine_RPM;

static void setupRegulator(int pinRevers, int PV_RPM, int hyster){
    pinMode(pinRevers, OUTPUT);         //пин реле
    pinRevers = pin_PV; 
    setPoint = PV_RPM;                  //установка по оборотам
    hysteresis = hyster;                //ширина гистерезиса
}

static bool PV_position(){
  if(engine_RPM >= setPoint + hysteresis){
      PV_ON = true;
  }
  if(engine_RPM <= setPoint - hysteresis){
      PV_ON = false;
  }
  digitalWrite(pin_PV, PV_ON);
  return PV_ON;
}
static bool get_PV_position(){
  return PV_ON;
}