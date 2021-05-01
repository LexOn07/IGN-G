//-------------------------------------------///
#include <Arduino.h>
#include <EEPROM.h>
#include "GyverFilters.h"
#include "Tacho.h"
#include "GyverUART.cpp"
#include "GyverTimer.cpp"
#include "GyverTimers.cpp"
//--------------------------------------------//
#define pin_EEPROM_reset 13  //пин сброса EEPROM в стандартное состояние
#define pin_sensor_temp 0    //пин датчика температуры двигателя
#define pin_revers_PV 32     //пин реверса мощностного клапана
#define pin_pwm_PV 7         //пин шим мощностного клапана
#define filter_const 50      //Знчение фильтра подбираеться в ручную
#define hyster_const 200     //Гистерезис для переключений мощностного клапана
#define PW_time_const 500    //Время между переключениями мощностного клапана (подбираеться в ручную)
#define rpm_over_time 200    //Время отключения двигателя при превышении оборотов
//--------------------------------------------//
int rpm_max = 7500; //Диапазон оборотов двигателя 0
int rpm_over = 8000;//обороты отсечки 1
int temp_max = 120;//диапазон датчика температуры двигателя 2
int PVrpm_max = 4200;//обороты переключения клапана 3
int pwm_max = 200;//Напряжение для открытия клапана мгновенное 4
int pwm_const = 80;//Напряжение для открытия клапана постоянное 5
int pwm_max_time = 800;//Время мгновенного удержания клапана в милисекундах 6
int arr_timing_rpm[12] = {15, 35, 76, 112, 124, 135, 156, 178, 197, 222, 235, 250};  //массив значений зависимости УОЗ от оборотов двигателя 7-19
unsigned long arr_timing_temp[12] = {15, 25, 40, 50, 60, 70, 80, 90, 100, 110, 120, 140};  //массив значений зависимости УОЗ от температуры 20-32
unsigned long arr_sens_voltage[12] = {1, 30, 140, 265, 310, 450, 590, 620, 722, 845, 955, 1023};  //массив значений кривой зависимости напряжения от температуры двигателя 33-45
int timing_corrector = 0;  //Общая коррекция УОЗ 46
int off_timer = 2000;  //Время подачи на коммутатор импульса 47
int UART_timer_delay = 500; //Время опроса UART 48
int TEMP_timer_delay = 300; //Время опроса дачика температуры 49

////////////////////////////////////////////////
unsigned long arr_rpm[12] = {}; //Массив для хранения диапазона оборотов (генерируеться при старте программы)
unsigned long arr_temp[12] = {}; //Массив для хранения диапазона температур (генерируеться при старте программы)
//--------------------------------------------//

GTimer UART_timer(MS, UART_timer_delay);
GTimer ignition_timer(US);
GTimer ignition_off_timer(US);
GTimer temp_timer(MS, TEMP_timer_delay);
GTimer PW_timer(MS, PW_time_const);
GTimer pwm_max_PW_timer(MS, pwm_max_time);
GTimer over(MS, rpm_over_time);
Tacho tachometr;
RingAverage<int, filter_const> fil;


#include "EEPROM.cpp"
#include "powerValve.cpp"

bool last_PW_position = 1;
bool ignition_on = true;         //разрешение подачи искры
unsigned long engine_RPM = 0;    //обороты двигателя (на лету)
unsigned long engine_temp = 0;   //температура двигателя (на лету)
unsigned long table_RPM = 0;     //задержка по таблице оборотов (на лету)
unsigned long table_temp = 0;    //задержка по таблице температуры (на лету)
bool connectUART = false;        // 
String uartRead = "Base";        //Прочитаные данные из УАРТ 

void ignition(){
  if(ignition_on){
    ignition_timer.setTimeout(10 * ((arr_timing_rpm[table_RPM] * engine_RPM / arr_rpm[table_RPM]) + arr_timing_temp[table_temp] + timing_corrector));
  }
}
void rpm_generation(){
  for (int i=0; i < 12; i++){
    if (i == 0){
      arr_rpm[i] = 625;
    } else {
      arr_rpm[i] = rpm_max / 12 * i + (rpm_max / 12);
    }
  }
}
void temp_generation(){
  for (int i=0; i < 12; i++){
    if (i == 0){
      arr_temp[i] = 0;
    } else {
      arr_temp[i] = temp_max / 12 * i;
    }
  }
}
void define_table_RPM(){
  for (int i=0; i < 12; i++){
    if(engine_RPM < arr_rpm[11]){
      if (arr_rpm[i] <= engine_RPM && arr_rpm[i + 1] >= engine_RPM) table_RPM = i;
    } else {
      table_RPM = i;
    }
  }
}
void define_table_temp(){
  for (int i=0; i < 12; i++){
    if(engine_temp < arr_temp[11]){
      if (arr_temp[i] <= engine_temp && arr_temp[i + 1] >= engine_temp) table_temp = i;
    } else {
      table_temp = i;
    }
  }
} 
unsigned long tmp;
unsigned long tmp2;
unsigned long tmp3;
void voltage_to_temp(int volt){
  for (int i=0; i < 12; i++){
    if(arr_sens_voltage[i] <= volt){
      engine_temp = arr_temp[i] * volt / arr_sens_voltage[i];
      tmp = arr_temp[i];
      tmp2 = volt;
      tmp3 = arr_sens_voltage[i];
    }
  }
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found> index ? data.substring (strIndex [0], strIndex [1]): "";
}
void ISR_func() {
  tachometr.tick();
  ignition();
}
void setup() {
  TCCR4A = TCCR4A & 0xe0 | 1;
  TCCR4B = TCCR4B & 0xe0 | 0x09;
  pinMode(pin_pwm_PV, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(pin_revers_PV, OUTPUT);         //пин реле
  uart.begin(115200);
  attachInterrupt(1, ISR_func, RISING); //Перывания для датчика коленвала
  start_read_eeprom(rpm_max, rpm_over, temp_max, PVrpm_max, pwm_max, pwm_const, pwm_max_time, timing_corrector, off_timer, UART_timer_delay, TEMP_timer_delay); //Читаем данные из ЕЕПРОМ если не верные то перезаписывем из прошивки
  rpm_generation();  //Генерируем сетку оборотов от которой отталкиваемся во время работы
  temp_generation(); //Генерируем сетку температуры от которой отталкиваемся во время работы
  setupRegulator(pin_revers_PV, PVrpm_max, hyster_const);
  pwm_max_PW_timer.stop();
  over.stop();
  //Timer2.setFrequency(8000); 
  //Timer2.outputEnable(CHANNEL_A, TOGGLE_PIN);
  //Timer2.outputState(CHANNEL_A, HIGH);
};

void loop() {
  if(uart.available()){
    String temp = uart.readString();
    if(temp == "Base" || temp == "Timing" || temp == "Temp" || temp == "Linear" || temp == "Sett"){
      uartRead = temp;
    }
    if (uartRead == "Timing_RPM_Wait"){
      for (int i = 0; i < 12; i++){
        arr_timing_rpm[i] = getValue(temp,' ', i).toInt();
      }
      write_arr_timing_rpm();
      uartRead = "Base";
    }
    if (uartRead == "Timing_TEMP_Wait"){
      for (int i = 0; i < 12; i++){
        arr_timing_temp[i] = getValue(temp,' ', i).toInt();
      }
      write_arr_timing_temp();
      uartRead = "Base";
    }
    if (uartRead == "Linear_TEMP_Wait"){
      for (int i = 0; i < 12; i++){
        arr_sens_voltage[i] = getValue(temp,' ', i).toInt();
      }
      write_arr_sens_voltage();
      uartRead = "Base";
    }
    if (uartRead == "SETT_Wait"){
      rpm_max = getValue(temp,' ', 0).toInt();
      temp_max = getValue(temp,' ', 1).toInt();
      TEMP_timer_delay = getValue(temp,' ', 2).toInt();
      rpm_over = getValue(temp,' ', 3).toInt();
      PVrpm_max = getValue(temp,' ', 4).toInt();
      pwm_max_time = getValue(temp,' ', 5).toInt();
      pwm_max = getValue(temp,' ', 6).toInt();
      pwm_const = getValue(temp,' ', 7).toInt();
      off_timer = getValue(temp,' ', 8).toInt();
      timing_corrector = getValue(temp,' ', 9).toInt();
      setupRegulator(pin_revers_PV, PVrpm_max, hyster_const);
      write_sett();
    }
  }
  engine_RPM = tachometr.getRPM();
  define_table_RPM();
  if(temp_timer.isReady()) {
    voltage_to_temp(analogRead(0));
    define_table_temp();
  }
  if(PW_timer.isReady()){
    if(PV_position() != last_PW_position){  
      digitalWrite(pin_revers_PV, !last_PW_position);
      pwm_max_PW_timer.start();
      analogWrite(pin_pwm_PV, pwm_max);
      last_PW_position = !last_PW_position;
    };
  }
  if(pwm_max_PW_timer.isReady()){
    analogWrite(pin_pwm_PV, pwm_const);
  }
  if(over.isReady()){
    ignition_on = true;
  }
  if(UART_timer.isReady()) {
    if (uartRead == "Base"){
      uart.print("US:" + String((arr_timing_rpm[table_RPM] * engine_RPM / arr_rpm[table_RPM]) + arr_timing_temp[table_temp] + timing_corrector) + " ");
      uart.print("RPM:" + String(engine_RPM) + " ");
      uart.print("TEMP:" + String(engine_temp) + " ");
      uart.print("PW:" + String(get_PV_position()) + " ");
      //uart.print(String(asa()));
    }  
  }
  if (uartRead == "Timing"){
      uart.print("RPM*");
      for (int i=0; i < 12; i++){
        uart.print(String(arr_rpm[i]) + " ");
      }
      uart.print("TIM*");
      for (int i=0; i < 12; i++){
        uart.print(String(arr_timing_rpm[i]) + " ");
      }
      uartRead = "Timing_RPM_Wait";
  }  
  if (uartRead == "Temp"){
      uart.print("TEMP*");
      for (int i=0; i < 12; i++){
        uart.print(String(arr_temp[i]) + " ");
      }
      uart.print("TIM*");
      for (int i=0; i < 12; i++){
        uart.print(String(arr_timing_temp[i]) + " ");
      }
      uartRead = "Timing_TEMP_Wait";
  }
  if (uartRead == "Linear"){
      uart.print("TEMP*");
      for (int i=0; i < 12; i++){
        uart.print(String(arr_temp[i]) + " ");
      }
      uart.print("LIN*");
      for (int i=0; i < 12; i++){
        uart.print(String(arr_sens_voltage[i]) + " ");
      }
      uartRead = "Linear_TEMP_Wait";
  }
  if (uartRead == "Sett"){
      uart.print("SETT*");
      uart.print(String(rpm_max) + " ");
      uart.print(String(temp_max) + " ");
      uart.print(String(TEMP_timer_delay) + " ");
      uart.print(String(rpm_over) + " ");
      uart.print(String(PVrpm_max) + " ");
      uart.print(String(pwm_max_time) + " ");
      uart.print(String(pwm_max) + " ");
      uart.print(String(pwm_const) + " ");
      uart.print(String(off_timer) + " ");
      uart.print(String(timing_corrector) + " ");
      uartRead = "SETT_Wait";
  } 
  if (ignition_timer.isReady() ){
    digitalWrite(22, true);
    ignition_off_timer.setTimeout(off_timer);

  }
  if (ignition_off_timer.isReady()){
    digitalWrite(22, false); 
  }
  if(engine_RPM > rpm_max){
    ignition_on = false;
    over.start();
  }
}
