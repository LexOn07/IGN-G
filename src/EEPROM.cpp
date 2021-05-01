#ifndef EEPROM_cpp
#define EEPROM_cpp

#include <Arduino.h>
#include <EEPROM.h>

extern int rpm_max;
extern int rpm_over;
extern int temp_max;
extern int PVrpm_max;
extern int pwm_max;
extern int pwm_const;
extern int pwm_max_time;
extern int timing_corrector;
extern int off_timer;
extern int UART_timer_delay;
extern int TEMP_timer_delay;
extern int arr_timing_rpm[12];
extern unsigned long arr_timing_temp[12];
extern unsigned long arr_sens_voltage[12];

static void start_read_eeprom(int a, int b, int c, int d, int e, int f,  int g, int h, int i, int j, int k){   //Стартовое чтение из ЕЕПРОМ
    if(digitalRead(24) == true){    //Если замкнуты контакты сброса ЕЕПРОМ то взять данные из прошивки.
        EEPROM.put(0, a);
        EEPROM.put(2, b);
        EEPROM.put(4, c);
        EEPROM.put(6, d);
        EEPROM.put(8, e);
        EEPROM.put(10, f);
        EEPROM.put(12, g);
        EEPROM.put(14, h);
        EEPROM.put(16, i);
        EEPROM.put(18, j);
        EEPROM.put(20, k);
        static int counter = 0;
        static int iter = 0;
        for (int number : arr_timing_rpm){
            EEPROM.put(20 + counter, arr_timing_rpm[iter]);
            EEPROM.put(68 + counter, arr_timing_temp[iter]);
            EEPROM.put(116 + counter, arr_sens_voltage[iter]);
            counter = counter + 4;
            iter++;
        }
    }
    else{
        EEPROM.get(0, rpm_max);
        EEPROM.get(2, rpm_over);
        EEPROM.get(4, temp_max);
        EEPROM.get(6, PVrpm_max);
        EEPROM.get(8, pwm_max);
        EEPROM.get(10, pwm_const);
        EEPROM.get(12, pwm_max_time);
        EEPROM.get(14, timing_corrector);
        EEPROM.get(16, off_timer);
        EEPROM.get(18, UART_timer_delay);
        EEPROM.get(20, TEMP_timer_delay);
        static int counter = 0;
        static int iter = 0;
        for (int number : arr_timing_rpm){
            EEPROM.get(20 + counter, arr_timing_rpm[iter]);
            EEPROM.get(68 + counter, arr_timing_temp[iter]);
            EEPROM.get(116 + counter, arr_sens_voltage[iter]);
            counter = counter + 4;
            iter++;
        }
    }
}
static void write_eeprom(int data, int num){
    EEPROM.write(num, data);
}
static void write_arr_eeprom(unsigned long data[], String type){
    int counterSUM;
    if(type = "timing_rpm"){
        counterSUM = 7;
    }
    if(type = "timing_temp"){
        counterSUM = 20;
    }
    if(type = "sens_voltage"){
        counterSUM = 33;
    }
    for (int i = 0; i < 12; i++){
        EEPROM.put(33 + i, data[i]);
    }
}
static void write(int a){
    EEPROM.put(0, a);
}
static void write_arr_timing_rpm(){
    static int counter = 0;
    static int iter = 0;
    for (int number : arr_timing_rpm){
        EEPROM.put(20 + counter, arr_timing_rpm[iter]);
        counter = counter + 4;
        iter++;
    }
}
static void write_arr_timing_temp(){
    static int counter = 0;
    static int iter = 0;
    for (int number : arr_timing_temp){
        EEPROM.put(68 + counter, arr_timing_temp[iter]);
        counter = counter + 4;
        iter++;
    }
}
static void write_arr_sens_voltage(){
    static int counter = 0;
    static int iter = 0;
    for (int number : arr_sens_voltage){
        EEPROM.put(116 + counter, arr_sens_voltage[iter]);
        counter = counter + 4;
        iter++;
    }
}
static void write_sett(){
    EEPROM.put(0, rpm_max);
    EEPROM.put(2, rpm_over);
    EEPROM.put(4, temp_max);
    EEPROM.put(6, PVrpm_max);
    EEPROM.put(8, pwm_max);
    EEPROM.put(10, pwm_const);
    EEPROM.put(12, pwm_max_time);
    EEPROM.put(14, timing_corrector);
    EEPROM.put(16, off_timer);
    EEPROM.put(18, UART_timer_delay);
    EEPROM.put(20, TEMP_timer_delay);
}    
#endif


