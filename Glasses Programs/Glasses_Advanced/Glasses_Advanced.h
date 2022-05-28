#ifndef GLASSES_ADVANCED_H_INCLUDED
#define GLASSES_ADVANCED_H_INCLUDED

#include <TimerOne.h>
#include <PinChangeInterrupt.h>
#include "src/MPU6050/I2Cdev.h"
#include "src/MPU6050/MPU6050.h"
#include "src/HMC5883L/compass.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define VoltageTransOE 4   // pin 4 is connected to the enable input of voltage translator
#define Buzzer1 5
#define Buzzer2 6
#define Buzzer_Loudness 200  // a number betweeen 1-255
#define User_Button1  8
#define User_Button2  7
#define LED1  17
#define LED2  16
#define LED3  15
#define LED4  14
#define LED_DutyCycle   4// a number between 1~5 (20%~100%)

extern bool Button1_State;
extern bool Button1_Old_State;
extern bool Button2_State;
extern bool Button2_Old_State;

extern bool LED_State;

extern unsigned int  Compass_Bearing;
extern unsigned int  Target_Bearing; // default north
extern int  Bearing_Delta;
extern float Compass_x_scalled, Compass_y_scalled, Compass_z_scalled;
extern bool Target_Reset;            // target reset flag, for disabling PWM driving when switching target heading

//extern int16_t   SD_Buffer1[10*Sample_Number];        // 3-axis accelerometer, 3-axis gyro, 3-axis mag --- each for 10 samples
//extern int16_t   SD_Buffer2[10*Sample_Number];        // This is the abandoned C array plan
extern String SD_Buffer1;
extern bool SD_initDone;

extern unsigned int    Ctr_20ms;
extern unsigned int    Ctr_20ms_Old;                               

extern int16_t Accelx,Accely,Accelz,Gyrox,Gyroy,Gyroz;

extern  File MotionData;
extern  MPU6050 mpu;

void Standby_ISR_1Hz();
void LED_ISR_1kHz();
void Button1_State_Change();
void Button2_State_Change();
void Bearing_Delta_Cal();
void LED_PWM();
void LEDAll(bool LEDState);
void Buzzer_Alert();
void Set_Target(byte times);
void MPU6050_init();
void SD_init();
void TargetReset_Routine();
void LED_Scrolling();

#endif // GLASSES_ADVANCED_H_INCLUDED
