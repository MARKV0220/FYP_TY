#include "Glasses_Advanced.h"

void setup() {
  // put your setup code here, to run once:

  Serial.begin(38400);
  Wire.begin();

  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  
  pinMode(Buzzer1,OUTPUT);
  pinMode(Buzzer2,OUTPUT);
  
  pinMode (User_Button1 ,INPUT_PULLUP);
  pinMode (User_Button2, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPCINT(User_Button1),Button1_State_Change , RISING);
  attachPinChangeInterrupt(digitalPinToPCINT(User_Button2),Button2_State_Change , RISING);

  pinMode(VoltageTransOE,OUTPUT);
  digitalWrite(VoltageTransOE,HIGH);

  compass_init(); // initialisation of HMC5883L
  MPU6050_init(); // initialisation of MPU6050, including vaious settings

  Timer1.initialize( 1000000 ); // timer1 initialization process (1s), using microsecond as unit
  Timer1.attachInterrupt(Standby_ISR_1Hz);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Target_Reset == 1) TargetReset_Routine();                                    // force LEDs to blink three times in 1.2s after target reset

  if (Ctr_20ms != Ctr_20ms_Old){                                                   //make measurements and record data each 20ms
    
    Bearing_Delta_Cal();                                                          //taking magnetic measurement and calculate bearing delta
    if (SD_initDone){                                                             // do MPU measurements and SD card logging only when SD card is present
        mpu.getMotion6(&Accelx, &Accely, &Accelz, &Gyrox, &Gyroy, &Gyroz);            //take accelerometer and gyroscope measurement, and put all 9 data + 1 enter into a long string
        SD_Buffer1 += (String)Accelx + " " + (String)Accely + " " + (String)Accelz + " " + (String)Gyrox + " " + (String)Gyroy + " " + (String)Gyroz + " " + (String)Compass_x_scalled + " " + (String)Compass_y_scalled + " " + (String)Compass_z_scalled + " " + (String)Compass_Bearing + " " + "\r\n";
        MotionData.print(SD_Buffer1);                                                 //write this measurement to SD card
        SD_Buffer1.remove(0);                                                         //clear the entire buffer, waiting for next loop
    }
    
    //Buzzer_Alert();                                                               //trigger buzzer alert if necessary
    Ctr_20ms_Old = Ctr_20ms;
  }

}

//--------------------------------------------------------
//FUNCTIONS---------------------------------------------
//-------------------------------------------------------
//All functions except the ISRs are moved to Glasses_Advanced.cpp
//ISRs are kept here for general clarity
//All the other variable declartions, function prototypes, #define and #include are in Glasses_Advanced.h

// low power standby mode --- 1Hz
void Standby_ISR_1Hz(){

  interrupts();

  if (SD.begin(10)){                                                // unsuccessful initialsation timeout = 500ms，the system will wait for 500ms and perform LED scrolling
      if (SD_initDone == 0) SD_init();                                  //if SD is present, initialise it once
      SD_initDone = 1;                                                  // just initialise once
      LEDAll(LED_State);                                            // blink all LED together,indicating standby
      LED_State = ! LED_State;
  }
  else{
      LED_Scrolling();                                              //indicating that the SD card is not present
      SD_initDone = 0;
  }
  
  if (Button1_State != Button1_Old_State){     
      Timer1.detachInterrupt();
      
      Button1_Old_State = Button1_State;                          // refresh the global button1 old state
      LED_State = 0;
      Set_Target(8);                                              //set the first heading target
      
      LEDAll(LOW);
      for (byte i=0;i<3;i++){                                     // blink LEDs three times, showing target selected
        LEDAll(HIGH);
        delay(200);
        LEDAll(LOW);
        delay(200);
      }

      Timer1.attachInterrupt(LED_ISR_250Hz,4000);                  // T=4ms, f=250Hz
  }
}

//LED bit-banging PWM ------ ISR Running at 250Hz, PWM has a frequency of 50Hz and 20% of resolution 
//This 50Hz frequency is also used for sensors to take samples
void LED_ISR_250Hz(){
  
  interrupts();
  static byte Ctr_4ms = 0;                                  //keeping incrementing every 4ms, restored to 0 when reaching 5 (every 20ms)
  Ctr_4ms++;

  if (Target_Reset == 0){
      if (Ctr_4ms == LED_DutyCycle)  LEDAll(LOW);                                        // Ctr_4ms(0-Dutycycle)-> LED HIGH,  Ctr_4ms(Dutycycle-40)-> LED LOW, the resolution of PWM is 20%
      if (Ctr_4ms == 5){LED_PWM(); Ctr_4ms = 0; Ctr_20ms++; }                              //every 4ms*5=20ms  (50Hz)                                                                                                                                                                                                      
  }
  else{
      if (Ctr_4ms == 5) {Ctr_4ms = 0; Ctr_20ms++;}                                         // normal PWM will be disabled during switching target heading,but the sensors will keep taking samples               
  }


  if (Button1_State != Button1_Old_State){                //user wants to change a heading target, press button1
      Button1_Old_State = Button1_State;
      Target_Reset = 1;                                   // entering reset state, disabling normal PWM driving, force to blink three times
  }

  if (Button2_State != Button2_Old_State){                //user wants to end this journey, press button2
      Timer1.detachInterrupt();
      Button2_Old_State = Button2_State;
      if (SD_initDone) {MotionData.close();SD_initDone=0;}                                 //if SD card is present, finish writing data to the SD. clear init flag, the system will create a new file when user starts a new trip
      Timer1.attachInterrupt(Standby_ISR_1Hz,1000000);             //back to the standby mode in 1Hz
  }
  
}
