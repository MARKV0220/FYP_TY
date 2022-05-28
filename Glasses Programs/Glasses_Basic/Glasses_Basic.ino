#include <TimerOne.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include "compass.h"

#define Buzzer1 5
#define Buzzer2 6
#define Buzzer_Loudness 200  // a number betweeen 1-255
#define User_Button1  8
#define User_Button2  7
#define LED1  17
#define LED2  16
#define LED3  15
#define LED4  14
#define LED_DutyCycle 4  // a number between 1~5
#define Sample_Number   10

bool Button1_State = 0;
bool Button1_Old_State = 0;
bool Button2_State = 0;
bool Button2_Old_State = 0;

unsigned int  Compass_Bearing = 0;
unsigned int  Target_Bearing = 0; // default north
int  Bearing_Delta = 0;

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

  compass_init(); // initialisation of HMC5883L

  Timer1.initialize( 500000 ); // timer1 initialization process (500ms), using microsecond as unit
  Timer1.attachInterrupt(ISR_1Hz);

}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("main loop running");

}

// low power standby mode
void ISR_1Hz(){

  static bool LED_State = 0;

  interrupts();

  LEDAll(LED_State);                                            // blink all LED together,indicating standby
  LED_State = ! LED_State;

  
  if (Button1_State != Button1_Old_State){    
      Timer1.detachInterrupt();
      Button1_Old_State = Button1_State;                          // refresh the global button1 old state

      Set_Target();                                               //set the first heading target

      Timer1.attachInterrupt(ISR_1kHz,4000); // T = 4ms, the previous 2khz is too high for the controller to follow, it will stuck in the middle forever
  }
  
}

void ISR_1kHz(){
  
  static byte Ctr1 = 0;

  interrupts();
  Ctr1++;

  Serial.println("1khz");

  if (Ctr1 == LED_DutyCycle)  LEDAll(LOW);                // Ctr1(0-Dutycycle)-> LED HIGH,  Ctr1(Dutycycle-5)-> LED LOW, the resolution of PWM is 2.5%


  if (Ctr1 == 5){                                        //take a measurement every 4ms*5=20ms
      Timer1.stop();                                        // stop the timer1 interrupt to take measurements and record data

      Bearing_Delta_Cal();
      Serial.print ("Bearing Delta = ");
      Serial.print (Bearing_Delta);
      Serial.println(" Degree");
    
      LED_PWM();
      Buzzer_Alert();
    
      Ctr1 = 0;
      Timer1.restart();                                    // restart timer1
  }

  if (Button1_State != Button1_Old_State){                //user wants to change a heading target, press button1
      Timer1.stop(); 
      Button1_Old_State = Button1_State;
      Set_Target();
      Timer1.restart();
  }

  
  if (Button2_State != Button2_Old_State){                //user wants to end this journey, press button2
      Button2_Old_State = Button2_State;
      Timer1.detachInterrupt();
      Timer1.attachInterrupt(ISR_1Hz,500000);             //back to the standby mode in 1Hz
  }
  
}

void Button1_State_Change(){            //button1 interrupt callback function
  Button1_State = ! Button1_State;
}

void Button2_State_Change(){            //button2 interrupt callback function
  Button2_State = ! Button2_State;
}

void Bearing_Delta_Cal(){
    Compass_Bearing = (unsigned int)(compass_headingYZ()); 
    Bearing_Delta = Compass_Bearing - Target_Bearing;

    if (Bearing_Delta > 180) Bearing_Delta = Compass_Bearing - 360 - Target_Bearing;
    if (Bearing_Delta < -180) Bearing_Delta = 360 - Target_Bearing + Compass_Bearing;
}

void LED_PWM(){
  static byte LEDMode = 0;

  if (Bearing_Delta<-60)          LEDMode = 3;
  else if (Bearing_Delta<-20)     LEDMode = 4;
  else if (Bearing_Delta>60)      LEDMode = 2;
  else if (Bearing_Delta>20)      LEDMode = 1;
  else                            LEDMode = 0;

  switch(LEDMode){
  case 0: LEDAll(LOW); break;
  case 1: digitalWrite(LED1,HIGH); break;
  case 2: digitalWrite(LED1,HIGH); digitalWrite(LED2,HIGH); break;
  case 3: digitalWrite(LED3,HIGH); digitalWrite(LED4,HIGH); break;
  case 4: digitalWrite(LED4,HIGH); 
 }
}

void LEDAll(bool LEDState){
  digitalWrite(LED1,LEDState);
  digitalWrite(LED2,LEDState);
  digitalWrite(LED3,LEDState);
  digitalWrite(LED4,LEDState);
}

void Buzzer_Alert(){
    
  if (abs(Bearing_Delta)>90){                                 
      if (Bearing_Delta<-90) analogWrite(Buzzer2,Buzzer_Loudness);    // active the right buzzr at 980Hz(driven by Timer0 default frequency),in the case of deviation to left
      else                   analogWrite(Buzzer1,Buzzer_Loudness);    // active the left buzzr at 980Hz(driven by Timer0 default frequency),in the case of deviation to right
  }
  else{
    digitalWrite(Buzzer1,LOW);                               //no significant deviation (>90 degree), shut down both buzzers
    digitalWrite(Buzzer2,LOW);
  }
  
}

void Set_Target(){
  
      Target_Bearing = 0;
      for (byte i=0;i<8;i++){
        Target_Bearing += (unsigned int)(compass_headingYZ());
      }
      Target_Bearing /= 8;                                        // take measurements for 3 times and use the average
      Serial.print ("Target Heading angle = ");
      Serial.print (Target_Bearing);
      Serial.println(" Degree");

      LEDAll(LOW);
      
      for (byte i=0;i<3;i++){                                     // blink LEDs three times, showing target selected
        LEDAll(HIGH);
        delay(200);
        LEDAll(LOW);
        delay(200);
      }
}
