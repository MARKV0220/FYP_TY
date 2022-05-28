#include "Glasses_Advanced.h"

bool Button1_State = 0;
bool Button1_Old_State = 0;
bool Button2_State = 0;
bool Button2_Old_State = 0;

bool LED_State = 0;

unsigned int  Compass_Bearing = 0;
unsigned int  Target_Bearing = 0; // default north
float Compass_x_scalled, Compass_y_scalled, Compass_z_scalled;
int  Bearing_Delta = 0;
bool Target_Reset = 0;

String SD_Buffer1;
bool SD_initDone = 0;

unsigned int    Ctr_20ms=0;
unsigned int    Ctr_20ms_Old=0;                           

int16_t Accelx,Accely,Accelz,Gyrox,Gyroy,Gyroz;

MPU6050 mpu; // mpu object to use
File MotionData; //SD reading and writing file object to use

//--------------------------
//Functions of user buttons
//--------------------------
void Button1_State_Change(){            //button1 interrupt callback function
  Button1_State = ! Button1_State;
}

void Button2_State_Change(){            //button2 interrupt callback function
  Button2_State = ! Button2_State;
}

//------------------------------
//Function about Buzzer playing -- 980Hz from timer0 default frequency
//-------------------------------
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

//--------------------------------
//Functions of LEDs
//-------------------------------
void LED_PWM(){
  static byte LEDMode = 0;

  if (Bearing_Delta<-60)          LEDMode = 3;            //-20
  else if (Bearing_Delta<-5)     LEDMode = 4;             
  else if (Bearing_Delta>60)      LEDMode = 2;
  else if (Bearing_Delta>5)      LEDMode = 1;             //20
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

void LED_Scrolling(){
  static byte LEDPos = 14;     // pin of LED1
  LEDAll(LOW);
  digitalWrite(LEDPos,HIGH);
  LEDPos++;                   //scrolling to left
  if (LEDPos==18) LEDPos=14;
}


// calculate the different between current bearing and target bearing. range:(-180~180)
void Bearing_Delta_Cal(){
    Compass_Bearing = (unsigned int)(compass_headingYZ(&Compass_x_scalled, &Compass_y_scalled, &Compass_z_scalled)); 
    Bearing_Delta = Compass_Bearing - Target_Bearing;

    if (Bearing_Delta > 180) Bearing_Delta = Compass_Bearing - 360 - Target_Bearing;
    if (Bearing_Delta < -180) Bearing_Delta = 360 - Target_Bearing + Compass_Bearing;
}


//--------------------------------
//Functions about setting and resetting target heading
//--------------------------------
void Set_Target(byte times){
  
      Target_Bearing = 0;
      for (byte i=0;i<times;i++){
        Target_Bearing += (unsigned int)(compass_headingYZ(&Compass_x_scalled, &Compass_y_scalled, &Compass_z_scalled));
      }
      Target_Bearing /= times;                                        // take measurements for N times and use the average
//      Serial.print ("Target Heading angle = ");
//      Serial.print (Target_Bearing);
//      Serial.println(" Degree");

      LEDAll(LOW);
}

void TargetReset_Routine(){
    static byte TargetResetCtr = 0;                                  // counter for driving LEDs during resetting
    
    if (TargetResetCtr==0)            Set_Target(3);             //first enter, reset the target
    if (Ctr_20ms != Ctr_20ms_Old){
        TargetResetCtr++;
        if (TargetResetCtr%10==0)   {LED_State = !LED_State; LEDAll(LED_State);}       //change all LED state every 10*20ms=200ms
    }
    if (TargetResetCtr == 60)         {Target_Reset = 0; TargetResetCtr=0;}            // blink three times, return to normal PWM driving
}

// accele and gyro initialisation
void MPU6050_init(){
      mpu.initialize();
    //mpu.setFullScaleGyroRange(3);
}

// micro SD card initialisation
void SD_init(){

  byte FileSerial=1;
  String FileName = "G_Data1.txt";

  while (SD.exists(FileName)){                              //check if there are some existing glasses data, if so, increment the serial number and create a new one
    FileSerial++;
    FileName = "G_Data" + (String)FileSerial + ".txt";
  }

  MotionData = SD.open(FileName,FILE_WRITE);
  MotionData.println("The following are the data for Trip "+(String)FileSerial);
  MotionData.println("Data Form: AcceleX AcceleY AcceleZ GyroX GyroY GyroZ MagX Magy MagZ");

  Serial.println(FileName);
  Serial.println("SD card init ok");
    
}
