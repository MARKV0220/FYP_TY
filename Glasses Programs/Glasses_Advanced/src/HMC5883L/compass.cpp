#include <Arduino.h>
#include <Wire.h>
#include "compass.h"


// Definitions ----------------------------------------------------------------
#define compass_address 0x1E       // The I2C address of the Magnetometer
#define compass_rad2degree 57.3

#define compass_x_offset  0
#define compass_y_offset  100                 //150
#define compass_z_offset  -90                 //-20
#define compass_x_gainError 1
#define compass_y_gainError 1            //1.04
#define compass_z_gainError 1             //1.00
#define compass_gain_fact 1               //1.22


//***************************************************************************
//***************************************************************************
//****************************  FUNCTIONS    ********************************
//***************************************************************************
//***************************************************************************

// --------------------------------------------------------------------------
// read the data
// This function updates the Global X Y Z Variables

void compass_read_XYZdata(int *compass_x, int *compass_y, int *compass_z){

  Wire.beginTransmission(compass_address);
  Wire.write(0x02);
  Wire.write(0b10000001);
  // Writing the register value 0000 0000 for continous mode
  // Writing the register value 0000 0001 for single
  Wire.endTransmission();
  Wire.requestFrom(compass_address,6);

  if (6 <= Wire.available()){

    *compass_x = Wire.read()<<8 | Wire.read();
    *compass_z = Wire.read()<<8 | Wire.read();
    *compass_y = Wire.read()<<8 | Wire.read();

  }

}


// --------------------------------------------------------------------------
// Setting the gain
// This Function updates the gain_fact variable

void compass_init(){

  float x,y,z; //dummy variables for an initial reading(to eliminate fake readings)

  Wire.beginTransmission(compass_address);
  Wire.write(0x01);

  Wire.write(0b01000000); // bit configuration = g2 g1 g0 0 0 0 0 0, g2 g1 g0 = 0 0 1 for 1.3 guass and 0 1 0 for 1.9 Guass
  Wire.write(0b00000011);  // Putting the Magnetometer in idle
  // Writing the register value 0000 0000 for continous mode
  // Writing the register value 0000 0001 for single
  // Writing the register value 0000 0011 for Idel
  Wire.endTransmission();

  compass_headingYZ(&x,&y,&z);

}

void compass_scalled_reading(float *compass_x_scalled, float *compass_y_scalled, float *compass_z_scalled){

  static int compass_x,compass_y,compass_z;

  Wire.beginTransmission(compass_address);
  Wire.write(0x02);
  Wire.write(0b10000001);
  // Writing the register value 0000 0000 for continous mode
  // Writing the register value 0000 0001 for single
  Wire.endTransmission();
  Wire.requestFrom(compass_address,6);

  if (6 <= Wire.available()){

    compass_x = Wire.read()<<8 | Wire.read();
    compass_z = Wire.read()<<8 | Wire.read();
    compass_y = Wire.read()<<8 | Wire.read();

  }

  *compass_x_scalled=compass_x*compass_gain_fact*compass_x_gainError+compass_x_offset;
  *compass_y_scalled=(compass_y*compass_gain_fact*compass_y_gainError+compass_y_offset)*0.67;
  *compass_z_scalled=compass_z*compass_gain_fact*compass_z_gainError+compass_z_offset;


}

/*
void compass_headingXY(){
  compass_scalled_reading();

  if (compass_y_scalled>0){
    Bearing = 90-atan(compass_x_scalled/compass_y_scalled)*compass_rad2degree;
  }else if (compass_y_scalled<0){
    bearing = 270-atan(compass_x_scalled/compass_y_scalled)*compass_rad2degree;
  }else if (compass_y_scalled==0 & compass_x_scalled<0){
    bearing = 180;
  }else{
    bearing = 0;
  }
}
*/
//This function calculates the angle between magnetic vector projection onto YZ plane and Y axis
float compass_headingYZ(float *compass_x_scalled, float *compass_y_scalled, float *compass_z_scalled){

  float Bearing = 0;

  compass_scalled_reading(compass_x_scalled,compass_y_scalled,compass_z_scalled);

  if ((*compass_z_scalled)>0){
    Bearing = 90-atan((*compass_y_scalled) / (*compass_z_scalled))*compass_rad2degree;
  }else if ((*compass_z_scalled)<0){
    Bearing = 270-atan((*compass_y_scalled) / (*compass_z_scalled))*compass_rad2degree;
  }else if ((*compass_z_scalled==0) & (*compass_y_scalled<0)){
    Bearing = 180;
  }else{
    Bearing = 0;
  }

  return Bearing;
}
