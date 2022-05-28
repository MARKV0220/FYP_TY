#ifndef compass.h
  #define compass.h
  
  #include "compass.h"
  
  extern float bearing;
  extern float compass_x_scalled;
  extern float compass_y_scalled;
  extern float compass_z_scalled;
    
  void compass_read_XYZdata();
  void compass_init();
  void compass_scalled_reading();
  //void compass_headingXY();
  float compass_headingYZ();
  
  
#endif
