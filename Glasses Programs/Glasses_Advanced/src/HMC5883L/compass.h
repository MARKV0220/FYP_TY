#ifndef compass.h
  #define compass.h

  #include "compass.h"

void compass_init();
  //void compass_headingXY();
float compass_headingYZ(float *compass_x_scalled, float *compass_y_scalled, float *compass_z_scalled);
void compass_scalled_reading(float *compass_x_scalled, float *compass_y_scalled, float *compass_z_scalled);
void compass_read_XYZdata(int *compass_x, int *compass_y, int *compass_z);


#endif
