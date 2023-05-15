#include <Arduino.h>
#include <Wire.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include "imu_manager.h"

BMM150 bmm = BMM150();
bmm150_mag_data value_offset;




/*
*@brief sets the initial conf to the required library
*@param 
*@return 
*/
void compassManagerSetup() {
  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while (1)
      ;
  } else {
    Serial.println("Initialize done!");
  }
}


// ************************************************************
//                             CALIBRATION
// ************************************************************

/**
    @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
    @param timeout - seconds of calibration period.
*/
void compassManagerCalibrate(uint32_t timeout) {
  Serial.println("Start figure-8 calibration after 3 seconds.");
  delay(3000);
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout) {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if (value_x_min > bmm.raw_mag_data.raw_datax)
      value_x_min = bmm.raw_mag_data.raw_datax;
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
      value_x_max = bmm.raw_mag_data.raw_datax;

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
      value_y_min = bmm.raw_mag_data.raw_datay;
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
      value_y_max = bmm.raw_mag_data.raw_datay;

    /* Update 7-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
      value_z_min = bmm.raw_mag_data.raw_dataz;
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
      value_z_max = bmm.raw_mag_data.raw_dataz;

    Serial.print(".");
    delay(100);
  }

  value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;


  Serial.print("\n\rCalibrate done..");
  Serial.println(value_offset.x);
  Serial.println(value_offset.y);
  Serial.println(value_offset.z);
   Serial.println("");
}






// ************************************************************
//                            GET HEADING
// ************************************************************

/*
*@brief returns raw magnitic x field in micro Gauss
*@param void
*@return the magnetic field in x axes
*/
float compassManagerGetHeading() {
  bmm150_mag_data value;
  bmm.read_mag_data();

  value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
  value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
  value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;

  float xyHeading = atan2(value.x, value.y);
  float zxHeading = atan2(value.z, value.x);
  float heading = xyHeading;

  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  float headingDegrees = heading * 180 / M_PI;
  float xyHeadingDegrees = xyHeading * 180 / M_PI;
  float zxHeadingDegrees = zxHeading * 180 / M_PI;


  delay(100);

  return headingDegrees;
}


// ************************************************************
//                          FIELD VALUES
// ************************************************************

/*
*@brief returns raw magnitic x field in micro Gauss
*@param void
*@return the magnetic field in x axes
*/
float compassManagerXfield() {
  bmm150_mag_data value;
  bmm.read_mag_data();
  return value.x;
}

/*
*@brief returns raw magnitic y field in micro Gauss
*@param void
*@return the magnetic field in y axes
*/
float compassManagerYfield() {
      bmm150_mag_data value;
    bmm.read_mag_data();
  return value.y;
}

/*
*@brief returns raw magnitic z field in micro Gauss
*@param void
*@return the magnetic field in z axes
*/
float compassManagerZfield() {
      bmm150_mag_data value;
    bmm.read_mag_data();
  return value.z;
}


// ************************************************************
//                            TEST 
// ************************************************************

/*
*@brief returns the magnitic earth field heading in degrees
*@param void
*@return the magnitic earth field heading in degrees
*/
 float compassPrintHeading(){
  float headingDegrees =compassManagerGetHeading();
  Serial.print("Heading: ");
  Serial.println(headingDegrees);   
 }
