// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "imu_manager.h"
#include <Arduino.h>


#define MEASURE_PERIOD_MS 100

// ********************************************************
//                       STRUCTURES
// *******************************************************

// GYROSCOPE
struct GyrData {
  float pitch;
  float roll;
  float yaw;
};

struct GyrOffset {
  float pitch;
  float roll;
  float yaw;
};

// ACCELEROMETER
struct AccData {
  float x;
  float y;
  float z;
};

struct AccOffset {
  float x;
  float y;
  float z;
};


// ********************************************************
//                       GLOBAL VARIABLES
// *******************************************************

// GYROSCOPE
GyrData gyroscopeData;
GyrOffset gyroscopeOffset;

// ACELEROMETER
AccData accelerometerData;
AccOffset accelerometerOffset;


float debounceGyro = 5;
float debounceAccel = 0.1;

uint8_t calibrationSamples = 10;

Adafruit_MPU6050 mpu;


void ImuManagerTestPrintGyroAcc(){
  updateAccelerometerData();
  updateGyroscopeData();

  Serial.print(gyroscopeData.roll, 2);
  Serial.print(", ");
  Serial.print(gyroscopeData.pitch, 2);
  Serial.print(", ");
  Serial.print(gyroscopeData.yaw, 2);
  Serial.println("");

   /*
  Serial.print(accelerometerData.x, 2);
  Serial.print(", ");
  Serial.print(accelerometerData.y, 2);
  Serial.print(", ");
  Serial.print(accelerometerData.z, 2);
  Serial.println("");
  delay(10);
*/

}

// ********************************************************
//                        FUNCTION DEFINITIONS
// *******************************************************

void ImuManagerSetup(){

    // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}





//********************** CALIBRATION ***********************

void startCalibrationProcess() {
  Serial.println("Leave the IMU over a flat surface and press enter to start calibration...");
  while (!Serial.available()) {}
  calibrateAcelerometer();
  calibrateGyroscope();
  Serial.println("IMU calibrated...");
}


/*
* @brief this function is use to calibrate the acelerometer, it is necessary to leave the IMU over a flat surface.
* basically it takes the average of the acelerometer readings over a period of time, and use it for the offset.
* @param void
* @return void
*/
void calibrateAcelerometer() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("Calibrating Acceleromter ...");
  delay(1000);
  float tempX = 0, tempY = 0, tempZ = 0;
  int auxSamples = 0;

  while (1) {

    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      tempX += a.acceleration.x;
      tempY += a.acceleration.y;
      tempZ += a.acceleration.z;
      prev_ms = millis();
      auxSamples++;
    }

    if (auxSamples == calibrationSamples)
      break;
  }

  accelerometerOffset.x = tempX / (float)calibrationSamples;
  accelerometerOffset.y = tempY / (float)calibrationSamples;
  accelerometerOffset.z = tempZ / (float)calibrationSamples - 9.8;
}

/* 
 * @brief this function is use to calibrate the gyroscope, it is necessary to leave the IMU over a flat surface.
 * basically it takes the average of the gyroscope readings over a period of time, and use it for the offset.
 * @param void
 * @return void
*/
void calibrateGyroscope() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float gx = g.gyro.x, gy = g.gyro.y, gz = g.gyro.z;


  Serial.println("Calibrating gyroscope ...");
  delay(5000);
  float tempX = 0, tempY = 0, tempZ = 0;
  int auxSamples = 0;

  while (1) {

    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      tempX += atan2(ay, az) * 180.0 / PI;
      tempY += atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
      tempZ += atan2(gy, gx) * 180.0 / PI;
      prev_ms = millis();
      auxSamples++;
    }
    if (auxSamples == calibrationSamples)
      break;
  }
  gyroscopeOffset.pitch = tempX / (float)calibrationSamples;
  gyroscopeOffset.roll = tempY / (float)calibrationSamples;
  gyroscopeOffset.yaw = tempZ / (float)calibrationSamples;
}


//************************** GET IMU DATA *************************


/* 
*@brief this  updates acceletometer data
*@param void
*@return void
*/
void updateAccelerometerData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // X ACELERATION
  float tempAX = a.acceleration.x - accelerometerOffset.x;
  if (abs(tempAX - accelerometerData.x) > debounceAccel)
    accelerometerData.x = tempAX;

  // Y ACELERATION
  float tempAY = a.acceleration.y - accelerometerOffset.y;
  if (abs(tempAY - accelerometerData.y) > debounceAccel)
    accelerometerData.y = tempAY;

  // Z ACELERATION
  float tempAZ = a.acceleration.z - accelerometerOffset.z;
  if (abs(tempAZ - accelerometerData.z) > debounceAccel)
    accelerometerData.z = tempAZ;
}


/*
*@brief this function updates the gyroscope data
*@param 
*@return 
*/
void updateGyroscopeData() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float gx = g.gyro.x, gy = g.gyro.y, gz = g.gyro.z;

  // PITCH
  float tempPitch = atan2(ay, az) * 180.0 / PI - gyroscopeOffset.pitch;
  if (abs(tempPitch -   gyroscopeData.pitch) > debounceGyro) {
    gyroscopeData.pitch = tempPitch;
  }

  // ROLL
  float tempRoll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI - gyroscopeOffset.roll;
  if (abs(tempRoll - gyroscopeData.roll) > debounceGyro) {
    gyroscopeData.roll = tempRoll;
  }

  // YAW
  float tempYaw = atan2(gy, gx) * 180.0 / PI - gyroscopeOffset.yaw;
  if (abs(tempYaw - gyroscopeData.yaw) > debounceGyro)
    gyroscopeData.yaw = tempYaw;
}

//************************** TEST *************************

void ImuManagerPrintOffsets(){
  Serial.println("");
  Serial.print("****************** OFFSETS VALUE ************\n");
  Serial.print("ROLL, PITCH, YAW: ");
  Serial.print(gyroscopeOffset.roll, 2);
  Serial.print(", ");
  Serial.print(gyroscopeOffset.pitch, 2);
  Serial.print(", ");
  Serial.print(gyroscopeOffset.yaw, 2);
  Serial.println("");
  Serial.print("ACCX, ACCY, ACCZ: ");
  Serial.print(accelerometerOffset.x, 2);
  Serial.print(", ");
  Serial.print(accelerometerOffset.y, 2);
  Serial.print(", ");
  Serial.print(accelerometerOffset.z, 2);
  Serial.println("");
}


//************************** GETTERS *************************


float ImuManagerGetPitnch(){
  return gyroscopeData.pitch;
}

float ImuManagerGetRoll(){
  return gyroscopeData.roll;
}

float ImuManagerGetYaw(){
  return gyroscopeData.yaw;
}

float ImuManagerGetAccX(){
  return accelerometerData.x;
}

float ImuManagerGetAccY(){
  return accelerometerData.y;
}

float ImuManagerGetAccZ(){
  return accelerometerData.z;
}



    


