
#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

// ********************************************************
//                       FUNCTION PROTOTYPE
// *******************************************************



// CALIBRATION
void startCalibrationProcess();
static void calibrateAcelerometer();
static void calibrateGyroscope();

// UPDATE
void updateAccelerometerData();
void updateGyroscopeData();

// SETUP
void ImuManagerSetup();

// GETTERS
float ImuManagerGetPitnch();
float ImuManagerGetRoll();
float ImuManagerGetYaw();
float ImuManagerGetAccX();
float ImuManagerGetAccY();
float ImuManagerGetAccZ();

// TEST 
void ImuManagerTestPrintGyroAcc();
void ImuManagerPrintOffsets();


#endif