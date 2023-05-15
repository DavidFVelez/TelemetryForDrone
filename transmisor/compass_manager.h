
#ifndef COMPASS_MANAGER_H
#define COMPASS_MANAGER_H

// ********************************************************
//                       FUNCTION PROTOTYPE
// *******************************************************

void compassManagerSetup();
void compassManagerCalibrate(uint32_t timeout);
float compassManagerGetHeading();

float compassManagerXfield();
float compassManagerYfield();
float compassManagerZfield();

float compassPrintHeading();

#endif