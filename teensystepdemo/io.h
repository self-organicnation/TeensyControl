#ifndef io_h
#define io_h

#include "Arduino.h"

void initIo();

// Simple IO 
#define LED 13
void toggleLed();

// Motors
#define NBMOTORS 6

const uint8_t STEP[NBMOTORS] = {5, 8, 11, 25, 28, 31};               
const uint8_t DIR[NBMOTORS] = {6, 9, 12, 26, 29, 32};                
const uint8_t EN[NBMOTORS] = {4, 7, 10, 24, 27, 30}; 
#define STEPMAX 6400   
void initSteppersPins();
void initSteppers();
int32_t getStep(uint8_t motorId); 
void setStep(uint8_t motorId, int32_t val);
void setGoal(uint8_t motorNumber, int32_t val);
void setMaxspeed(uint8_t motorNumber, int32_t val);
void setAccel(uint8_t motorNumber, int32_t val);
void moveMotors();
void moveMotorsAsync();
void rotateMotors();
void overrideSpeed(int percent);
void stopMotors();

// Encoders
const uint8_t ENCODERAS[NBMOTORS] = {21, 14, 18, 15, 36, 33};
const uint8_t ENCODERBS[NBMOTORS] = {22,  2, 19, 16, 37, 34};
const uint8_t ENCODERZS[NBMOTORS] = {23, 39, 20, 17, 38, 35};
#define ENCODERMAX 4000
void initEncoders();
int32_t getCount(uint8_t motorId);
void setCount(uint8_t motorId, int32_t val);
void displayAllCountSerial1();

#endif
