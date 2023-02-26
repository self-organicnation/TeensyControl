#ifndef processing_h
#define processing_h

#include "io.h"

void initProcessing();
void processingControl(); 
int16_t computeDeltaPosition(uint8_t n);
void recvWithStartEndMarkers();
void parseData();
void noJoeTransformation();

#endif