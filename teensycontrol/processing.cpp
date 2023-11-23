#include "processing.h"


//   Receive with start- and end-markers combined with parsing
const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = { 0 };  //or 5 doesn't change anything

#define NBDATA 10

int timeElasped;

//int missedStepInterval;

int  timeElapsedBeforeAdaptPositionFromProcessing;



int integerFromPC[NBDATA] = { 0 };
int PC[NBDATA] = { 0 };
int PCTer[NBDATA] = { 0 };

int orderTrigger = 0;
int orderCohesion = 0;
int orderCohesionB = 0;
int startStop = 0;

float floatFromPC = 0.0;  // not used for the moment

boolean newData = false;
//==================================================================== CHECK NUMBER OF ROUND AND GOOD MOVEMENT

#define TX_SIZE 512
uint8_t tx_buffer[TX_SIZE];

//==================================================================    Jonathan computingDelta between actual and previous position.

int processingPosition[NBDATA] = { 0 };
int positionX[NBDATA] = { 0 };

int16_t computeDeltaPosition(uint8_t n) {
  static uint16_t oldPositionAbsolue[NBDATA] = { 0 };
  uint16_t positionAbsolue = processingPosition[n];
  int16_t resultat;
  if (positionAbsolue < oldPositionAbsolue[n])
    positionAbsolue += NBSTEPSPERTURN;
  if (positionAbsolue - oldPositionAbsolue[n] < NBSTEPSPERTURN / 2)
    resultat = positionAbsolue - oldPositionAbsolue[n];
  else
    resultat = positionAbsolue - oldPositionAbsolue[n] - NBSTEPSPERTURN;
  oldPositionAbsolue[n] = positionAbsolue;
  return resultat;
}
void doReboot() {  // reset Teensy
  SCB_AIRCR = 0x05FA0004;
}

void initProcessing() {
  for (uint8_t i = 0; i < NBDATA; i++) {
    processingPosition[i] = PC[i];
  }

  int tourTest = 6400 * 0;

  timeElapsedBeforeAdaptPositionFromProcessing=8000;

  for (uint8_t i = 0; i < NBMOTORS; i++) {
    PC[i] = tourTest * 1;  // premier devant moi
  }

  PCTer[2] = -3;  //noJoe method of Processng data conversion
  
  PCTer[4] = 1;  // driver actived (LOW
  PCTer[1] = 0;  // driver disable (HIGH
}


void processingControl() {

  if (PCTer[1] > 0)
  {
    digitalWrite(EN[0], LOW);  
  }

  if (PCTer[1] < 1)
  {
    digitalWrite(EN[0], HIGH);  
  }

  if (PCTer[4] > 0)
  {
    digitalWrite(EN[NBMOTORS-1], LOW); //
    digitalWrite(EN[NBMOTORS-2], LOW);
    digitalWrite(EN[NBMOTORS-3], LOW);
    digitalWrite(EN[NBMOTORS-4], LOW); 
    digitalWrite(EN[NBMOTORS-5], LOW);  
  } 
  
  if (PCTer[4] < 1)
  {
    digitalWrite(EN[NBMOTORS-1], HIGH);  // 
    digitalWrite(EN[NBMOTORS-2], HIGH);
    digitalWrite(EN[NBMOTORS-3], HIGH);
    digitalWrite(EN[NBMOTORS-4], HIGH); 
    digitalWrite(EN[NBMOTORS-5], HIGH);  
  }

   timeElapsedBeforeAdaptPositionFromProcessing = PCTer[5]; 

 //  timeElapsedBeforeAdaptPosition=  timeElapsedBeforeAdaptPositionFromProcessing;

   
/*
  if ( PCTer[5] != missedStepInterval ) {
       moveMissedStep.interval(PCTer[5]);
       missedStepInterval = PCTer[5]; 
      }  
 */     

  //------------  Bellow method to avoiding motor go back to zero when i restart Processing. It doesn't work

    // load 0 to all motor. Erase position datas
  if (PCTer[3] >= 2) {
     for (uint8_t i = 0; i < NBDATA; i++) {
      PC[i]=0;
      processingPosition[i] = PC[i];
      positionX[i]=processingPosition[i];
        
    }
    doReboot();
  }
 
/*
  if (PCTer[3] >= 2) {
     for (uint8_t i = 0; i < NBDATA; i++) {
      PC[i]=0;
      processingPosition[i] = PC[i];
     
      
    }
    doReboot();
  }

   //  Maybe it is better to find something else in Processing
 */ 
  recvWithStartEndMarkers();  // recevoir chaine de character et disperse en unité
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    newData = false;
  }

  for (uint8_t i = 0; i < NBMOTORS - 0; i++) {
    setAccel(i, 1600 * PCTer[0]);
  }

  if (PCTer[2] < -1) {  // noJoTransformation   Lire les données telles qu'elles arrivent depuis Processing
    noJoeTransformation();
  } else if (PCTer[2] > -1) {  // transforme data avec la methode de Jo

    for (uint8_t i = 0; i < NBDATA; i++) {
      processingPosition[i] = PC[i];
      positionX[i] = computeDeltaPosition(i);
    }
    for (uint8_t i = 0; i < NBMOTORS; i++) {
      setGoal(i, -positionX[i]);
    }
    writeTargets();
  }


  

}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//================================================================= RECEIVE 24 DATAS FROM PROCESSING

void parseData() {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");  // get the first part - the string
  /*
  for(uint8_t i = 0; i < NBDATA; i++) {
    integerFromPC[i] = atoi(strtokIndx); 
    strtokIndx = strtok(NULL, ",");
  }
  */
  for (uint8_t i = 0; i < NBDATA; i++) {
    PC[i] = atoi(strtokIndx);        // convert this part to an integer
    strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  }

  for (uint8_t i = 0; i < NBDATA; i++) {
    PCTer[i] = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  }

  strtokIndx = strtok(NULL, ",");     // this continues where the previous call left off
  orderTrigger = atoi(strtokIndx);    // convert this part to an integer
  strtokIndx = strtok(NULL, ",");     // this continues where the previous call left off
  orderCohesion = atoi(strtokIndx);   // convert this part to an integer
  strtokIndx = strtok(NULL, ",");     // this continues where the previous call left off
  orderCohesionB = atoi(strtokIndx);  // convert this part to an integer
  strtokIndx = strtok(NULL, ",");     // this continues where the previous call left off
  startStop = atoi(strtokIndx);       // convert this part to an integer
}

void noJoeTransformation() {
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    setGoal(i, PC[i]);
  }
  writeTargets();
}