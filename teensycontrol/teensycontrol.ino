//***************POSITION moteur 0 et PID
#include <AccelStepper.h>  // Define a stepper and the pins it will use
#define NBMOTEURS 6
//#define SerialUSB Serial // to change instance SerialUSB as Serial1

#define NBPASPARTOUR 6400  // Set on the driver

//******* From the behind to the front

// 6400 Teensy
const uint8_t PINDIRECTION[NBMOTEURS] = { 6, 9, 12, 26, 29, 32 };
const uint8_t PINSPEED[NBMOTEURS] = { 5, 8, 11, 25, 28, 31 };
const uint8_t ENABLEPIN[NBMOTEURS] = { 4, 7, 10, 24, 27, 30 };


// Define a stepper and the pins it will use
AccelStepper stepper[NBMOTEURS] = {
  AccelStepper(AccelStepper::DRIVER, PINSPEED[0], PINDIRECTION[0]),
  AccelStepper(AccelStepper::DRIVER, PINSPEED[1], PINDIRECTION[1]),
  AccelStepper(AccelStepper::DRIVER, PINSPEED[2], PINDIRECTION[2]),
  AccelStepper(AccelStepper::DRIVER, PINSPEED[3], PINDIRECTION[3]),
  AccelStepper(AccelStepper::DRIVER, PINSPEED[4], PINDIRECTION[4]),
  AccelStepper(AccelStepper::DRIVER, PINSPEED[5], PINDIRECTION[5]),

};
//   Receive with start- and end-markers combined with parsing

const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = { 0 };  //or 5 doesn't change anything

#define NBDATA 10

int integerFromPC[NBDATA] = {0};
int PC[NBDATA] = {0};
int PCTer[NBDATA] = {0};

int orderTrigger = 0;
int orderCohesion = 0;
int orderCohesionB = 0;
int startStop = 0;

int led = 13;

float floatFromPC = 0.0;  // not used for the moment

boolean newData = false;
//==================================================================== CHECK NUMBEROF ROUND AND GOOD MOVEMENT

#define TX_SIZE 512
uint8_t tx_buffer[TX_SIZE];

//==================================================================    Jonathan computingDelta between actual and previous position.

int processingPosition[NBDATA] = {0};
int positionX[NBDATA] = {0};

int16_t computeDeltaPosition(uint8_t n) {  
  static uint16_t oldPositionAbsolue[NBDATA] = {0};
  uint16_t positionAbsolue = processingPosition[n];
  int16_t resultat;
  if (positionAbsolue < oldPositionAbsolue[n])
    positionAbsolue += NBPASPARTOUR;
  if (positionAbsolue - oldPositionAbsolue[n] < NBPASPARTOUR / 2)
    resultat = positionAbsolue - oldPositionAbsolue[n];
  else
    resultat = positionAbsolue - oldPositionAbsolue[n] - NBPASPARTOUR;
  oldPositionAbsolue[n] = positionAbsolue;
  return resultat;
}

void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


  for(uint8_t i =0; i < NBDATA; i++) {
    processingPosition[i] = PC[i];
  }


  Serial.begin(115200);
  Serial1.begin(115200);


  for (uint8_t i = 0; i < NBMOTEURS - 0; i++) {

    // Initialisation des pins moteurs
    pinMode(ENABLEPIN[i], OUTPUT);
    digitalWrite(ENABLEPIN[i], OUTPUT);
    digitalWrite(ENABLEPIN[i], LOW);

    pinMode(PINDIRECTION[i], OUTPUT);
    digitalWrite(PINDIRECTION[i], OUTPUT);
    pinMode(PINSPEED[i], OUTPUT);
    digitalWrite(PINSPEED[i], OUTPUT);
    //  stepper[i].setMinPulseWidth(5); //****************************************ADD WITH TEENSY if clock very fast


    stepper[i].setMaxSpeed(6400 * 2.5);  // 11000

    stepper[i].setAcceleration(6400 * 1);  //tmc 2209
  }

  int tourTest = 6400 * 1;

  for(uint8_t i=0; i < NBMOTEURS; i++) {
    PC[i] = tourTest * 1;  // premier devant moi
  }

  PCTer[2] = -3;  //noJoe method of Processng data conversion
}


void loop() {

  for (uint8_t i = 0; i < NBMOTEURS - 0; i++) {
    stepper[i].setAcceleration(1600 * PCTer[0]);  //tmc 2209
  }


  recvWithStartEndMarkers();  // recevoir chaine de character et disperse en unité
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    newData = false;
  }
  //--------

  if (PCTer[2] < -1) {  // noJoTransformation   Lire les données telles qu'elles arrivent depuis Processing
    noJoeTransformation();
  } else if (PCTer[2] > -1) {  // transforme data avec la methode de Jo

    for(uint8_t i = 0; i < NBDATA; i++) {
      processingPosition[i] = PC[i];
      positionX[i] = computeDeltaPosition(i);
    }
    
    for(uint8_t i = 0; i < NBMOTEURS; i++) {
      stepper[i].moveTo(-positionX[i]); 
      stepper[i].run();
    }
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
  for(uint8_t i = 0; i < NBDATA; i++) {
    PC[i] = atoi(strtokIndx);      // convert this part to an integer
    strtokIndx = strtok(NULL, ",");// this continues where the previous call left off
  }

  for(uint8_t i = 0; i < NBDATA; i++) {
    PCTer[i] = atoi(strtokIndx);      // convert this part to an integer
    strtokIndx = strtok(NULL, ",");   // this continues where the previous call left off
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
  for(uint8_t i = 0; i < NBMOTEURS; i++) {
    stepper[i].moveTo(PC[i]);
    stepper[i].run();
  }
}