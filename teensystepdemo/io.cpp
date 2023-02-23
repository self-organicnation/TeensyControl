#include "io.h"
#include <TeensyStep.h>

void initIo() {
  pinMode(LED, OUTPUT);
  initSteppersPins();
  initEncoders();
}

void toggleLed() {
  static bool ledState = LOW;
  ledState = !ledState;
  digitalWrite(LED, ledState);
}

// STEPPER

#define NBSTEPSPERTURN 6400
#define MAXSPEED 6400  // Max 300 000
#define ACCEL 40000    // Max 500 000

Stepper stepperMotors[NBMOTORS] = { Stepper(STEP[0], DIR[0]), Stepper(STEP[1], DIR[1]), Stepper(STEP[2], DIR[2]), Stepper(STEP[3], DIR[3]), Stepper(STEP[4], DIR[4]), Stepper(STEP[5], DIR[5]) };
StepControl controller;


RotateControl rotateController(1, 500);

bool isInit[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };      //Etat précisant si le moteur a été initialisé
int32_t goal[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };
uint32_t maxspeed[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };
uint32_t accel[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };
volatile int32_t zeroPos[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };        //Position zéro enregistrée quand Z passe de 1 à 0
//const int32_t offsets[NBMOTORS] = {1300, 3720, 5120, 4720, 4370, 5920}; // => Boitier 1 ( étiquette)
const int32_t offsets[NBMOTORS] = {3990, 1030, 585, 1430, 750, 2050}; // => Boitier 2

void initSteppersPins() {
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    pinMode(EN[i], OUTPUT);
    digitalWrite(EN[i], LOW);
    pinMode(DIR[i], OUTPUT);
    digitalWrite(DIR[i], LOW);
    pinMode(STEP[i], OUTPUT);
    digitalWrite(STEP[i], LOW);
    setMaxspeed(i, MAXSPEED);
    setAccel(i, ACCEL);
  }
}

void initSteppers() {
  rotateController.stop();
  for(uint8_t i=0; i < NBMOTORS; i++) {
    setMaxspeed(i, MAXSPEED);
    setStep(i,0);
    setGoal(i, NBSTEPSPERTURN);
  }
  moveMotors();
  for(uint8_t i=0; i < NBMOTORS; i++) {
    int32_t val =(zeroPos[i] + offsets[i]) % NBSTEPSPERTURN;
    if(val < NBSTEPSPERTURN / 2)
      val += NBSTEPSPERTURN;
    setGoal(i,val);
  }
  moveMotors();
  for(uint8_t i=0; i < NBMOTORS; i++) {
    setStep(i,0);
    setGoal(i,0);
    setCount(i,0);
  }
  Serial.println("initDone");
}

void setStep(uint8_t motorNumber, int32_t val) {
  stepperMotors[motorNumber].setPosition(val);  // TeensyStep
  //stepperMotors[motorNumber].setCurrentPosition(val); // accelStepper
}

void setGoal(uint8_t motorNumber, int32_t val) {
  goal[motorNumber] = val;
}

void setMaxspeed(uint8_t motorNumber, int32_t val) {
  maxspeed[motorNumber] = val;
  stepperMotors[motorNumber].setMaxSpeed(maxspeed[motorNumber]);  // steps/s
}

void setAccel(uint8_t motorNumber, int32_t val) {
  accel[motorNumber] = val;  // steps/s^2
  stepperMotors[motorNumber].setAcceleration(accel[motorNumber]);
}

void moveMotors() {
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    stepperMotors[i].setTargetAbs(goal[i]);
    //stepperMotors[i].moveTo(goal[i]); // AccelStepper
  }
  controller.move(stepperMotors[0], stepperMotors[1], stepperMotors[2], stepperMotors[3], stepperMotors[4], stepperMotors[5]);
}

void moveMotorsAsync() {
  rotateController.stop();
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    stepperMotors[i].setTargetAbs(goal[i]);
    //stepperMotors[i].moveTo(goal[i]); // AccelStepper
  }
  controller.moveAsync(stepperMotors[0], stepperMotors[1], stepperMotors[2], stepperMotors[3], stepperMotors[4], stepperMotors[5]);
}


void rotateMotors() {
  controller.stop();
  rotateController.stop();
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    stepperMotors[i].setMaxSpeed(maxspeed[i]);
  }
  rotateController.rotateAsync(stepperMotors[0], stepperMotors[1], stepperMotors[2], stepperMotors[3], stepperMotors[4], stepperMotors[5]);
}

void overrideSpeed(int percent) {
  float ratio = percent / 100.0;
  rotateController.overrideSpeed(ratio); 
}

void stopMotors(){
  overrideSpeed(0);
  rotateController.stop();
  controller.stop();
}

int32_t getStep(uint8_t motorNumber) {
  return stepperMotors[motorNumber].getPosition();  // TeensyStep
  // return stepperMotors[motorNumber].currentPosition(); // accelStepper
}


// Encodeurs

volatile int32_t count[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };          //Comptage des tics d'encodeur qui est incrémenté à chaque interruption
volatile int16_t rotationSpeed[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };  //Vitesse de rotation extraite de la variable count
volatile bool laststateA[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };        //Etat précédent des sorties A des enncodeurs
volatile bool laststateB[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };        //Etat précédent des sorties B des enncodeurs
volatile bool laststateZ[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };        //Etat précédent des sorties Z des enncodeurs


void counterA(uint8_t motorNumber) {
  bool stateA = digitalRead(ENCODERAS[motorNumber]);
  (stateA ^ laststateB[motorNumber]) ? count[motorNumber]++ : count[motorNumber]--;
  laststateA[motorNumber] = stateA;
}

void counterB(uint8_t motorNumber) {
  bool stateB = digitalRead(ENCODERBS[motorNumber]);
  (stateB ^ laststateA[motorNumber]) ? count[motorNumber]-- : count[motorNumber]++;
  laststateB[motorNumber] = stateB;
}

void interruptZ(uint8_t motorNumber) {
  bool stateZ = digitalRead(ENCODERZS[motorNumber]);
  zeroPos[motorNumber] = getStep(motorNumber);
  laststateZ[motorNumber] = stateZ;
}

void counterA0() {
  counterA(0);
}
void counterA1() {
  counterA(1);
}
void counterA2() {
  counterA(2);
}
void counterA3() {
  counterA(3);
}
void counterA4() {
  counterA(4);
}
void counterA5() {
  counterA(5);
}
void counterB0() {
  counterB(0);
}
void counterB1() {
  counterB(1);
}
void counterB2() {
  counterB(2);
}
void counterB3() {
  counterB(3);
}
void counterB4() {
  counterB(4);
}
void counterB5() {
  counterB(5);
}
void interruptZ0() {
  interruptZ(0);
}
void interruptZ1() {
  interruptZ(1);
}
void interruptZ2() {
  interruptZ(2);
}
void interruptZ3() {
  interruptZ(3);
}
void interruptZ4() {
  interruptZ(4);
}
void interruptZ5() {
  interruptZ(5);
}

void (*counterEncoderA[])(void) = { counterA0, counterA1, counterA2, counterA3, counterA4, counterA5 };
void (*counterEncoderB[])(void) = { counterB0, counterB1, counterB2, counterB3, counterB4, counterB5 };
void (*interruptEncoderZ[])(void) = { interruptZ0, interruptZ1, interruptZ2, interruptZ3, interruptZ4, interruptZ5 };

void initEncoders() {
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    pinMode(ENCODERAS[i], INPUT_PULLUP);
    pinMode(ENCODERBS[i], INPUT_PULLUP);
    pinMode(ENCODERZS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODERAS[i]), counterEncoderA[i], CHANGE);    //S'active lorsqu'il y a un changement d'état sur la pin de l'encodeur A
    attachInterrupt(digitalPinToInterrupt(ENCODERBS[i]), counterEncoderB[i], CHANGE);    //S'active lorsqu'il y a un changement d'état sur la pin de l'encodeur B
    attachInterrupt(digitalPinToInterrupt(ENCODERZS[i]), interruptEncoderZ[i], FALLING); //S'active lorsqu'il y a un changement d'état haut vers bas sur la pin de l'encodeur Z
  }
}

int32_t getCount(uint8_t motorId) {
  return count[motorId];
}

void setCount(uint8_t motorId, int32_t val) {
  count[motorId] = val;
}

void displayAllCountSerial1() {
  Serial1.print("<");
  for(uint8_t i=0; i < NBMOTORS - 1; i++) {
    Serial1.print(getCount(i));
    Serial1.print(",");
  }
  Serial1.print(getCount(NBMOTORS - 1));
  Serial1.println(">");
}
