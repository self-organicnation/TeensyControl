#include "io.h"

#define INITSPEED 6400
#define INITACCEL 6400
#define MAXSPEED 196000  // Max value = 300 000 but too fast...
#define ACCEL 6400   //  12800 // Max value = 500 000 but too much... 6400 for accelstepper ?  6400 * 3 for DM556

#define ACCELSTEP //TEENSYSTEP //ACCELSTEP

void displayMode() {
#ifdef TEENSYSTEP
  Serial.println("Teensystep");
#endif
#ifdef ACCELSTEP
  Serial.println("Accelstep");
#endif
}

#ifdef TEENSYSTEP

#include <TeensyStep.h>

Stepper stepperMotors[NBMOTORS] = { Stepper(STEP[0], DIR[0]), Stepper(STEP[1], DIR[1]), Stepper(STEP[2], DIR[2]), Stepper(STEP[3], DIR[3]), Stepper(STEP[4], DIR[4]), Stepper(STEP[5], DIR[5]) };
StepControl controller;
RotateControl rotateController(1, 500);

#endif

#ifdef ACCELSTEP

#include <AccelStepper.h>  // Define a stepper and the pins it will use

AccelStepper stepperMotors[NBMOTORS] = {
  AccelStepper(AccelStepper::DRIVER, STEP[0], DIR[0]),
  AccelStepper(AccelStepper::DRIVER, STEP[1], DIR[1]),
  AccelStepper(AccelStepper::DRIVER, STEP[2], DIR[2]),
  AccelStepper(AccelStepper::DRIVER, STEP[3], DIR[3]),
  AccelStepper(AccelStepper::DRIVER, STEP[4], DIR[4]),
  AccelStepper(AccelStepper::DRIVER, STEP[5], DIR[5]),
};

#endif


bool positionMode = true;
bool isInit[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };  //Etat précisant si le moteur a été initialisé
int32_t goal[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };
uint32_t maxspeed[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };
uint32_t accel[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };
volatile int32_t zeroPos[NBMOTORS] = { 0, 0, 0, 0, 0, 0 };  //Position zéro enregistrée quand Z passe de 1 à 0
//const int32_t offsets[NBMOTORS] = { 1300, 3720, 5120, 4720, 4370, 5920 };  // => Boitier 1 ( étiquette)
const int32_t offsets[NBMOTORS] = { 3990, 1030, 585, 1430, 750, 2050 };  // => Boitier 2
int32_t errorArray[NBMOTORS] = { 0, 0, 0, 0, 0, 0 }; 
int32_t errorTargetArray[NBMOTORS] = { 0, 0, 0, 0, 0, 0 }; 

void initSteppersPins() {
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    pinMode(EN[i], OUTPUT);
    digitalWrite(EN[i], LOW);
    pinMode(DIR[i], OUTPUT);
    digitalWrite(DIR[i], LOW);
    pinMode(STEP[i], OUTPUT);
    digitalWrite(STEP[i], LOW);
  }
}


void initSteppers() {
  positionMode = true;
  stopMotors();
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    setMaxspeed(i, INITSPEED);
    setAccel(i, INITACCEL);
    setStep(i, 0);
    setGoal(i, NBSTEPSPERTURN);
  }
  moveMotors();
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    int32_t val = (zeroPos[i] + offsets[i]) % NBSTEPSPERTURN;
    if (val < NBSTEPSPERTURN / 2)
      val += NBSTEPSPERTURN;
    setGoal(i, val);
  }
  moveMotors();
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    setStep(i, 0);
    setGoal(i, 0);
    setCount(i, 0);
  }
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    setMaxspeed(i, MAXSPEED);
    setAccel(i, ACCEL);
  }
  Serial.println("initDone");
}

void setStep(uint8_t motorNumber, int32_t val) {
#ifdef TEENSYSTEP
  stepperMotors[motorNumber].setPosition(val);  // TeensyStep
#endif
#ifdef ACCELSTEP
  stepperMotors[motorNumber].setCurrentPosition(val);  // accelStepper
#endif
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

void writeTarget(uint8_t n) {
#ifdef TEENSYSTEP
  stepperMotors[n].setTargetAbs(goal[n]);  // Teensystep
#endif
#ifdef ACCELSTEP
  stepperMotors[n].setMaxSpeed(maxspeed[n]);  // in case maxspeed have been override with override function
  stepperMotors[n].moveTo(goal[n]);           // AccelStepper
#endif
}

void writeTargets() {
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    writeTarget(i);
  }
}
void moveMotors() {
  positionMode = true;
#ifdef TEENSYSTEP
  rotateController.stop();
  controller.stop();
#endif
  writeTargets();
#ifdef TEENSYSTEP
  controller.move(stepperMotors[0], stepperMotors[1], stepperMotors[2], stepperMotors[3], stepperMotors[4], stepperMotors[5]);
#endif
#ifdef ACCELSTEP
  bool end = false;
  while (!end) {
    end = true;
    for (uint8_t i = 0; i < NBMOTORS; i++) {
      if (stepperMotors[i].distanceToGo())
        end = false;
    }
    run();
  }
#endif
}

void run() {
#ifdef ACCELSTEP
  if (positionMode) {
    for (uint8_t i = 0; i < NBMOTORS; i++)
      stepperMotors[i].run();
  } else {
    for (uint8_t i = 0; i < NBMOTORS; i++)
      stepperMotors[i].runSpeed();
  }
#endif
}

void moveMotorsAsync() {
  positionMode = true;
#ifdef TEENSYSTEP
  rotateController.stop();
  controller.stop();
#endif
  writeTargets();
#ifdef TEENSYSTEP
  controller.moveAsync(stepperMotors[0], stepperMotors[1], stepperMotors[2], stepperMotors[3], stepperMotors[4], stepperMotors[5]);
#endif
}


void rotateMotors() {
  positionMode = false;
#ifdef TEENSYSTEP
  controller.stop();
  rotateController.stop();
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    stepperMotors[i].setMaxSpeed(maxspeed[i]);
  }
  rotateController.rotateAsync(stepperMotors[0], stepperMotors[1], stepperMotors[2], stepperMotors[3], stepperMotors[4], stepperMotors[5]);
#endif
#ifdef ACCELSTEP
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    stepperMotors[i].setMaxSpeed(maxspeed[i]);
    stepperMotors[i].setSpeed(maxspeed[i]);
  }
#endif
}

void overrideSpeed(int percent) {
#ifdef TEENSYSTEP
  float ratio = percent / 100.0;
  rotateController.overrideSpeed(ratio);
#endif
#ifdef ACCELSTEP
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    int32_t speed = maxspeed[i] * percent / 100;
    stepperMotors[i].setMaxSpeed(speed);
    stepperMotors[i].setSpeed(speed);
  }
#endif
}

void stopMotors() {
  positionMode = true;
#ifdef TEENSYSTEP
  overrideSpeed(0);
  rotateController.stop();
  controller.stop();
#endif
#ifdef ACCELSTEP
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    stepperMotors[i].stop();
    setGoal(i, getStep(i));
  }
  writeTargets();
#endif
}

int32_t getStep(uint8_t motorNumber) {
#ifdef TEENSYSTEP
  return stepperMotors[motorNumber].getPosition();  // TeensyStep
#endif
#ifdef ACCELSTEP
  return stepperMotors[motorNumber].currentPosition();  // accelStepper
#endif
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
    attachInterrupt(digitalPinToInterrupt(ENCODERAS[i]), counterEncoderA[i], CHANGE);     //S'active lorsqu'il y a un changement d'état sur la pin de l'encodeur A
    attachInterrupt(digitalPinToInterrupt(ENCODERBS[i]), counterEncoderB[i], CHANGE);     //S'active lorsqu'il y a un changement d'état sur la pin de l'encodeur B
    attachInterrupt(digitalPinToInterrupt(ENCODERZS[i]), interruptEncoderZ[i], FALLING);  //S'active lorsqu'il y a un changement d'état haut vers bas sur la pin de l'encodeur Z
  }
}

int32_t getCount(uint8_t motorId) {
  return count[motorId];
}

void setCount(uint8_t motorId, int32_t val) {
  count[motorId] = val;
}

void toggleLed() {
  static bool ledState = LOW;
  ledState = !ledState;
  digitalWrite(LED, ledState);
}

void initIo() {
  pinMode(LED, OUTPUT);
  initSteppersPins();
  initEncoders();
}

void displayAllCountSerial() {
  Serial.print("<");
  for (uint8_t i = 0; i < NBMOTORS - 1; i++) {
    Serial.print(getCount(i));
    Serial.print(",");
  }
  Serial.print(getCount(NBMOTORS - 1));
  Serial.println(">");
}

void displayAllCountSerial1() {
  for (uint8_t i = 0; i < NBMOTORS-1; i++) {
    Serial1.print(getCount(i));
     Serial.print(",");
  }
  Serial.print(getCount(NBMOTORS - 1));
  Serial.println(">");
}

void rewriteCurrentStepWhileMoving(uint8_t n, int32_t step) {
  setStep(n, step);
  writeTarget(n);
  if (positionMode)
    moveMotorsAsync();
}

uint8_t checkMissedStep(bool rewrite) {
  uint8_t result = 0;
//  int32_t  errorTargetArray[NBMOTORS];
    
  for (uint8_t i = 0; i < NBMOTORS; i++) {
    int32_t step = getStep(i);
    int32_t count = getCount(i);
    int32_t error = count - step * 4000 / 6400;
    errorArray[i] = count - step * 4000 / 6400;
    int32_t errorTarget = count - goal[i] * 4000 / 6400;

 //     if (abs(error) > 20 && ((count - goal[0] * 4000 / 6400)>=5) ||
 //     ((count - goal[i] * 4000 / 6400)<=5))
 
 //   int32_t  errorTargetArray[NBMOTORS];
    
    errorTargetArray[i]=count - goal[i] * 4000 / 6400;

        if (abs(errorArray[0]) > 20 && (abs(errorTargetArray[0]) > 5 ))    
      {
        Serial1.println(" encodeur0 ");
      }
    
        if (abs(errorArray[1]) > 20 && (abs(errorTargetArray[1]) > 5 ))      
      {
        Serial1.println(" encodeur1 ");
      }

        if (abs(errorArray[2]) > 20 && (abs(errorTargetArray[2]) > 5 ))    
      {
        Serial1.println(" encodeur2 ");
      }
 
        if (abs(errorArray[3]) > 20 && (abs(errorTargetArray[3]) > 5 ))      
      {
        Serial1.println(" encodeur3 ");
      }
        if (abs(errorArray[4]) > 20 && (abs(errorTargetArray[4]) > 5 ))    
      {
        Serial1.println(" encodeur4 ");
      }
        if (abs(errorArray[5]) > 20 && (abs(errorTargetArray[5]) > 5 ))      
      {
        Serial1.println(" encodeur5 ");
      }


    if (abs(error) > 20) {
      result += (1 << i);
      /*
      Serial.print("motor ");
      Serial.print(i);
      Serial.print(" => count = ");
      Serial.print(count);
      Serial.print(" => step =");
      Serial.print(step);
      Serial.print(" => error = ");
      Serial.println(error);
      */
      /*
      Serial1.print("motor ");
      Serial1.print(i);
      Serial1.print(" => count = ");
      Serial1.print(count);
      Serial1.print(" => step =");
      Serial1.print(step);
      Serial1.print(" => error = ");
      Serial1.println(error);
      */
      if (rewrite) {
        if (errorTarget >= 4000) {
          count -= 4000;
          setCount(i, count);
        } else if (errorTarget <= -4000) {
          count += 4000;
          setCount(i, count);
        }
        rewriteCurrentStepWhileMoving(i, count * 6400 / 4000);
      }
    }
  }


  
  return result;
}