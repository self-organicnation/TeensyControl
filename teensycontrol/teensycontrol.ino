#include "io.h"
#include "com.h"
#include "processing.h"
#include <Metro.h> // Include the Metro library

// int timeElapsedBeforeAdaptPositionFromProcessing;
int timeElapsedBeforeAdaptPosition = 8000;
int missedStepInterval;

Metro displayMissedStep = Metro(30);                          // 30ms
Metro moveMissedStep = Metro(timeElapsedBeforeAdaptPosition); // 1seconde

#define PROCESSING // Uncomment this line to use proccessing, comment this line if you want to use serial demo.
#define AUTORETURN // Uncomment this line to use auto return feature, comment to desactivate the feature.

void setup()
{
  initIo();
  initCom();
  initSteppers();
#ifdef PROCESSING
  initProcessing();
#else
  printSerialCommandsAvailable();
#endif
}

void loop()
{
  if (displayMissedStep.check())
  {
    toggleLed();

//    if (checkMissedStep(false)) { // permet d'avoir les positions tous le temps sur le port serie1
#ifdef PROCESSING
    displayAllCountSerial1();
    checkMissedStep(false);
#else
    displayAllCountSerial();
#endif
    //    }
  }

  if (moveMissedStep.check())
  {
#ifdef AUTORETURN
    checkMissedStep(true);
#endif
  }

#ifdef PROCESSING
  processingControl();

  /*
    if ( PCTer[5] != missedStepInterval ) {
       moveMissedStep.interval(PCTer[5]);
       missedStepInterval = PCTer[5];
      }
  */

  // timeElapsedBeforeAdaptPosition=  timeElapsedBeforeAdaptPositionFromProcessing;

  if (timeElapsedBeforeAdaptPosition != missedStepInterval)
  {
    moveMissedStep.interval(timeElapsedBeforeAdaptPosition);
    missedStepInterval = timeElapsedBeforeAdaptPosition;
  }

#else
  readSerial();
#endif
  run();
}