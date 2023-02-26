#include "io.h"
#include "com.h"
#include "processing.h"
#include <Metro.h>  // Include the Metro library

Metro displayMissedStep = Metro(100);  // 100ms
Metro moveMissedStep = Metro(2000);    // 2s

#define PROCESSING  // Uncomment this line to use proccessing, comment this line if you want to use serial demo.
#define AUTORETURN  // Uncomment this line to use auto return feature, comment to desactivate the feature.

void setup() {
  initIo();
  initCom();
  initSteppers();
#ifdef PROCESSING
  initProcessing();
#else
  printSerialCommandsAvailable();
#endif
}

void loop() {

  if (displayMissedStep.check()) {
    toggleLed();
    if (checkMissedStep(false)) {
#ifdef PROCESSING
      displayAllCountSerial1();
#else
      displayAllCountSerial();
#endif
    }
  }

  if (moveMissedStep.check()) {
#ifdef AUTORETURN
    checkMissedStep(true);
#endif
  }

#ifdef PROCESSING
  processingControl();
#else
  readSerial();
#endif
  run();
}
