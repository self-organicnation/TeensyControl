#include "io.h"
#include "com.h"

void initCom() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void printSerialCommandsAvailable() {
  Serial.print("");
  Serial.println("Liste des commandes disponibles VERSION DEV 1.0");
  Serial.println("");
  Serial.println("help= : Montre l'ensemble des commandes serial disponible");
  Serial.println("init= : Lance la séquence d'initialisation");
  Serial.println("stop= : Stop les moteurs ");
  Serial.println("goal=N Steps : Donne l'objectif en step au moteur N");
  Serial.println("go= : Fait bouger tous les moteurs");
  Serial.println("rotate= Fait tourner tous les moteurs à leur vitesse maxspeed");
  Serial.println("override=PERCENT change proportionnellement la vitesse de tous les moteurs en % de maxspeed");
  Serial.println("maxspeed=N Steps : Donne l'objectif en step.s-1 au moteur N");
  Serial.println("accel=N Steps : Donne l'objectif en step.s-2 au moteur N");
  Serial.println("codeur=N : Donne la position du codeur N");
  Serial.println("codeurs= : Donne la position de tous les codeurs");
  Serial.println("mode= : indique le mode de pilotage Teensystepper ou Accelstepper");
 
  Serial.println("");
}

bool readSerial() {
  bool received = false;
  while (Serial.available()) {
    char data = Serial.read();
    addCharToSerialBuffer(data);
    received = true;
  }
  return received;
}

// Read Serial data storage
char cmdOp[NBCHAR] = {""};
char cmdArg[NBCHAR] = {""};
char cmdArg2[NBCHAR] = {""};

/**
   Lors de l'envoi d'une commande, enregistre la commande ainsi que son ou ses arguments
   et lance la fonction parseSerialBuffer() pour traiter la commande
*/
void addCharToSerialBuffer(char c) {
  static uint8_t cmdState = 0;
  static uint8_t cmdOpi = 0;
  static uint16_t cmdArgi = 0;
  static uint16_t cmdArg2i = 0;

  switch (cmdState)
  {
    case 0 :
      if (c == '\n')
        cmdOpi = 0;
      else if (c == '=') {
        cmdOp[cmdOpi] = '\0';
        cmdArgi = 0;
        cmdState = 1;
      }
      else if (cmdOpi < sizeof(cmdOp) - 1)
        cmdOp[cmdOpi++] = c;
      break;

    case 1 :
      if (c == '\n') {
        cmdArg[cmdArgi] = 0;
        cmdArg2[0] = '\0';
        parseSerialBuffer();
        cmdOpi = 0;
        cmdState = 0;
      }
      else if (c == ' ') {
        cmdArg[cmdArgi] = 0;
        cmdOpi = 0;
        cmdArg2i = 0;
        cmdState = 2;
      }
      else if (cmdArgi < sizeof(cmdArg) - 1)
        cmdArg[cmdArgi++] = c;
      break;
    case 2 :
      if (c == '\n') {
        cmdArg2[cmdArg2i] = '\0';
        parseSerialBuffer();
        cmdState = 0;
      }
      else if (cmdArg2i < sizeof(cmdArg2) - 1)
        cmdArg2[cmdArg2i++] = c;
      break;
  }
}

void parseSerialBuffer() {  // Parse serial data, handle all commands available over Serial
  int32_t val = atol(cmdArg);
  int32_t val2 = atol(cmdArg2);

  // help= : Montre l'ensemble des commandes serial disponible
  if (!strcmp(cmdOp, "help")) {
    printSerialCommandsAvailable();
  }
  // init= : Lance la séquence d'initialisation
  else if (!strcmp(cmdOp, "init")) {
    Serial.println("init");
    initSteppers();
  }
  // stop= : Stop les moteurs 
  else if (!strcmp(cmdOp, "stop")) {
    Serial.println("stop");
    stopMotors();
  } 
  // goal=N Steps : Donne l'objectif en step au moteur N
  else if (!strcmp(cmdOp, "goal")) {
    printCmdValVal("goal", val, val2);
    setGoal(val, val2);
  }
  // go= : Fait bouger tous les moteurs
  else if (!strcmp(cmdOp, "go")) {
    Serial.println("go");
    moveMotorsAsync();
  }
  // rotate= Fait tourner tous les moteurs à leur vitesse maxspeed
  else if (!strcmp(cmdOp, "rotate")) {
    Serial.println("rotate");
    rotateMotors();
  }
  // override=PERCENT change proportionnellement la vitesse de tous les moteurs en % de maxspeed
  else if (!strcmp(cmdOp, "override")) {
    printCmdVal("override", val);
    overrideSpeed(val);
  } 
  // maxspeed=N Steps : Donne l'objectif en step.s-1 au moteur N
  else if (!strcmp(cmdOp, "maxspeed")) {
    printCmdValVal("maxspeed", val, val2);
    setMaxspeed(val, val2);
  }
  // accel=N Steps : Donne l'objectif en step.s-2 au moteur N
  else if (!strcmp(cmdOp, "accel")) {
    printCmdValVal("accel", val, val2);
    setAccel(val, val2);
  }
  // codeur=N : Donne la position du codeur N
  else if (!strcmp(cmdOp, "codeur")) {
    printCmdValVal("codeur", val, getCount(val));
  }
  // codeurs= : Donne la position de tous les codeurs
  else if (!strcmp(cmdOp, "codeurs")) {
    for(uint8_t i=0; i < NBMOTORS; i++)
      printCmdValVal("codeur", i, getCount(i));
  }
  //mode= : indique le mode de pilotage Teensystepper ou Accelstepper
  else if (!strcmp(cmdOp, "mode")) {
    displayMode();
  }
  // unknown => stop motors
  else {
    Serial.println("Unknown => stop");
    stopMotors();
  }
}

void printCmdVal( String cmd, int32_t val) {
  Serial.print(cmd);
  Serial.print("=");
  Serial.print(val);
  Serial.print("\n");
}

void printCmdValVal( String cmd, int32_t val1, int32_t val2) {
  Serial.print(cmd);
  Serial.print("=");
  Serial.print(val1);
  Serial.print(" ");
  Serial.print(val2);
  Serial.print("\n");
}