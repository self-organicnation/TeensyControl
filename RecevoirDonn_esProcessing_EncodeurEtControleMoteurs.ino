//***************POSITION moteur 0 et PID
 
#include <AccelStepper.h> // Define a stepper and the pins it will use
#define NBMOTEURS 6
//#define SerialUSB Serial // to change instance SerialUSB as Serial1

#define NBPASPARTOUR 6400 // Set on the driver
 
#define STEP 1//// 800-->Mode 1/4 pas MS2 sur ON
//  1600-->Mode 1/8 MS1+ MS2 sur ON
// 3200--> Mode 1/16 MS3 sur ON
// 6400--> Mode 1/32 MS1+ MS3 sur ON

 //******* From the behind to the front

// 6400 Teensy  
 const uint8_t PINDIRECTION[NBMOTEURS] = {6, 9, 12, 26, 29, 32};      
 const uint8_t PINSPEED[NBMOTEURS]=      {5, 8, 11, 25, 28, 31};    
 const uint8_t ENABLEPIN[NBMOTEURS]=     {4, 7, 10, 24, 27, 30}; 


// Define a stepper and the pins it will use 
AccelStepper stepper[ NBMOTEURS] = {
AccelStepper (STEP, PINSPEED[0], PINDIRECTION[0]), 
AccelStepper (STEP, PINSPEED[1], PINDIRECTION[1]), 
AccelStepper (STEP, PINSPEED[2], PINDIRECTION[2]), 
AccelStepper (STEP, PINSPEED[3], PINDIRECTION[3]), 
AccelStepper (STEP, PINSPEED[4], PINDIRECTION[4]),  
AccelStepper (STEP, PINSPEED[5], PINDIRECTION[5]), 

};
//   Receive with start- and end-markers combined with parsing

const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0}; //or 5 doesn't change anything

int integerFromPC0 = 0;
int integerFromPC1 = 0;
int integerFromPC2 = 0;
int integerFromPC3 = 0;
int integerFromPC4 = 0;
int integerFromPC5 = 0;
int integerFromPC6 = 0;
int integerFromPC7 = 0;
int integerFromPC8 = 0;
int integerFromPC9 = 0;

int PC0= 0;
int PC1= 0;
int PC2= 0;
int PC3= 0;
int PC4= 0;
int PC5= 0;
int PC6= 0;
int PC7= 0;
int PC8= 0;
int PC9= 0;


int PCTer0= 0;
int PCTer1= 0;
int PCTer2= 0;
int PCTer3= 0;
int PCTer4= 0;
int PCTer5= 0;
int PCTer6= 0;
int PCTer7= 0;
int PCTer8= 0;
int PCTer9= 0;
int PCTer10= 0;
int PCTer11= 0;

int orderTrigger = 0;
int orderCohesion  = 0;
int orderCohesionB = 0;
int startStop = 0;

int led = 13;
 
float floatFromPC = 0.0; // not used for the moment

boolean newData = false;
//==================================================================== CHECK NUMBEROF ROUND AND GOOD MOVEMENT


int tourTest;
// IntervalTimer t1;

#define TX_SIZE 512
uint8_t tx_buffer[TX_SIZE];

//==================================================================    Jonathan computingDelta between actual and previous position.

int processingPosition, processingPosition1,processingPosition2, processingPosition3, processingPosition4; 
int processingPosition5, processingPosition6, processingPosition7, processingPosition8, processingPosition9;


int positionX, positionX1, positionX2, positionX3,positionX4,positionX5, positionX6, positionX7, positionX8, positionX9; 
int previousX, previousX1; //  positionX precedente


int16_t computeDeltaPosition( uint16_t processingPosition) { //Moteur 11, le plus à droite des drivers
      static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR*1;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR*1/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR*1;
  oldPositionAbsolue = processingPosition;
  return resultat;

}

int16_t computeDeltaPosition1( uint16_t processingPosition1) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition1;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR*1;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR*1/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR*1;
  oldPositionAbsolue = processingPosition1;
  return resultat;

}

int16_t computeDeltaPosition2( uint16_t processingPosition2) {
   static uint16_t oldPositionAbsolue = 0;
    uint16_t positionAbsolue = processingPosition2;
    int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR*1;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR*1/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR*1;
  oldPositionAbsolue = processingPosition2;
  return resultat;

}


int16_t computeDeltaPosition3( uint16_t processingPosition3) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition3;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition3;
  return resultat;

}
int16_t computeDeltaPosition4( uint16_t processingPosition4) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition4;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition4;
  return resultat;

}

int16_t computeDeltaPosition5( uint16_t processingPosition5) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition5;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition5;
  return resultat;

}

int16_t computeDeltaPosition6( uint16_t processingPosition6) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition6;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition6;
  return resultat;

}

int16_t computeDeltaPosition7( uint16_t processingPosition7) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition7;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition7;
  return resultat;

}
int16_t computeDeltaPosition8( uint16_t processingPosition8) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition8;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition8;
  return resultat;

}

int16_t computeDeltaPosition9( uint16_t processingPosition9) {
   static uint16_t oldPositionAbsolue = 0;
   uint16_t positionAbsolue = processingPosition9;
   int16_t resultat;
   if(positionAbsolue < oldPositionAbsolue)
    positionAbsolue+=NBPASPARTOUR;
   if(positionAbsolue - oldPositionAbsolue < NBPASPARTOUR/2)
   resultat = positionAbsolue - oldPositionAbsolue;
  else
    resultat = positionAbsolue - oldPositionAbsolue - NBPASPARTOUR;
  oldPositionAbsolue = processingPosition9;
  return resultat;

}

void setup() {

  
 // initialize digital pin LED_BUILTIN as an output.

 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
 // pinMode(led, OUTPUT);

    // It's to fix the good period time between two ticks
 //    t1.priority(16);  //32
   //I have to change this value in the file serial.c.
   //Later we will can change this directly in the setup
   //This file is a folder in the application teensy
//   NVIC_SET_PRIORITY(IRQ_UART0_STATUS, 16); //32  

//  t1.begin(tickSteppers, 100   );  // call every 100µs, change if requred OK. If it's shorter it's preciser
  // I don't use this fonction but i let it if i need

  processingPosition =  PC0;
  processingPosition1 = PC1;
  processingPosition2 = PC2;
  processingPosition3 = PC3;
  processingPosition4 = PC4;
  processingPosition5 = PC5;
  processingPosition6 = PC6;
  processingPosition7 = PC7;
  processingPosition8 = PC8;
  processingPosition9 = PC9; //K

    Serial.begin (115200);
    Serial1.begin (115200);

   
 for(uint8_t i = 0; i < NBMOTEURS-0; i++) { 

    // Initialisation des pins moteurs 
    pinMode(ENABLEPIN[i], OUTPUT);
    digitalWrite(ENABLEPIN[i], OUTPUT);
    digitalWrite(ENABLEPIN[i], LOW);
    
    pinMode(PINDIRECTION[i], OUTPUT);
    digitalWrite(PINDIRECTION[i], OUTPUT);
    pinMode(PINSPEED[i], OUTPUT);
    digitalWrite(PINSPEED[i], OUTPUT);
  //  stepper[i].setMinPulseWidth(5); //****************************************ADD WITH TEENSY if clock very fast

 // stepper[i].setMaxSpeed(25000); // 11000
 //*** stepper[i].setMaxSpeed(60000/2); // 11000
   stepper[i].setMaxSpeed(6400*2.5); // 11000
 //  stepper[i].setMaxSpeed(NBPASPARTOUR*20); // 11000
 // stepper[i].setAcceleration(12800);  // 12000
 // stepper[i].setMaxSpeed(19800); // 3.5 round/s
// stepper[i].setMaxSpeed(3200); // 3.5 round/s

//  stepper[i].setAcceleration(NBPASPARTOUR*5);  // 12000
//*** stepper[i].setAcceleration(40000/2);  // 12000
   stepper[i].setAcceleration(6400*1);  //tmc 2209
 //    stepper[i].setAcceleration(6400*3);  // augmente acceleration avec DM556
   //   stepper[i].setAcceleration(12000); 
   //  stepper[i].setAcceleration(4800); 
  //   stepper[i].setAcceleration(3200);  // poiur satie

     }
     
 tourTest=6400*1;
 /*
 positionX9= tourTest*2; // premier
 
 positionX8=tourTest*3;  // DM556
 positionX7=tourTest;

 positionX6=tourTest;
 
 positionX5=tourTest;
 positionX4=tourTest*3;
 
 positionX3=tourTest*1;
 positionX2=tourTest*1;
 positionX1=tourTest;
 
 positionX=tourTest*-3; // premier devant moi
*/

//PCTer1=1; //off
 PCTer1=3; //on

 
// PCTer3=1; //off
  PCTer3=3; //on


 PCTer2= 2; // JoeTransfo
 PCTer2= -2; // noJoeTransfo

 PC0=tourTest*1; // premier devant moi
 
  PC9=PC0; //PC9 dernier moteur
  PC8=PC0;
  PC7=PC0;
  PC6=PC0;
  PC5=PC0;
  PC4=PC0;
  PC3=PC0;
  PC2=PC0;
  PC1=PC0; //PC0 premier moteur

  PCTer2 = -3; //noJoe method of Processng data conversion
   }


void loop() { 
/*
  if (PCTer1 > 0) {
 digitalWrite(ENABLEPIN[0], LOW); // you turn on the driver
  } 
  if (PCTer1 < 1) {
 digitalWrite(ENABLEPIN[0], HIGH); // you turn off the driver
  } 
  if (PCTer3 > 0) {
 digitalWrite(ENABLEPIN[1], LOW); // you turn on the driver
  } 
  if (PCTer3 < 1) {
 digitalWrite(ENABLEPIN[1], HIGH); // you turn off the driver
  } 
*/
 

 for(uint8_t i = 0; i < NBMOTEURS-0; i++) { 

  //  stepper[i].setMinPulseWidth(5); //****************************************IN SETUP ADD WITH TEENSY 4.0 ( if clock very fast, steppermotors don't turn)

   stepper[i].setAcceleration(1600*PCTer0);  //tmc 2209

   } 

   
   recvWithStartEndMarkers(); // recevoir chaine de character et disperse en unité 
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();       
        newData = false;
     
    }
   //--------
    
   readDataFromProcessingToSerial1(); // READ 30 datas even if there is ony 24 datas from Processing
   //________
 
    
      if (PCTer2 < -1) {  // noJoTransformation   Lire les données telles qu'elles arrivent depuis Processing
 noJoeTransformation();
  } 
 else if (PCTer2 > -1) { // transforme data avec la methode de Jo
   
  processingPosition =  PC0;
  processingPosition1 = PC1;
  processingPosition2 = PC2;//R
  processingPosition3 = PC3;
  processingPosition4 = PC4;
  processingPosition5 = PC5;
  processingPosition6 = PC6;
  processingPosition7 = PC7;
  processingPosition8 = PC8;
  processingPosition9 = PC9;//K
  
  positionX += computeDeltaPosition(processingPosition);
  positionX1+= computeDeltaPosition1(processingPosition1);
  positionX2+= computeDeltaPosition2(processingPosition2);
  positionX3+= computeDeltaPosition3(processingPosition3);
  positionX4+= computeDeltaPosition4(processingPosition4);
  positionX5+= computeDeltaPosition5(processingPosition5);
  positionX6+= computeDeltaPosition6(processingPosition6);
  positionX7+= computeDeltaPosition7(processingPosition7);
  positionX8+= computeDeltaPosition8(processingPosition8);
  positionX9+= computeDeltaPosition9(processingPosition9);

   /*
       stepper[9].moveTo(-positionX9);
       stepper[9].run();
      
       stepper[8].moveTo(-positionX8);
       stepper[8].run();    
  
       stepper[7].moveTo(-positionX7);
       stepper[7].run();
   
       stepper[6].moveTo(-positionX6);
       stepper[6].run();
   */  
       stepper[5].moveTo(-positionX5); // 
       stepper[5].run();
         
       stepper[4].moveTo(-positionX4);
       stepper[4].run();
       
       stepper[3].moveTo(-positionX3);
       stepper[3].run();  
            
       stepper[2].moveTo(-positionX2);
       stepper[2].run();
       
       stepper[1].moveTo(-positionX1);
       stepper[1].run();
  
       
       stepper[0].moveTo(-positionX); // premiere donnée envoyée dansla chaine de String de Processing 
       stepper[0].run();             // plus pres de moi. oscillator 11
    


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
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
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

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
 /*
    integerFromPC0 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC1 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC2 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC3 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC4 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC5 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC6 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC7 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC8 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC9 = atoi(strtokIndx);     // convert this part to an integer
*/

 //   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC0 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC1 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC2 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC3 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC4 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC5 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC6 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC7 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC8 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PC9 = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer0 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer1 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer2 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer3 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer4 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer5 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer6 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer7 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer8 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    PCTer9 = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    orderTrigger = atoi(strtokIndx); // convert this part to an integer 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    orderCohesion  = atoi(strtokIndx); // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    orderCohesionB  = atoi(strtokIndx); // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    startStop = atoi(strtokIndx); // convert this part to an integer

}

//============


 void readDataFromProcessingToSerial1() {  // READ 30 datas even if there is ony 24 datas from Processing

// DONT FORGET SerialUSB.println at the end of each line!!!!! 
    Serial1.print ("A "); Serial1.println (integerFromPC0);      
    Serial1.print ("B "); Serial1.println (integerFromPC1);         
    Serial1.print ("C "); Serial1.println (integerFromPC2);    
    Serial1.print ("D "); Serial1.println (integerFromPC3);         
    Serial1.print ("E "); Serial1.println (integerFromPC4);   
    Serial1.print ("F "); Serial1.println (integerFromPC5);    
    Serial1.print ("G "); Serial1.println (integerFromPC6);        
    Serial1.print ("H "); Serial1.println (integerFromPC7);    
    Serial1.print ("I "); Serial1.println (integerFromPC8);         
    Serial1.print ("J "); Serial1.println (integerFromPC9);
 
  Serial1.print ("K ");Serial1.println(PC0);
  Serial1.print ("L ");Serial1.println(PC1);
  Serial1.print ("M ");Serial1.println(PC2);
  Serial1.print ("N ");Serial1.println(PC3);
  Serial1.print ("O ");Serial1.println(PC4); 
  Serial1.print ("P ");Serial1.println(PC5);
  Serial1.print ("Q ");Serial1.println(PC6);
  Serial1.print ("R ");Serial1.println(PC7);
  Serial1.print ("S ");Serial1.println(PC8);
  Serial1.print ("T ");Serial1.println(PC9);  

  Serial1.print ("U ");Serial1.println(positionX);
  Serial1.print ("V ");Serial1.println(positionX1);
  Serial1.print ("W ");Serial1.println(positionX2);
  Serial1.print ("X ");Serial1.println(positionX3);
  Serial1.print ("Y ");Serial1.println(positionX4); 
  Serial1.print ("Z ");Serial1.println(positionX5);
  Serial1.print ("a ");Serial1.println(positionX6);
  Serial1.print ("b ");Serial1.println(positionX7);
  Serial1.print ("c ");Serial1.println(positionX8);
  Serial1.print ("d ");Serial1.println(positionX9);  
    
} 

void noJoeTransformation()
 {
  
     
       stepper[5].moveTo(PC5); 
       stepper[5].run();    
       stepper[4].moveTo(PC4);
       stepper[4].run();
       stepper[3].moveTo(PC3);
       stepper[3].run();       
       stepper[2].moveTo(PC2);
       stepper[2].run();
       stepper[1].moveTo(PC1);
       stepper[1].run();      
       stepper[0].moveTo(PC0); // premiere donnée envoyée dansla chaine de String de Processing <xxx,xxx,xx,xxxx,xx,xxxx>
       stepper[0].run();
} 

void sendPositionNativePort() {  //

void tickSteppers()
{  
      //    Serial1.print ("AftertickSteppers>"); 
      //    Serial1.println (micros());  
}
