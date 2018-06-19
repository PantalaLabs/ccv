/*
  Color Control Voltage 3.0

  This device is used to convert any coloured picture/photo/painting into 3 channels (RGB) 0-5V eurorack synthesizer compatible.
  Creative Commons License
  Color Control Voltage by Pantala Labs is licensed
  under a Creative Commons Attribution 4.0 International License.
  Gibran Curtiss SalomÃ£o. MAY/2017 - CC-BY
*/

#include  <Wire.h>
#include  <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
#define   DAC_RESOLUTION  (8)
#include  <Arduino.h>
#include  <AccelStepper.h>

//#include <Servo.h>
//Servo myservo;

boolean debugMe = true;

//==========================PINS=======================================
//digital pins
#define CLOCKINPIN          2             //interruption pin - optiwheel

#define CLOCKOUTR           9             //clock out red channel
#define CLOCKOUTG           10            //clock out green channel
#define CLOCKOUTB           11            //clock out blue channel
#define CLOCKOUTM           12            //clock out master channel pin

#define S0PIN               31            //S0 color sensor
#define S1PIN               33            //S1 color sensor
#define S3PIN               25            //S3 color sensor
#define S2PIN               27            //S2 color sensor
#define OUTPIN              29            //out color sensor

#define DACRPIN_SEL1        41            //red color DAC array select 
#define DACGPIN_SEL2        43            //green color DAC array select 
#define DACBPIN_SEL3        45            //blue color DAC array select 

#define DIR                 47            //nema direction
#define STEP                49            //nema steps

#define SERVOPIN            37            //servo commander

//analogic pins
#define MOTORSPEEDPIN       A12           //nema speed pot pin
#define ARMSPEEDPIN         A13           //arm speed pot pin
#define CLOCKSOURCEPIN      A14           //2 positions to define clock source (internal / external)
#define SAMPLESPEEDPIN      A15           //sample speed pot pin , if internal clock source

//ocatve quantity for color channels 0 to 5
#define OCTAVEQUANTITYR     A11
#define OCTAVEQUANTITYG     A8
#define OCTAVEQUANTITYB     A5

//ocatve shift for color channels -12 to +12
#define NOTESHIFTR          A10
#define NOTESHIFTG          A7
#define NOTESHIFTB          A4

//clock divider for color channels - many!!!
#define CLOCKDIVIDERR       A9
#define CLOCKDIVIDERG       A6
#define CLOCKDIVIDERB       A3



//=================================================================
//step motor
#define       MINMOTORRPM          230  //slower
#define       MAXMOTORRPM          400  //faster
unsigned int  MOTORSPEEDRead       = 0;
unsigned int  MOTORSPEEDValue      = 0;
AccelStepper  mystepper(1, STEP, DIR);

//colors
unsigned int  red       = 0;
unsigned int  green     = 0;
unsigned int  blue      = 0;
#define       MINCOLOR  15
#define       MAXCOLOR  210

//clock source
unsigned int  CLOCKSOURCEPINRead = 0;     //clock dource analogRead value
boolean       internalClockSource;        //actual clock source int / ext
boolean       lastClockSource;            //last good clock source value

//internal sample speed
#define       MINSAMPLEINTERVAL   50      //min clock interval ms
#define       MAXSAMPLEINTERVAL   4000    //max clock interval ms
unsigned int  SAMPLESPEEDPINRead  = 0;    //sample speed analogRead value
unsigned int  sampleSpeedInterval = 1000; //sample speed

//octave quantity analogRead value
unsigned int  OCTAVEQUANTITYRRead  = 0;
unsigned int  OCTAVEQUANTITYGRead  = 0;
unsigned int  OCTAVEQUANTITYBRead  = 0;

//octave quantity final processed value -5 to 5
int           OCTAVEQUANTITYRValue = 0;
int           OCTAVEQUANTITYGValue = 0;
int           OCTAVEQUANTITYBValue = 0;

//notes shift analogRead value
unsigned int  NOTESHIFTRRead = 0;
unsigned int  NOTESHIFTGRead = 0;
unsigned int  NOTESHIFTBRead = 0;
//notes shift final value -12 to 12
int           NOTESHIFTRValue = 0;
int           NOTESHIFTGValue = 0;
int           NOTESHIFTBValue = 0;

//midi note final calculated value
unsigned int  midiNoteR = -1;
unsigned int  midiNoteG = -1;
unsigned int  midiNoteB = -1;

//servo
#define       SERVOINIT     90            //servo max degree
#define       SERVOEND      48            //servo min degree
#define       MINARMDELAY         500     //min arm move interval ms
#define       MAXARMDELAY         16000   //max arm move interval ms
unsigned int  ARMSPEEDPINRead    = 0;     //arm speed analogRead value
unsigned int  servoPosition = SERVOINIT;  //servo actual position
unsigned int  servoStep    = 1;           //servo step degree e
unsigned int  servoStepDelayValue;        //servo step interval in steps
unsigned long servoNextStep;              //servo step interval
int           armSpeed = 0;               //arm interval analogRead

//tick timers and triggers
#define       TRIGGERWIDTH        5       //trigger width ms
#define       MAXCLOCKCOUNTER     64      //max steps 
volatile boolean startNextStep  = false;  //flag to read new colors each interruption
unsigned long lastTick = 0;               //last timer interruption
unsigned long nextTick = 0;               //next timer interruption
int           clockCounter = -1 ;         //tick counter 0-31
boolean       ledState  = false;
boolean       ledStateR = false;
boolean       ledStateG = false;
boolean       ledStateB = false;

#define       MAXRYTHMCOMB 24

boolean rythmTable[MAXRYTHMCOMB][MAXCLOCKCOUNTER] = {
{ 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,},
{ 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,},
{ 1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0, 1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,},
{ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0, 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,},
{ 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,},
};

//clock divider analogRead value
unsigned int  CLOCKDIVIDERRRead = 0;
unsigned int  CLOCKDIVIDERGRead = 0;
unsigned int  CLOCKDIVIDERBRead = 0;
//clock divider final processed value
unsigned int  CLOCKDIVIDERRValue = 0;
unsigned int  CLOCKDIVIDERGValue = 0;
unsigned int  CLOCKDIVIDERBValue = 0;

#define       DEBOUNCERANGE       15      //dejitter value for analogread
unsigned int  potArrayTime = 0;

void setup()
{
  if (debugMe) {
    Serial.begin(9600);
  }

  pinMode(S0PIN, OUTPUT);
  pinMode(S1PIN, OUTPUT);
  pinMode(S2PIN, OUTPUT);
  pinMode(S3PIN, OUTPUT);
  pinMode(OUTPIN, INPUT);

  digitalWrite(S0PIN, HIGH);
  digitalWrite(S1PIN, LOW);


  pinMode(SERVOPIN, OUTPUT);
  pinMode(CLOCKINPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLOCKINPIN), externalInterrupt, RISING);

  pinMode(CLOCKOUTM, OUTPUT);
  pinMode(CLOCKOUTR, OUTPUT);
  pinMode(CLOCKOUTG, OUTPUT);
  pinMode(CLOCKOUTB, OUTPUT);

  pinMode(DACRPIN_SEL1, OUTPUT);
  pinMode(DACGPIN_SEL2, OUTPUT);
  pinMode(DACBPIN_SEL3, OUTPUT);

  digitalWrite(DACRPIN_SEL1, LOW);
  digitalWrite(DACGPIN_SEL2, LOW);
  digitalWrite(DACBPIN_SEL3, LOW);

  dac.begin(0x62);
  dac.begin(0x63);

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  //  digitalWrite(DIR, HIGH);          // move motor in one direction

  mystepper.setMaxSpeed(MAXMOTORRPM);
  mystepper.setAcceleration(MAXMOTORRPM / 2);
  mystepper.setSpeed(MAXMOTORRPM / 2);

  //myservo.attach(SERVOPIN);
  //myservo.write(servoPosition);
}

void loop() {
  updateNemaSpin();
  readPot();
  updateClockSource();
  closeTriggers();
  checkIfNextStepTime();
  moveNextStep();
  updateServo();
}

void updateNemaSpin() {
  mystepper.runSpeed();
}

void updateClockSource() {
  //if clock source mode changed
  if (lastClockSource != internalClockSource) {
    if (internalClockSource) {
      //becomes internal timer , dettach interruption
      detachInterrupt(digitalPinToInterrupt(CLOCKINPIN));
    } else {
      //becomes external, , attach interruption
      attachInterrupt(digitalPinToInterrupt(CLOCKINPIN), externalInterrupt, RISING);
    }
    lastClockSource = internalClockSource;
    clockCounter = -1;
  }
}

void checkIfNextStepTime() {
  //if internal timer, and time to next step
  if ( (internalClockSource) && (millis() >= nextTick) ) {
    startNextStep = true;
  }
}

void moveNextStep() {
  //move to next step
  if (startNextStep) {
    startNextStep = false;
    clockCounter++;
    if (clockCounter >= MAXCLOCKCOUNTER) {
      clockCounter = 0;
    }
    lastTick = millis();
    if (internalClockSource) {
      nextTick = lastTick + sampleSpeedInterval;
    }
    readColors();   //disable this line if sensor not connected
    updateDacs();
    openTriggers();
  }
}

void updateServo() {
  //time to move servo ?
  return ;

  if ( millis() > servoNextStep ) {
    servoNextStep = millis() + servoStepDelayValue;
    servoPosition = servoPosition - servoStep;        //go to another step
    //invert servo step direction
    if ((servoPosition <= SERVOEND) || (servoPosition >= SERVOINIT)) {
      servoStep = -servoStep;
    }
    //this is a servo hold
    //if servoStepDelay < 90% then step the servo, so dont
    if (servoStepDelayValue < (0.90 * MAXARMDELAY)) {
      //myservo.write(servoPosition); //comanda o servo
    }
  }
}

void readPot() {
  int value;

  //read all inputs , one pot at time
  potArrayTime++;
  if (potArrayTime >= 13) {
    potArrayTime = 0;
  }

  switch (potArrayTime) {
    case 0:
      //servo step interval
      ARMSPEEDPINRead         = softDebounce(analogRead(ARMSPEEDPIN), ARMSPEEDPINRead, DEBOUNCERANGE);
      servoStepDelayValue     = map(ARMSPEEDPINRead, 0, 1023, MINARMDELAY, MAXARMDELAY);
      break;

    case 1:
      //clock source selector : 0=internal / 1=external (manual switchable)
      CLOCKSOURCEPINRead      = softDebounce(analogRead(CLOCKSOURCEPIN), CLOCKSOURCEPINRead, DEBOUNCERANGE);
      internalClockSource     = CLOCKSOURCEPINRead < 500;
      break;

    case 2:
      //if internal timer , calculate the sample speed interval
      if (internalClockSource) {
        SAMPLESPEEDPINRead    = softDebounce(analogRead(SAMPLESPEEDPIN), SAMPLESPEEDPINRead, DEBOUNCERANGE);
        sampleSpeedInterval   = map(SAMPLESPEEDPINRead, 0, 1023, MAXSAMPLEINTERVAL, MINSAMPLEINTERVAL);
      }
      break;

    case 3:
      //read motor speed pot and updates nema
      MOTORSPEEDRead = softDebounce(analogRead(MOTORSPEEDPIN), MOTORSPEEDRead, 2 * DEBOUNCERANGE);
      value = map(MOTORSPEEDRead, 0, 1023, MINMOTORRPM, MAXMOTORRPM);
      if (value != MOTORSPEEDValue) {
        mystepper.setSpeed(value);
        MOTORSPEEDValue = value;
      }
      break;

    case 4:
      CLOCKDIVIDERRRead = softDebounce(analogRead(CLOCKDIVIDERR), CLOCKDIVIDERRRead, DEBOUNCERANGE);
      CLOCKDIVIDERRValue = map(CLOCKDIVIDERRRead, 0, 1023, 0, MAXRYTHMCOMB);
      break;

    case 5:
      CLOCKDIVIDERGRead = softDebounce(analogRead(CLOCKDIVIDERG), CLOCKDIVIDERGRead, DEBOUNCERANGE);
      CLOCKDIVIDERGValue = map(CLOCKDIVIDERGRead, 0, 1023, 0, MAXRYTHMCOMB);
      break;

    case 6:
      CLOCKDIVIDERBRead = softDebounce(analogRead(CLOCKDIVIDERB), CLOCKDIVIDERBRead, DEBOUNCERANGE);
      CLOCKDIVIDERBValue = map(CLOCKDIVIDERBRead, 0, 1023, 0, MAXRYTHMCOMB);
      break;

    //octaves quantity -5 to 5
    case 7:
      OCTAVEQUANTITYRRead = softDebounce(analogRead(OCTAVEQUANTITYR), OCTAVEQUANTITYRRead, DEBOUNCERANGE);
      value = map(OCTAVEQUANTITYRRead, 0, 1023, -5, 6);
      if (value == 0) {
        value = 1;
      }
      value = min(value, 5);
      if (value != OCTAVEQUANTITYRValue) {
        OCTAVEQUANTITYRValue = value;
      }
      break;

    case 8:
      OCTAVEQUANTITYGRead = softDebounce(analogRead(OCTAVEQUANTITYG), OCTAVEQUANTITYGRead, DEBOUNCERANGE);
      value = map(OCTAVEQUANTITYGRead, 0, 1023, -5, 6);
      if (value == 0) {
        value = 1;
      }
      value = min(value, 5);
      if (value != OCTAVEQUANTITYGValue) {
        OCTAVEQUANTITYGValue = value;
      }
      break;

    case 9:
      OCTAVEQUANTITYBRead = softDebounce(analogRead(OCTAVEQUANTITYB), OCTAVEQUANTITYBRead, DEBOUNCERANGE);
      value = map(OCTAVEQUANTITYBRead, 0, 1023, -5, 6);
      if (value == 0) {
        value = 1;
      }
      value = min(value, 5);
      if (value != OCTAVEQUANTITYBValue) {
        OCTAVEQUANTITYBValue = value;
      }
      break;

    case 10:
      NOTESHIFTRRead = softDebounce(analogRead(NOTESHIFTR), NOTESHIFTRRead, DEBOUNCERANGE);
      value = map(NOTESHIFTRRead, 0, 1023, -12, 13);
      NOTESHIFTRValue = constrain(value, -12, 12);
      break;

    case 11:
      NOTESHIFTGRead = softDebounce(analogRead(NOTESHIFTG), NOTESHIFTGRead, DEBOUNCERANGE);
      value = map(NOTESHIFTGRead, 0, 1023, -12, 13);
      NOTESHIFTGValue = constrain(value, -12, 12);
      break;

    case 12:
      NOTESHIFTBRead = softDebounce(analogRead(NOTESHIFTB), NOTESHIFTBRead, DEBOUNCERANGE);
      value = map(NOTESHIFTBRead, 0, 1023, -12, 13);
      NOTESHIFTBValue = constrain(value, -12, 12);
      break;
  }
}

void openTriggers() {
  ledState = true;
  digitalWrite(CLOCKOUTM, HIGH);
  if (rythmTable[CLOCKDIVIDERRValue][clockCounter]) {
    digitalWrite(CLOCKOUTR, HIGH);
    ledStateR = true;
  }
  if (rythmTable[CLOCKDIVIDERGValue][clockCounter]) {
    digitalWrite(CLOCKOUTG, HIGH);
    ledStateG = true;
  }
  if (rythmTable[CLOCKDIVIDERBValue][clockCounter]) {
    digitalWrite(CLOCKOUTB, HIGH);
    ledStateB = true;
  }
}

void closeTriggers() {
  //if its time to close and master trigger is still open
  if ( (millis() >= lastTick + TRIGGERWIDTH) && (ledState) ) {
    digitalWrite(CLOCKOUTM, LOW);
    ledState = false;
    if (ledStateR) {
      digitalWrite(CLOCKOUTR, LOW);
      ledStateR = false;
    }

    if (ledStateG) {
      digitalWrite(CLOCKOUTG, LOW);
      ledStateG = false;
    }

    if (ledStateB) {
      digitalWrite(CLOCKOUTB, LOW);
      ledStateB = false;
    }
  }
}


void updateDacs() {
  int  midiNote;
  int  voltageEquivalent;

  //red dac
  if (rythmTable[CLOCKDIVIDERRValue][clockCounter]) {
    //stretch to (5 octaves) * 4 range
    midiNote = map(red, MINCOLOR, MAXCOLOR, 1, 240);
    //red scale invert
    if (OCTAVEQUANTITYRValue > 0) {
      red =   240 - red;
    }
    //shrink to user selected octaves quantity
    midiNote = map(red, 1, 240, 1, 12 * abs(OCTAVEQUANTITYRValue));
    //shift user selected notes/octaves quantity
    midiNote = midiNote + NOTESHIFTRValue;
    //assure 1 to 60 range midi note
    midiNoteR = constrain(midiNote, 1, 60);
    voltageEquivalent = map(midiNoteR, 0, 60, 0, 4095);
    digitalWrite(DACRPIN_SEL1, HIGH);
    dac.setVoltage(voltageEquivalent, false);
    digitalWrite(DACRPIN_SEL1, LOW);
  }

  //green dac
  if (rythmTable[CLOCKDIVIDERGValue][clockCounter]) {
    //stretch to (5 octaves) * 4 range
    midiNote = map(green, MINCOLOR, MAXCOLOR, 1, 240);
    //green scale invert
    if (OCTAVEQUANTITYGValue > 0) {
      green =   240 - green;
    }
    //shrink to user selected octaves quantity
    midiNote = map(green, 1, 240, 1, 12 * abs(OCTAVEQUANTITYGValue));
    //shift user selected notes/octaves quantity
    midiNote = midiNote + NOTESHIFTGValue;
    //assure 1 to 60 range midi note
    midiNoteG = constrain(midiNote, 1, 60);
    voltageEquivalent = map(midiNoteG, 0, 60, 0, 4095);
    digitalWrite(DACGPIN_SEL2, HIGH);
    dac.setVoltage(voltageEquivalent, false);
    digitalWrite(DACGPIN_SEL2, LOW);
  }

  //blue dac
  if (rythmTable[CLOCKDIVIDERBValue][clockCounter]) {
    //stretch to (5 octaves) * 4 range
    midiNote = map(blue, MINCOLOR, MAXCOLOR, 1, 240);
    //blue scale invert
    if (OCTAVEQUANTITYBValue > 0) {
      blue =   240 - blue;
    }
    //shrink to user selected octaves quantity
    midiNote = map(blue, 1, 240, 1, 12 * abs(OCTAVEQUANTITYBValue));
    //shift user selected notes/octaves quantity
    midiNote = midiNote + NOTESHIFTBValue;
    //assure 1 to 60 range midi note
    midiNoteB = constrain(midiNote, 1, 60);
    voltageEquivalent = map(midiNoteB, 0, 60, 0, 4095);
    digitalWrite(DACBPIN_SEL3, HIGH);
    dac.setVoltage(voltageEquivalent, false);
    digitalWrite(DACBPIN_SEL3, LOW);
  }
}

void readColors() {
  //read sensor colors
  digitalWrite(S2PIN, LOW);
  digitalWrite(S3PIN, LOW);
  red   = pulseIn(OUTPIN, digitalRead(OUTPIN) == HIGH ? LOW : HIGH);
  digitalWrite(S3PIN, HIGH);
  blue  = pulseIn(OUTPIN, digitalRead(OUTPIN) == HIGH ? LOW : HIGH);
  digitalWrite(S2PIN, HIGH);
  green = pulseIn(OUTPIN, digitalRead(OUTPIN) == HIGH ? LOW : HIGH);
  red     = constrain(red,    MINCOLOR, MAXCOLOR);
  green   = constrain(green,  MINCOLOR, MAXCOLOR);
  blue    = constrain(blue,   MINCOLOR, MAXCOLOR);
}

int  softDebounce(int readCV, int oldRead, int range) {
  if (abs(readCV - oldRead) > range) {
    return readCV;
  }
  return oldRead;
}

void externalInterrupt() {
  startNextStep = true;
}

void printAllControls() {
  Serial.print(OCTAVEQUANTITYRValue);
  Serial.print(", ");
  Serial.print(OCTAVEQUANTITYGValue);
  Serial.print(", ");
  Serial.print(OCTAVEQUANTITYBValue);
  Serial.println(".");

  Serial.print(NOTESHIFTRValue);
  Serial.print(", ");
  Serial.print(NOTESHIFTGValue);
  Serial.print(", ");
  Serial.print(NOTESHIFTBValue);
  Serial.println(".");

  Serial.print(CLOCKDIVIDERRValue);
  Serial.print(", ");
  Serial.print(CLOCKDIVIDERGValue);
  Serial.print(", ");
  Serial.print(CLOCKDIVIDERBValue);
  Serial.println(".");

  Serial.print(servoStepDelayValue);
  Serial.print(", ");
  Serial.print(internalClockSource);
  Serial.print(", ");
  Serial.print(sampleSpeedInterval);
  Serial.println(".");

  Serial.print(midiNoteR);
  Serial.print(", ");
  Serial.print(midiNoteG);
  Serial.print(", ");
  Serial.print(midiNoteB);
  Serial.println(".");

  Serial.println("---------------------------");

}


void printMidiNotes() {
  Serial.print(midiNoteR);
  Serial.print(" , ");
  Serial.print(midiNoteG);
  Serial.print(" , ");
  Serial.print(midiNoteB);
  Serial.println(" ;");
}

void printColors() {
  Serial.print("red:");
  Serial.print(red);
  Serial.print(" ,green:");
  Serial.print(green);
  Serial.print(" ,blue:");
  Serial.print(blue);
  Serial.println(" ;");
}

