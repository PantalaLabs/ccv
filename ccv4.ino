bool debug = true;
/*
  Color Control Voltage 3.0

  This device converts any coloured picture/photo/painting into 3 channels (RGB) 0-5V eurorack synthesizer compatible.
  Creative Commons License
  Color Control Voltage by Pantala Labs is licensed
  under a Creative Commons Attribution 4.0 International License.
  Gibran Curtiss SalomÃ£o. MAY/2017 - CC-BY
*/

#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
#define DAC_RESOLUTION (8)
#include <Arduino.h>
#include <AccelStepper.h>
#include <AnalogInput.h>
#include <Trigger.h>
#include <Counter.h>
#include <EventDebounce.h>
#include <Switch.h>

#define MAXPATTERNS 19
#define MAXstepCounter 16

#include "patterns.h"

extern boolean patterns[MAXPATTERNS][MAXstepCounter];

//==========================PINS=======================================
//digital pins
#define CLOCKINPIN 2 //interruption pin - optiwheel
EventDebounce triggerInterval(50);

//TRIGGER out
Trigger gateR(43);   //clock out red channel
Trigger gateG(45);   //clock out green channel
Trigger gateB(47);   //clock out blue channel
Trigger gateM(49);   //clock out master channel pin
Trigger gateSlv(51); //slave clock divider

//COLOR sensor
#define S0PIN 31       //S0 color sensor
#define S1PIN 33       //S1 color sensor
#define S3PIN 29       //S3 color sensor
#define S2PIN 27       //S2 color sensor
#define OUTCOLORPIN 25 //out color sensor

//DAC select
#define DACRPIN_SEL1 14 //red color DAC array select
#define DACGPIN_SEL2 15 //green color DAC array select
#define DACBPIN_SEL3 16 //blue color DAC array select

//NEMA
#define DIR 35  //nema direction
#define STEP 37 //nema steps

//CLOCK source
Switch swIntClockSrc(53);

//analog pins
AnalogInput SAMPLESPEED(12); //sample speed pot pin , if internal clock source
AnalogInput MAINCLKDIV(14);  //arm speed pot pin++++++++++++++++++++++deactivated
AnalogInput MOTORSPEED(15);  //nema speed pot pin

//octave quantity for color channels -5 to 0 to 5
AnalogInput OCTAVEQUANTITYR(3);
AnalogInput OCTAVEQUANTITYG(4);
AnalogInput OCTAVEQUANTITYB(5);

//ocatve shift for color channels -12 to +12
AnalogInput NOTESHIFTR(6);
AnalogInput NOTESHIFTG(7);
AnalogInput NOTESHIFTB(8);

//clock divider for color channels
AnalogInput CLOCKDIVIDERR(9);
AnalogInput CLOCKDIVIDERG(10);
AnalogInput CLOCKDIVIDERB(11);

//=================================================================
//step motor
#define MINMOTORRPM 230 //slower
#define MAXMOTORRPM 400 //faster
uint16_t MOTORSPEEDValue = 0;
AccelStepper mystepper(1, STEP, DIR);

//colors
uint16_t red = 0;
uint16_t green = 0;
uint16_t blue = 0;
#define MINCOLOR 15
#define MAXCOLOR 210

//internal sample speed
#define MINSAMPLEINTERVAL 50         //min clock interval ms
#define MAXSAMPLEINTERVAL 4000       //max clock interval ms
uint16_t sampleSpeedInterval = 1000; //sample speed

//octave quantity final processed value -5 to 5
int16_t OCTAVEQUANTITYRValue = 0;
int16_t OCTAVEQUANTITYGValue = 0;
int16_t OCTAVEQUANTITYBValue = 0;

//notes shift final value -12 to 12
int16_t NOTESHIFTRValue = 0;
int16_t NOTESHIFTGValue = 0;
int16_t NOTESHIFTBValue = 0;
int16_t MAINCLKDIVValue = 0;

//midi note final calculated value
uint16_t midiNoteR = -1;
uint16_t midiNoteG = -1;
uint16_t midiNoteB = -1;

//tick timers and triggers
Counter stepCounterM(15); //tick counter
Counter stepCounterR(15); //tick counter
Counter stepCounterG(15); //tick counter
Counter stepCounterB(15); //tick counter

boolean startNextStep = false; //flag to read new colors each interruption
unsigned long lastTick = 0;    //last timer interruption
unsigned long nextTick = 0;    //next timer interruption

Counter potPriority(11); //tick counter

//clock divider final processed value
uint16_t CLOCKDIVIDERRValue = 0;
uint16_t CLOCKDIVIDERGValue = 0;
uint16_t CLOCKDIVIDERBValue = 0;

void setup()
{
  if (debug)
    Serial.begin(9600);

  //color sensor
  pinMode(S0PIN, OUTPUT);
  pinMode(S1PIN, OUTPUT);
  pinMode(S2PIN, OUTPUT);
  pinMode(S3PIN, OUTPUT);
  pinMode(OUTCOLORPIN, INPUT);
  digitalWrite(S0PIN, HIGH);
  digitalWrite(S1PIN, LOW);

  pinMode(CLOCKINPIN, INPUT);

  pinMode(DACRPIN_SEL1, OUTPUT);
  pinMode(DACGPIN_SEL2, OUTPUT);
  pinMode(DACBPIN_SEL3, OUTPUT);

  digitalWrite(DACRPIN_SEL1, LOW);
  digitalWrite(DACGPIN_SEL2, LOW);
  digitalWrite(DACBPIN_SEL3, LOW);

  dac.begin(0x63);

  //NEMA
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  //  digitalWrite(DIR, HIGH);          // move motor in one direction
  mystepper.setMaxSpeed(MAXMOTORRPM);
  mystepper.setAcceleration(MAXMOTORRPM / 2);
  mystepper.setSpeed(MAXMOTORRPM / 2);
}

void loop()
{
  swIntClockSrc.readPin();
  if (swIntClockSrc.switchOn())
    stepCounterM.reset();
  mystepper.runSpeed();
  readPot();
  closeTriggers();
  checkIfNextStepTime();
  //move to next step
  if (startNextStep)
  {
    startNextStep = false;
    stepCounterM.advance();
    stepCounterR.advance();
    stepCounterG.advance();
    stepCounterB.advance();

    lastTick = millis();
    if (swIntClockSrc.active())
      nextTick = lastTick + sampleSpeedInterval;
    readColors();
    updateDacs();
    openTriggers();
  }
}

void checkIfNextStepTime()
{
  //if internal timer, and time to next step
  if (swIntClockSrc.active() && (millis() >= nextTick))
    startNextStep = true;
  else if (!swIntClockSrc.active() && digitalRead(CLOCKINPIN) && triggerInterval.debounced())
  {
    triggerInterval.debounce();
    startNextStep = true;
  }
}

void readPot()
{
  int16_t value;

  //if internal timer , calculate the sample speed interval
  if (swIntClockSrc.active())
    sampleSpeedInterval = map(SAMPLESPEED.readPin(), 0, 1023, MAXSAMPLEINTERVAL, MINSAMPLEINTERVAL);

  //read inputs , one pot at time
  switch (potPriority.advance())
  {
  case 0:
    //read motor speed pot and updates nema
    value = map(MOTORSPEED.readPin(), 0, 1023, MINMOTORRPM, MAXMOTORRPM);
    if (value != MOTORSPEEDValue)
    {
      mystepper.setSpeed(value);
      MOTORSPEEDValue = value;
    }
    break;
  case 1:
    value = map(CLOCKDIVIDERR.readPin(), 0, 1023, 0, MAXPATTERNS);
    if (value != CLOCKDIVIDERRValue)
    {
      stepCounterR.reset();
      CLOCKDIVIDERRValue = value;
    }
    break;
  case 2:
    value = map(CLOCKDIVIDERG.readPin(), 0, 1023, 0, MAXPATTERNS);
    if (value != CLOCKDIVIDERGValue)
    {
      stepCounterG.reset();
      CLOCKDIVIDERGValue = value;
    }
    break;
  case 3:
    value = map(CLOCKDIVIDERB.readPin(), 0, 1023, 0, MAXPATTERNS);
    if (value != CLOCKDIVIDERBValue)
    {
      stepCounterB.reset();
      CLOCKDIVIDERBValue = value;
    }
    break;
  //octaves quantity -5 to 5
  case 4:
    OCTAVEQUANTITYRValue = constrain(map(OCTAVEQUANTITYR.readPin(), 0, 1023, -5, 6), -5, 5);
    // if (OCTAVEQUANTITYRValue == 0)
    //   value = 1;
    break;
  case 6:
    OCTAVEQUANTITYGValue = constrain(map(OCTAVEQUANTITYG.readPin(), 0, 1023, -5, 6), -5, 5);
    // if (OCTAVEQUANTITYGValue == 0)
    //   OCTAVEQUANTITYGValue = 1;
    break;
  case 7:
    OCTAVEQUANTITYBValue = constrain(map(OCTAVEQUANTITYB.readPin(), 0, 1023, -5, 6), -5, 5);
    // if (OCTAVEQUANTITYBValue == 0)
    //   OCTAVEQUANTITYBValue = 1;
    break;
  case 8:
    NOTESHIFTRValue = constrain(map(NOTESHIFTR.readPin(), 0, 1023, -12, 13), -12, 12);
    break;
  case 9:
    NOTESHIFTGValue = constrain(map(NOTESHIFTG.readPin(), 0, 1023, -12, 13), -12, 12);
    break;
  case 10:
    NOTESHIFTBValue = constrain(map(NOTESHIFTB.readPin(), 0, 1023, -12, 13), -12, 12);
    break;
  case 11:
    MAINCLKDIVValue = map(MAINCLKDIV.readPin(), 0, 1023, 0, MAXPATTERNS);
    break;
  }
}

void openTriggers()
{
  gateM.start();
  if (patterns[CLOCKDIVIDERRValue][stepCounterR.getValue()])
    gateR.start();
  if (patterns[CLOCKDIVIDERGValue][stepCounterG.getValue()])
    gateG.start();
  if (patterns[CLOCKDIVIDERBValue][stepCounterB.getValue()])
    gateB.start();
  if (patterns[MAINCLKDIVValue][stepCounterM.getValue()])
    gateSlv.start();
}

void closeTriggers()
{
  gateM.compute();
  gateR.compute();
  gateG.compute();
  gateB.compute();
  gateSlv.compute();
}

void updateDacs()
{
  int16_t midiNote;
  int16_t voltageEquivalent;

  //red dac
  if (patterns[CLOCKDIVIDERRValue][stepCounterR.getValue()])
  {
    //stretch to (5 octaves) * 4 range
    midiNote = map(red, MINCOLOR, MAXCOLOR, 1, 240);
    //red scale invert
    if (OCTAVEQUANTITYRValue > 0)
      red = 240 - red;
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
  if (patterns[CLOCKDIVIDERGValue][stepCounterG.getValue()])
  {
    //stretch to (5 octaves) * 4 range
    midiNote = map(green, MINCOLOR, MAXCOLOR, 1, 240);
    //green scale invert
    if (OCTAVEQUANTITYGValue > 0)
      green = 240 - green;
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
  if (patterns[CLOCKDIVIDERBValue][stepCounterB.getValue()])
  {
    //stretch to (5 octaves) * 4 range
    midiNote = map(blue, MINCOLOR, MAXCOLOR, 1, 240);
    //blue scale invert
    if (OCTAVEQUANTITYBValue > 0)
      blue = 240 - blue;
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

void readColors()
{
  //read sensor colors
  //disable this if sensor not connected
  digitalWrite(S2PIN, LOW);
  digitalWrite(S3PIN, LOW);
  red = pulseIn(OUTCOLORPIN, digitalRead(OUTCOLORPIN) == HIGH ? LOW : HIGH);
  digitalWrite(S3PIN, HIGH);
  blue = pulseIn(OUTCOLORPIN, digitalRead(OUTCOLORPIN) == HIGH ? LOW : HIGH);
  digitalWrite(S2PIN, HIGH);
  green = pulseIn(OUTCOLORPIN, digitalRead(OUTCOLORPIN) == HIGH ? LOW : HIGH);
  red = constrain(red, MINCOLOR, MAXCOLOR);
  green = constrain(green, MINCOLOR, MAXCOLOR);
  blue = constrain(blue, MINCOLOR, MAXCOLOR);
  printColors();
}

void printAllControls()
{
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

  Serial.print(swIntClockSrc.active());
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

void printMidiNotes()
{
  Serial.print(midiNoteR);
  Serial.print(" , ");
  Serial.print(midiNoteG);
  Serial.print(" , ");
  Serial.print(midiNoteB);
  Serial.println(" ;");
}

void printColors()
{
  Serial.print("red:");
  Serial.print(red);
  Serial.print(" ,green:");
  Serial.print(green);
  Serial.print(" ,blue:");
  Serial.print(blue);
  Serial.println(" ;");
}
