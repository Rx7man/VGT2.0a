#pragma once

#ifndef PinDefs_h
#define PinDefs_h


//digital inputs
#define TssPin  2
#define RpmPin 21

#define SCK    52
#define SDI    51
#define SDO    50

#define Switch2HighPin 24 //right switch up active low
#define Switch2LowPin 22  //right switch down active low
#define Switch1HighPin 25 //left switch up   active low
#define Switch1LowPin 23  //left switch down active low
#define JakeSwitchPin 26  //Jake switch active low
#define ThumbSwitchPin 27 //Shifter knob thumbswitch active low



//digital outputs
#define GreenLedPin 40
#define AmberLedPin 41
#define LineLockPin 28 
#define CruiseVentPin 30
#define CruiseVacPin 32
#define CruiseLedPin 38
#define BOVsolenoidPin CruiseLedPin  //lets just use the cruise LED pin for this for now until I have a BOV

//SPI channels
#define CanControllerPin 10
#define EgtThermoPin 49
#define CotThermoPin 48
#define IatThermoPin 47
#define CtsThermoPin 46


//Analog inputs
#define EgpPin  A0
#define TpsPin  A1
#define MapPin  A2
#define BrakePin  A3
#define Aux2Pin  A4 //console potentiometer right side 
#define Aux1Pin  A5 //console potentiometer left side

#define BaroPin A8
#define CIPpin A9
#define CruiseInputPin A13
#define FpsPin A15
#define OpsPin A14
#endif // !PinDefs_h

