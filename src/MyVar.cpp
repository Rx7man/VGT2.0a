#include "Arduino.h"
#include "MyVar.h"



MyVar::MyVar() {
   //Just set up some defaults that ought to do for most things here 
  Smoothing = .9;
  SlopeSmoothing = .9;
  Value = 0.0f;
}

void MyVar::UpdateValue(float NewValue) {
    float NewSmoothedValue = GetSmoothedValue(NewValue, Value, Smoothing);
     
    CalculateSlope(UseSmoothValueForSlope?NewSmoothedValue:NewValue, Value);   
	 Value = NewSmoothedValue;
}


 void MyVar::CalculateSlope(float NewValue, float OldValue) {
  unsigned long Now = millis();
   float TimeDifference = Now - LastUpdateTime; //And time
    float NewSlope = (NewValue - OldValue) / TimeDifference * 1000; //Calculate the new slope, multiply by 1000 to convert from milliseconds to seconds

  float SlopeDifference = NewSlope - Slope; //repeat for the DeltaSlope
  Slope = GetSmoothedValue(NewSlope, Slope, SlopeSmoothing); //And smooth it out 
  LastUpdateTime = Now;
 
}

 String MyVar::ToString(int Decimalplaces = 3) {
     return Name + " = " + String(Value,Decimalplaces) + "\n" + Name + "s = " + String(Slope, Decimalplaces);
 }
void MyVar::Reset() {
  Slope = 0;
  Value = 0;
  LastUpdateTime = millis();
}

 float MyVar::GetSmoothedValue(float Newvalue, float Oldvalue, float smoothing) //0 = no smoothing, 1 = infinite smoothing
{
  //provides a decaying infinite average.. Smoothing goes from 0 to 1, where 0 will yield an unchanging output! (you have been warned)
  //depending on read frequency and desired aggressiveness  The benefit to this method
  //is it doesn't require an array to hold the history, and reduces the math required
    //smoothing = constrain(smoothing, 0, 1); //funky stuff can happen if we're not within bounds.. like sign flipping, etc
    float returnval = (Newvalue * (1 - smoothing) + (Oldvalue * smoothing));
    return returnval;
 
} //GetSmoothedValue


 /*
 float MyVar::mapf(float value, float oldlow, float oldhigh, float newlow, float newhigh)
{
  //I think this is identical to the arduino map, but using floats rather than ints, and preventing a divide by 0 error
  float rval = 0;
  if ((oldhigh - oldlow) != 0) {
    rval = float(value - oldlow) / float(oldhigh - oldlow) * float(newhigh - newlow) + newlow;
  }//if
  else {
    rval = 0;
  }//else
  return rval;
} //mapf
*/