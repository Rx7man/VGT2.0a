
#ifndef MyVar_h
#define MyVar_h

#include "Arduino.h"


class MyVar {
  public:
    MyVar();
    
    String Name;
	float Smoothing; //dejitters the value, 0 for inactive, 1 for infinite smoothing
    float SlopeSmoothing;
    boolean UseSmoothValueForSlope = false;
    float Slope;  //Used for PID-like functions, rate of change of variable, in units per second (Acceleration)
    float Value; //use this to access the storage value, without the curve applied

    void Reset(); //use this to reset the storage and slope
    void UpdateValue(float NewValue); //use this to update the value, applying the min/max change, min/max value, and smoothing all at once
    String ToString(int DecimalPlaces = 3);
  private:
    unsigned long LastUpdateTime;
    void CalculateSlope(float,float);
    static float GetSmoothedValue(float Newvalue, float Oldvalue, float smoothing); //Does the work for UpdateValue
   // static float mapf(float value, float oldlow, float oldhigh, float newlow, float newhigh);
   
};

#endif
