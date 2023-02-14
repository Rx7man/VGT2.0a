#pragma once
#ifndef MyAnalogSensor_h
#define MyAnalogSensor_h





class AnalogSensor
{
public:
	
	AnalogSensor(float, float, float, float, bool);
	
	float GetValue(float RawValue);
	float ReadFromPin(int PinNumber);

	
	float _LowRawReading;
	float _HighRawReading;
	float _LowValue;
	float _HighValue;
    bool _Constrain;



~AnalogSensor();
};


#endif

