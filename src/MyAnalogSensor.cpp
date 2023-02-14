#include "MyAnalogSensor.h"
#include "Arduino.h"

AnalogSensor::AnalogSensor(float LowRawReading, float HighRawReading, float LowValue, float HighValue, bool Constrain)
{
	_LowRawReading = LowRawReading;
	_HighRawReading = HighRawReading;
	_LowValue = LowValue;
	_HighValue = HighValue;
	_Constrain = Constrain;
}

float AnalogSensor::GetValue(float RawValue) {
	float rval = 0;
	if ((_HighRawReading - _LowRawReading) != 0) {
		rval = float(RawValue - _LowRawReading) / float(_HighRawReading - _LowRawReading) * float(_HighValue - _LowValue) + _LowValue;
	}//if
	else {
		rval = 0;
	}//else
	if (_Constrain) {
		rval = constrain(rval, _LowValue, _HighValue);
	}
	return rval;
	
}
float AnalogSensor::ReadFromPin(int Pin) {
	int RawValue = analogRead(Pin);
	return GetValue(RawValue);
}




AnalogSensor::~AnalogSensor()
{
}
