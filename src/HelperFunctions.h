#pragma once
#ifndef HelperFunctions_h
#define HelperFunctions_h
inline float mapf(float value, float oldlow, float oldhigh, float newlow, float newhigh, boolean Constrain) {
	//TESTED
	if (oldhigh == oldlow) {
		Serial.print("Mapf function got a divide by zero, OldLow and OldHigh = ");
		Serial.print(oldlow);
		return 0;
	}
	if (Constrain) {
		return constrain(float(value - oldlow) / float(oldhigh - oldlow) *
								float(newhigh - newlow) + newlow, newlow, newhigh);
	}
	else {
		return float(value - oldlow) / float(oldhigh - oldlow) * float(newhigh - newlow) + newlow;
	}
} //mapf

inline float GetGenericCompensationLin(float value, float NegativeFactor, float PositiveFactor, boolean Constrain = true) //value should be -1 to 1 for the input
{
	//Linear mode, TESTED
	if (value < 0) {
		return mapf(value, 0, -1, 0, NegativeFactor, Constrain);
	}
	else {
		return mapf(value, 0, 1, 0, PositiveFactor, Constrain);
	}

}
inline float GetGenericCompensationExp(float value, float NegativeFactor, float PositiveFactor, boolean Constrain = true) //value should be -1 to 1 for the input
{ //Exponential mode, TESTED
	if (Constrain) { value = constrain(value, -1, 1); }

	if (value < 0) {
		return -pow(NegativeFactor, -value);
	}
	else {
		return pow(PositiveFactor, value);
	}
}

inline float GetAdcVoltage(int RawADC) {
	return RawADC * 0.004887f;
}

inline void Serialprint(char *Name, double Value, int Precision) {
	Serial.print(Name);
	Serial.print(" = ");
	Serial.println(Value, Precision);
}
inline void Serialprintint(char *Name, long Value) {
	Serial.print(Name);
	Serial.print(" = ");
	Serial.println(Value);
}


#endif // !HelperFunctions_h