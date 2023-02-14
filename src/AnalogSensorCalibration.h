#pragma once
#ifndef AnalogSensorCalibration_h
#define AnalogSensorCalibration_h
#define Honeywell100PsiLowRawValue  100  //100 = 101 kpa
#define Honeywell100PsiLowValuePressure  0
#define Honeywell100PsiHighRawValue 921
#define Honeywell100PsiHighValuePressure  100

#define Honeywell1000PsiLowRawValue  80
#define Honeywell1000PsiLowValuePressure  0
#define Honeywell1000PsiHighRawValue  921
#define Honeywell1000PsiHighValuePressure  1000

//2 bar GM map sensor = 615 raw value at sea level
#define Gm2BarMapSensorLowRawValue 106
#define Gm2BarMapSensorLowValuePressure 1.87
#define Gm2BarMapSensorHighRawValue 469
#define Gm2BarMapSensorHighValuePressure 13.7

#define Gm1BarMapSensorLowRawValue 187
#define Gm1BarMapSensorLowValuePressure 1.87
#define Gm1BarMapSensorHighRawValue 939
#define Gm1BarMapSensorHighValuePressure 13.7


#endif // !AnalogSensorCalibration_h

