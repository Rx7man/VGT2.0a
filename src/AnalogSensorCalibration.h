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

AnalogSensor GM1Bar(Gm1BarMapSensorLowRawValue, Gm1BarMapSensorHighRawValue, Gm1BarMapSensorLowValuePressure, Gm1BarMapSensorHighValuePressure, false);
AnalogSensor GM2Bar(Gm2BarMapSensorLowRawValue, Gm2BarMapSensorHighRawValue, Gm2BarMapSensorLowValuePressure, Gm2BarMapSensorHighValuePressure, false);
AnalogSensor Honeywell100PSI(Honeywell100PsiLowRawValue, Honeywell100PsiHighRawValue, Honeywell100PsiLowValuePressure, Honeywell100PsiHighValuePressure, false);
AnalogSensor Honeywell1000PSI(Honeywell1000PsiLowRawValue, Honeywell1000PsiHighRawValue, Honeywell1000PsiLowValuePressure, Honeywell1000PsiHighValuePressure, false);

#endif // !AnalogSensorCalibration_h

