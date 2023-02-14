#pragma once
#ifndef TuningParameters_h
#define TuningParameters_h

int TurbineSpeedDivisions[] = { 0, 20, 40, 60, 80, 100, 120, 140 };
int TpsRowLabels[] = { 0, 5, 20, 40, 60, 100 };
int RPMdivisions[] = { 0, 500, 750, 1000, 1250, 1500, 2000,  2500, 3000, 5000 };
int PressureDivisions[] = { 0, 5, 10, 15, 20, 25, 30, 40, 60, 90 };


int LilbbMapData_Up[] = //Tow
// 0   20   40   60   80   100  120  140 
{ 95,  95,  90,  80,  75,  50,  30,  0 };

int LilbbMapData_Middle[] = //Normal
// 0   20   40   60   80   100  120  140 
{ 90,  90,  85,  80,  70,  52,  30,  0 };

int LilbbMapData_Down[] = //Low Boost
// 0   20   40   60   80   100  120  140 
{ 90,  90,  80,  70,  60,  50,  20,  0 };


//                    RPM      500    1000    1500    2500    5000  Uses RpmColumnLabels as breakpoints
int AutobrakeMapData[] = { 90, 90, 90, 90, 96, 95, 94, 93, 90, 80 };
int BoostRPMMapData[] = { 25,  5,  5,  5,  10, 29, 25, 35, 40, 40 };


//                    RPM               500    1000    1500    2500    5000  Uses RpmColumnLabels as breakpoints
int RPMminimumPositionMapData[] = { 60, 96, 96, 91, 89, 84, 80, 75, 70, 50 };

//                                  0   20  40  60  80  100 120 140  
int TSSminimumPositionMapData[] = { 96, 96, 96, 90, 85, 70, 30, 10 };

//								      0    5   10     20      30  40  60  90
int BoostMinimumPositionMapData[] = { 96, 93, 90, 85, 80, 75, 70, 50, 30, 10 };

//								    0    5   10      20      30  40  60  90
int EGPminimumPositionMapData[] = { 96, 95,  94, 90, 85, 80, 70, 60, 30, 10 };




const int VGTstartPosition = 0; //VGT position when starting or not running

const int MinRunRPM = 400; //minimum RPM to consider the engine running, should be just a bit faster than the fastest crank speed
const int MaxIdleRPM = 800; //maximum RPM to consider it idle
const int MaxIdleTPS = 3;  //maximum TPS to consider it idle, if you have very steady TPS sensor you can set very low, it's not critical though


const float MaxTotalCompensation = 3;
const float MinTotalCompensation = .5;

const float JakeBoostFactor = .8;            //partially closes turbo to build more boost when TPS > JakeBoostTPS
const int   JakeBrakeTPSthreshold = 5;

const int AuxCompTSSLowPoint = 40; //TSS below this will be soley influenced by Aux1 pot, 
const int AuxCompTSSHighPoint = 80; //TSS above this will be soley influenced by Aux2 pot, if TSS is between, it is interpolated

const int AntisurgeMinimumTPSslope = 20; //Set higher to make less sensitive to accidental triggers
const float AntisurgeResponseRate = .1; //higher number = more response to 
const float AntisurgeFalloffRate = .97; //higher number = faster falloff, .985 = slow, .950 = fast
const float AntisurgeMaxVal = 1000;     //maximum the antisurge can get to, can be set above 1000 which will keep it at max open for a while before it closes
const int BOVantisurgeThreshold = 600; //if antisurge is greater than this, enable BOV as well

const float BoostLimitProportionalRate = .8;  //how aggressive the boost limiting to RPM happens
const float BoostLimitIntegralRate = .02f;  // How much of the current limiting rate gets carried over as an integral
const float BoostLimitIntegralFalloffRate = 0.96f; //How fast the integral rate falls off, Probably between .95 and .985 (it's sensitive)
//remainder after 1 second is FalloffRate ^25 (25 loops per second) .95 ^25 ~= 1/4, .97^25 ~= 1/2
const float BoostLimitIntegralMaxComp = 1.5;  //Maximium Integral contribution
const float BoostLimitDifferentialRate = .01f;     //How much differential rate there is
const float BoostLimitDifferentialMaxComp = 1.5;   //Maximum Differential contribution
const float BoostLimitDifferentialOverboostRatio = .75; //Percentage of full boost when differential factor starts to kick in


const float TurbineOverspeedStartLimitRPM = 120; // RPM*1000  Where the turbine limiting starts to kick in (Yellowline)
const float TurbineOverspeedLimitingWidth = 20; //RPM *1000 + above. Where the nozzle size should be increased due to turbine overspeed (redline)
const float MaxTSSspeedCompensation = 1.5;  // This is the additional opening of the vanes when turbine is at startlimitingRPM + limitingwidth

const float IdleWalkdownLimitDelay = 20; //delay before idle walkdown starts (seconds)
const float IdleWalkdownLimitSpeed = 1; //how fast the walkdown happens
const float IdleWalkdownLimitMaxPos = 500; //Maximum open position 
const float IdleWalkdownLimitMinTemp = 80; //Doesn't walkdown the idle position on unless warm (promote warmup) (VGT RAW temp)





#endif // !TuningParameters_h
