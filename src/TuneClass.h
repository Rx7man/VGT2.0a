#pragma once
#include "Map1d.h"
#ifndef TuneClass_h
#define TuneClass_h


class TuneClass
{
public:
	Map1d LilbbMap;     //Main map
	Map1d AutobrakeMap; //Autobrake position
	Map1d BoostRPMMap;  //Maximum boost based on RPM, helps prevent bark and blown head gaskets, this is the compensator that does it nicely
	Map1d RPMminimumPositionMap; //Minimum position based on engine RPM
	Map1d BoostMinimumPositionMap; //minimum position based on boost, may help keep the truck driveable if other sensors fail
	Map1d EGPminimumPositionMap;   //minimum position based on backpressure
	Map1d TSSminimumPositionMap;   //Minimum position based on turbine speed (should be below the LilbbMap to allow for compensators)


	int TurbineSpeedDivisions[8] = { 0, 20, 40, 60, 80, 100, 120, 140 };
	int TpsRowLabels[6] = { 0, 5, 20, 40, 60, 100 };
	int RPMdivisions[10] = { 0, 500, 750, 1000, 1250, 1500, 2000,  2500, 3000, 5000 };
	int PressureDivisions[10] = { 0, 5, 10, 15, 20, 25, 30, 40, 60, 90 };

	//                      0   20   40   60   80   100  120  140 
	int LilbbMapData[8] = { 90,  90,  80,  70,  60,  50,  20,  0 };


	//                    RPM    0   500    1000    1500    2500    5000  Uses RpmColumnLabels as breakpoints
	int AutobrakeMapData[10] = { 90, 90, 90, 90, 96, 95, 94, 93, 90, 80 };
	int BoostRPMMapData[10] = { 25,  5,  5,  7,  10, 29, 25, 35, 40, 40 };


	//                    RPM                 500    1000    1500    2500    5000  Uses RpmColumnLabels as breakpoints
	int RPMminimumPositionMapData[10] = { 60, 96, 96, 91, 89, 84, 80, 75, 70, 50 };

	//                                   0   20  40  60  80  100 120 140  
	int TSSminimumPositionMapData[8] = { 96, 96, 96, 90, 85, 70, 30, 10 };

	//								      0    5   10     20      30  40  60  90
	int BoostMinimumPositionMapData[10] = { 96, 93, 90, 85, 80, 75, 70, 50, 30, 10 };

	//								    0    5   10      20      30  40  60  90
	int EGPminimumPositionMapData[10] = { 96, 95,  94, 90, 85, 80, 70, 60, 30, 10 };
	
	
	
	int VGTstartPosition; //VGT position when starting or not running

	int MinRunRPM = 400; //minimum RPM to consider the engine running, should be just a bit faster than the fastest crank speed
	int MaxIdleRPM = 800; //maximum RPM to consider it idle
	int MaxIdleTPS = 3;  //maximum TPS to consider it idle, if you have very steady TPS sensor you can set very low, it's not critical though


	float MaxTotalCompensation = 3;
	float MinTotalCompensation = .5;

	float JakeBoostFactor = .8;            //partially closes turbo to build more boost when TPS > JakeBoostTPS
	int   JakeBrakeTPSthreshold = 5;

	int AuxCompTSSLowPoint = 40; //TSS below this will be soley influenced by Aux1 pot,
	int AuxCompTSSHighPoint = 80; //TSS above this will be soley influenced by Aux2 pot, if TSS is between, it is interpolated

	int AntisurgeMinimumTPSslope = 50; //Set higher to make less sensitive to accidental triggers
	float AntisurgeResponseRate = .5; //higher number = more response to
	float AntisurgeFalloffRate = .96; //higher number = faster falloff, .985 = slow, .950 = fast
	float AntisurgeTSScancelSpeed = 50; //cancels antisurge when TSS falls below this AND TPS slope is positive
	float AntisurgeMaxVal = 1000;     //maximum the antisurge can get to, can be set above 1000 which will keep it at max open for a while before it closes
	int BOVantisurgeThreshold = 600; //if antisurge is greater than this, enable BOV as well

	float BoostLimitProportionalRate = .8;  //how aggressive the boost limiting to RPM happens
	float BoostLimitIntegralRate = .02f;  // How much of the current limiting rate gets carried over as an integral
	float BoostLimitIntegralFalloffRate = 0.96f; //How fast the integral rate falls off, Probably between .95 and .985 (it's sensitive)
	//remainder after 1 second is FalloffRate ^25 (25 loops per second) .95 ^25 ~= 1/4, .97^25 ~= 1/2
	float BoostLimitIntegralMaxComp = 1.5;  //Maximium Integral contribution
	float BoostLimitDifferentialRate = .01f;     //How much differential rate there is
	float BoostLimitDifferentialMaxComp = 1.5;   //Maximum Differential contribution
	float BoostLimitDifferentialOverboostRatio = .75; //Percentage of full boost when differential factor starts to kick in


	float TurbineOverspeedStartLimitRPM = 120; // RPM*1000  Where the turbine limiting starts to kick in (Yellowline)
	float TurbineOverspeedLimitingWidth = 20; //RPM *1000 + above. Where the nozzle size should be increased due to turbine overspeed (redline)
	float MaxTSSspeedCompensation = 1.5;  // This is the additional opening of the vanes when turbine is at startlimitingRPM + limitingwidth

	float IdleWalkdownLimitDelay = 20; //delay before idle walkdown starts (seconds)
	float IdleWalkdownLimitSpeed = 1; //how fast the walkdown happens
	float IdleWalkdownLimitMaxPos = 500; //Maximum open position
	float IdleWalkdownLimitMinTemp = 80; //Doesn't walkdown the idle position on unless warm (promote warmup) (VGT RAW temp)


	TuneClass() {

		LilbbMap = Map1d(8);
		LilbbMap.MapData = LilbbMapData; //this should be set when switches change but just give it something to start with
		LilbbMap.Divisions = TurbineSpeedDivisions;
		LilbbMap.Scaling = 10;
       
		TSSminimumPositionMap = Map1d(8);
		TSSminimumPositionMap.MapData = TSSminimumPositionMapData;
		TSSminimumPositionMap.Divisions = TurbineSpeedDivisions;
		TSSminimumPositionMap.Scaling = 10;

		AutobrakeMap = Map1d(10);
		AutobrakeMap.MapData = AutobrakeMapData;
		AutobrakeMap.Divisions = RPMdivisions;
		AutobrakeMap.Scaling = 10;

		BoostRPMMap = Map1d(10);
		BoostRPMMap.MapData = BoostRPMMapData;
		BoostRPMMap.Divisions = RPMdivisions;
		BoostRPMMap.Scaling = 1;  //not 10! this is boost not position

		RPMminimumPositionMap = Map1d(10);
		RPMminimumPositionMap.MapData = RPMminimumPositionMapData;
		RPMminimumPositionMap.Divisions = RPMdivisions;
		RPMminimumPositionMap.Scaling = 10;

		
		BoostMinimumPositionMap = Map1d(10);
		BoostMinimumPositionMap.MapData = BoostMinimumPositionMapData;
		BoostMinimumPositionMap.Divisions = PressureDivisions;
		BoostMinimumPositionMap.Scaling = 10;

		EGPminimumPositionMap = Map1d(10);
		EGPminimumPositionMap.MapData = EGPminimumPositionMapData;
		EGPminimumPositionMap.Divisions = PressureDivisions;
		EGPminimumPositionMap.Scaling = 10;

		
	};



private:

};



#endif // !Tunes_h
