/*
 Name:		VGT2.ino
 Created:	2/13/2023 4:37:04 PM
 Author:	Y2D
*/


#include "PinDefs.h"
#include "math.h"
#include "HelperFunctions.h"
#include "CANLibrarymaster/can.h"
#include "Timer/Timer.h"
#include "MyPIDs.h"
#include "MyVar.h"
#include "Map1d.h"
#include "MyDataTypes.h"
#include "MyAnalogSensor.h"

#include "VGTstuff.h"


//#define baudrate 115200
//#define baudrate 115200
#define baudrate 921600

MCP CAN1(CanControllerPin); //define CAN bus

volatile uint16_t TSScounts;      //counter for turbine shaft speed ISR and calculations
volatile uint16_t RPMcounts;      //counter for engine RPM ISR and calculations

char SerialMode;

#define ManualRecalibrate 0

tristateswitch Switch1Status = undefined;
tristateswitch Switch2Status = undefined;

boolean Running = false;
boolean Idling = false;

boolean JakeSwitch = false;
boolean ThumbSwitch = false;
boolean JakeBrakeMode = false;		 //Jake switch active and low TPS
boolean JakeBoostMode = false;		 //Jake switch active and high TPS
boolean LimitingActive = false;      //When any MINIMUM size limits are active, except antisurge
boolean AutobrakeActive = false;       //Autobrake mode is actually doing something
boolean AntisurgeActive = false;     //Antisurge is actually doing something
boolean BOVactive = false;

unsigned int ProgressiveTiming[10]; //to monitor how long each part of the program timing actually takes
unsigned int CalculationLoops;

float RunningTime = 0; //Running time in seconds
float IdlingTime = 0;  //Idling time in seconds

int CurrentMap = undefined; //Keeps track of which map is currently in use, start it with a map number that doesn't exist so it initializes properly
int CurrentMode = undefined; //Keeps track of which run mode is currently in use

float Aux1; //Left Auxilaiary pot (scaled 0-100)
float Aux2; // Right Auxiliary pot (scaled 0-100)
int loopcounts = 0;         //number of times it runs through the loop between serial updates
int blinksequencer = 0;

#pragma region Program timing

int SerialUpdateTiming = 250;

int Timer0Interval = 4;  //Progressive controller trigger, as well as blinksequencer
int Timer1Interval = 20; // update turbo position
int Timer3Interval = SerialUpdateTiming; //time between serial updates
int MaxBlinkSequencer = 20; //20 = 5% increments.. going too accurate will cause it to run blink very slowly

Timer t0;
Timer t1;
Timer t3;
#pragma endregion


#pragma region Mapping




Map1d LilbbMap(8);
Map1d TSSminimumPositionMap(8); //Minimum position based on turbine shaft speed, prevents overspeed regardless of other settings

Map1d AutobrakeMap(10); //Autobrake position
Map1d BoostRPMMap(10); //Maximum boost based on RPM, helps prevent bark and blown head gaskets, this is the compensator that does it nicely
Map1d RPMminimumPositionMap(10); //Minimum position based on engine RPM
Map1d BoostMinimumPositionMap(10); //minimum position based on boost, may help keep the truck driveable if other sensors fail
Map1d EGPminimumPositionMap(10);      //minimum position based on backpressure

int TurbineSpeedDivisions[] = { 0, 20, 40, 60, 80, 100, 120, 140 };
int TpsRowLabels[] = { 0, 5, 20, 40, 60, 100 };
int RPMdivisions[] = { 0, 500, 750, 1000, 1250, 1500, 2000,  2500, 3000, 5000 };
int PressureDivisions[] = { 0, 5, 10, 15, 20, 25, 30, 40, 60, 90 };

//                    RPM      500    1000    1500    2500    5000  Uses RpmColumnLabels as breakpoints
int AutobrakeMapData[] = { 90, 90, 90, 90, 96, 95, 94, 93, 90, 80 };
int BoostRPMMapData[] = { 10,  5,  5,  5, 10, 15, 20, 30, 40, 40 };

//                    RPM               500    1000    1500    2500    5000  Uses RpmColumnLabels as breakpoints
int RPMminimumPositionMapData[] = { 80, 96, 96, 96, 95, 94, 92, 90, 80, 50 };

//                                  0   20  40  60  80  100 120 140 
int TSSminimumPositionMapData[] = { 96, 96, 96, 95, 90, 70, 30, 10 };

//								      0    5   10     20      30  40  60  90
int BoostMinimumPositionMapData[] = { 96, 93, 90, 85, 80, 75, 70, 50, 30, 10 };

//								    0    5   10      20      30  40  60  90
int EGPminimumPositionMapData[] = { 96, 95,  94, 90, 85, 80, 70, 60, 30, 10 };




/*
Map2d PositionMap;

uint8_t *PositionMapData;
uint8_t  PositionMapData_Up[] =
{
	//      750    1250    2000    3000
	90, 60, 90, 90, 96, 95, 95, 94, 90, 90,  //0%  TPS
	90, 60, 80, 80, 85, 70, 60, 60, 60, 60,  //10%
	90, 90, 80, 80, 70, 70, 60, 60, 60, 60,  //20%
	90, 70, 80, 80, 75, 70, 60, 60, 60, 60,  //40%
	90, 50, 80, 75, 75, 70, 70, 70, 65, 50,  //60%
	90, 50, 80, 75, 75, 70, 70, 70, 65, 50,  //100%
};
uint8_t  PositionMapData_Middle[] =
{
	//      750    1250    2000    3000
	90, 60, 90, 90, 96, 95, 95, 94, 90, 90,  //0%  TPS
	90, 60, 80, 80, 85, 75,  0,  0,  0,  0,  //10%
	90, 90, 80, 80, 70, 70, 60,  0,  0,  0,  //20%
	90, 70, 80, 80, 75, 70, 65, 60, 40,  0,  //40%
	90, 50, 80, 75, 75, 70, 70, 70, 65, 50,  //60%
	90, 50, 80, 75, 75, 70, 70, 70, 65, 50,  //100%
};
uint8_t  PositionMapData_Down[] =
{
	//      750    1250    2000    3000
	95, 90, 94, 96, 95, 94, 93, 92, 91, 90,  //0%  TPS
	90, 95, 90, 85, 70, 50, 30, 30, 30, 30,  //10%
	90, 90, 90, 85, 70, 50, 20, 30, 30, 30,  //20%
	90, 70, 90, 85, 80, 60, 30, 20, 20, 20,  //40%
	90, 50, 85, 75, 70, 60, 40, 20, 20, 10,  //60%
	90, 50, 80, 75, 70, 60, 40, 20, 20, 10,  //100%
};
*/
int* LilbbMapData;
int LilbbMapData_Up[] = //Tow
// 0   20   40   60   80   100  120  140 
{ 95,  95,  90,  80,  75,  50,  30,  0 };

int LilbbMapData_Middle[] = //Normal
// 0   20   40   60   80   100  120  140 
{ 90,  90,  85,  80,  70,  52,  30,  0 };

int LilbbMapData_Down[] = //Low Boost
// 0   20   40   60   80   100  120  140 
{ 90,  90,  80,  70,  60,  50,  20,  0 };

#pragma endregion Mapping

#pragma region globals
enum indexes :byte {
	Undefined,  // just a dummy here that I can use if I haven't set anything, MUST BE FIRST (0 index)

	//The following are compensators that apply to nozzle SIZE (not position)
	BaseNozzleSizeIndex, //Nozzle size based on raw map data, no compensators applied

	OverspeedCompIndex, //might not need this one with the TSSminimumPosition 
	BoostCompIndex,
	AutoCompIndex, //Total compensation but not including the Aux compensation
	AuxCompIndex, AuxCompLowIndex, AuxCompHighIndex, //Manual compensation, high and low broken up for debugging
	CompensatedNozzleSizeIndex,
	TotalCompensationIndex,


	//The following are position based
	BasePositionIndex,  //Position from Lilbbmap, unadjusted
	CompensatedPositionIndex, //Position after compensators but before minimum position limits
	LimitedPositionIndex, //Position after compensators and minimum size limits have been applied (except antisurge)

	AutobrakeMapPositionIndex, //Position autobrake map is returning, active or not
	AutobrakedPositionIndex, //Position after LimitedPosition and Autobrake have been applied

	FinalPositionIndex, //What is actually sent to the VGT controller

	//Safeties to prevent overspeeding and such, and can help keep the truck driveable if a sensor fails
	AntisurgeMinimumPositionIndex, //Opens vanes when throttle is release
	TSSMinimumPositionIndex, //minimum position based on shaft speed
	RPMminimumPositionIndex, //minimum position based on engine RPM
	BoostRPMminimumPositionIndex,  //minimum position based on boost
	EGPminimumPositionIndex,       //minimum position based on backpressure
	IdleWalkdownMinimumPositionIndex, //limit based on the idle walkdown
	MinimumPositionIndex,    //The combination of all the above
	LimitingAmountIndex,    //How much limiting is actually occurring Positive when limiting

	//Internal function monitoring

	BoostCompIntegratorAccumulatorIndex,
	BoostCompSetpointIndex,
	BoostCompOverboostRatioIndex,

	BoostCompProportionalFactorIndex,
	BoostCompIntegralFactorIndex,
	BoostCompDifferentialFactorIndex,

	AntisurgeCarryoverIndex,       //Monitor the carryover


	IndexCount //Keep this last so you can see how many indices there are
};


float GlobalFloats[IndexCount];

MyVar RPM; //Engine RPM
MyVar TSS; //Turbine shaft speed
MyVar TPS; //Throttle position
MyVar MAP; //Manifold pressure
MyVar EGP; //Exhaust pressure
MyVar BPS; //brake pressure sensor
MyVar FPS; //Fuel pressure sensor
MyVar SendPosition;            //The value we're sending to the VGT

#pragma endregion  


#pragma region Normal mode parameters
//NORMAL MODE PARAMETERS
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

#pragma endregion Normal mode parameters
#pragma region Sensor_Calibration
int MinRawTPSposition = 25; // this automatically gets reduced to the minimum TPS raw position
int MaxRawTPSposition = 500; // this needs to be manually adjusted to WOT

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
#pragma endregion






#pragma region Setup Stuff

void setup() {

	// put your setup code here, to run once:
	Serial.begin(baudrate);
	Serial.println("Welcome to BigBlackBox V1.7e");
	Serial.println("Apply jake switch and hold thumbswitch while powering up to start a VGT recalibrate");

	InitializePins();
	InitializeMyVars();
	InitializeMaps();
	InitializeCAN();
	InitializeTimers();
	InitializeInterrupts();
	InitializeRecalibrate();

	TestOutputs(); //cycle outputs.. relays, indicator lamps, etc a few times
	TestVGT(1); //cycle VGT to limits a couple times

	TSScounts = 0;
	TSS.Reset();
	RPMcounts = 0;
	RPM.Reset();

	Serial.println("End of Setup");
}
void InitializeRecalibrate() {
	if (digitalRead(JakeSwitchPin) || digitalRead(ThumbSwitchPin)) { //active low switches, remember?
		return; //don't continue even start if the switches aren't right
	}
	Serial.println("Starting VGT recalibrate in 10 seconds, Abort by releasing any switch");

	for (int i = 0; i <= 10; i++) {
		Serial.print(int((10 - i) / 2));
		Serial.print("...  ");
		digitalWrite(AmberLedPin, HIGH);
		digitalWrite(GreenLedPin, HIGH);
		delay(250);
		digitalWrite(AmberLedPin, LOW);
		digitalWrite(GreenLedPin, LOW);
		delay(250);
		Serial.print(".");
		if (digitalRead(ThumbSwitchPin) || digitalRead(JakeSwitchPin)) {
			Serial.println("\nAborting recalibrate");
			return;
		}
	}
	Serial.println("\nRecalibrating, this usually takes 15 seconds");
	SendVgtRecalibrate();  //send actual recalibration code to controller, then wait

	unsigned long startime = millis();
	while (startime + 20000 >= millis()) { //flash amber light for 20 seconds
		digitalWrite(AmberLedPin, HIGH);
		delay(125);
		digitalWrite(AmberLedPin, LOW);
		delay(125);

		while (CAN1.msgAvailable() > 0) { //read and print can messages if there are any
			CAN msg;
			CAN1.read(&msg);
			PrintCANmessage(msg);
		}
	}
	digitalWrite(GreenLedPin, HIGH); //Turn on green LED to signal finished
	Serial.println("\nTurn key off, it should be recalibrated");
	while (1) {} //hard stop
}
void InitializeCAN() {
	Serial.print("Starting Can bus... ");

	CAN1.begin(NORMAL, 250);
	Serial.println("Done");

	CAN1.clearRxBuffers();

}
void InitializeTimers() {
	Serial.println("Starting Timers");
	t0.every(Timer0Interval, Timer0Routine);
	Serialprintint("Timer 0 interval (ms) = ", Timer0Interval);
	t1.every(Timer1Interval, Timer1Routine);
	Serialprintint("Timer 1 interval (ms) = ", Timer1Interval);
	t3.every(Timer3Interval, Timer3Routine);
	Serialprintint("Timer 3 interval (ms) = ", Timer3Interval);
}
void InitializePins() {
	Serial.println("Initializing pins");
	analogReference(DEFAULT);
	//analogReference(EXTERNAL);

	pinMode(8, INPUT); //frequencymeasure pin must not pull up!


		//Analog inputs
	for (int i = 0; i < 16; i++) {
		pinMode(A0 + i, INPUT); //Just set all the analog pins to input!
	}
	for (int i = 0; i < 16; i++) {
		analogRead(A0 + i); //Run an analog read on all the pins and get it all settled
		delay(1);
	}



	pinMode(CanControllerPin, OUTPUT);
	pinMode(EgtThermoPin, OUTPUT);
	pinMode(CotThermoPin, OUTPUT);
	pinMode(IatThermoPin, OUTPUT);
	pinMode(CtsThermoPin, OUTPUT);

	digitalWrite(CanControllerPin, HIGH);
	digitalWrite(EgtThermoPin, HIGH);
	digitalWrite(CotThermoPin, HIGH);
	digitalWrite(IatThermoPin, HIGH);
	digitalWrite(CtsThermoPin, HIGH);



	//Digital inputs
	pinMode(Switch1HighPin, INPUT_PULLUP);
	pinMode(Switch1LowPin, INPUT_PULLUP);
	pinMode(Switch2HighPin, INPUT_PULLUP);
	pinMode(Switch2LowPin, INPUT_PULLUP);
	pinMode(ThumbSwitchPin, INPUT_PULLUP);
	pinMode(JakeSwitchPin, INPUT_PULLUP);

	//Digital Outputs
	pinMode(AmberLedPin, OUTPUT);
	pinMode(GreenLedPin, OUTPUT);
	pinMode(LineLockPin, OUTPUT);
	pinMode(CruiseLedPin, OUTPUT);
	pinMode(CruiseVacPin, OUTPUT);
	pinMode(CruiseVentPin, OUTPUT);
	pinMode(BOVsolenoidPin, OUTPUT);

}
void InitializeInterrupts() {
	//Interrupts
	attachInterrupt(digitalPinToInterrupt(TssPin), TssISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(RpmPin), RpmISR, FALLING);
}
void InitializeMaps() {
	Serial.println("Initializing Maps");

	// 2D maps---------------------------------------------
	//PositionMap.rowData = TpsRowLabels;
	//PositionMap.columnData = RpmColumnLabels;
	//PositionMap.mapData = PositionMapData_Middle;


	// 1D maps---------------------------------------------
	LilbbMap.MapData = LilbbMapData_Middle; //this should be set when switches change but just give it something to start with
	LilbbMap.Divisions = TurbineSpeedDivisions;
	LilbbMap.Scaling = 10;
	//TODO: should actually use the "Scaling" option here for each map rather than having to multiply x10 later in the code


	AutobrakeMap.MapData = AutobrakeMapData;
	AutobrakeMap.Divisions = RPMdivisions;
	AutobrakeMap.Scaling = 10;

	BoostRPMMap.MapData = BoostRPMMapData;
	BoostRPMMap.Divisions = RPMdivisions;
	BoostRPMMap.Scaling = 1.0f;  //not 10! this is boost not position

	RPMminimumPositionMap.MapData = RPMminimumPositionMapData;
	RPMminimumPositionMap.Divisions = RPMdivisions;
	RPMminimumPositionMap.Scaling = 10;

	TSSminimumPositionMap.MapData = TSSminimumPositionMapData;
	TSSminimumPositionMap.Divisions = TurbineSpeedDivisions;
	TSSminimumPositionMap.Scaling = 10;

	BoostMinimumPositionMap.MapData = BoostMinimumPositionMapData;
	BoostMinimumPositionMap.Divisions = PressureDivisions;
	BoostMinimumPositionMap.Scaling = 10;

	EGPminimumPositionMap.MapData = EGPminimumPositionMapData;
	EGPminimumPositionMap.Divisions = PressureDivisions;
	EGPminimumPositionMap.Scaling = 10;

}
void InitializeMyVars() {
	Serial.println("Initializing Variables");
	//Smoothing, a number higher than .9 gives a lot of smoothing, a number closer to 0 gives less smoothing
	TSS.Name = "TSS";
	TSS.Smoothing = .90;
	TSS.SlopeSmoothing = .90;

	RPM.Name = "RPM";
	RPM.Smoothing = 0.9;
	RPM.SlopeSmoothing = .9;

	MAP.Name = "MAP";
	MAP.Smoothing = .95;
	MAP.SlopeSmoothing = .95;

	EGP.Name = "EGP";
	EGP.Smoothing = .95;
	EGP.SlopeSmoothing = .95;

	TPS.Name = "TPS";
	TPS.Smoothing = .95;
	TPS.SlopeSmoothing = .95;

	FPS.Name = "FPS";
	FPS.Smoothing = .98;
	FPS.SlopeSmoothing = .98;

	BPS.Name = "BPS";
	BPS.Smoothing = .95;
	BPS.SlopeSmoothing = .95;

	SendPosition.Name = "SendPos";
	SendPosition.Smoothing = .5;
	SendPosition.SlopeSmoothing = .1;
}
void TestOutputs() {
	//Blinks lights and triggers relays briefly to test on startup, also indicates when setup is complete
	for (int i = 0; i < 5; i++) {


		digitalWrite(GreenLedPin, HIGH);
		digitalWrite(AmberLedPin, HIGH);
		digitalWrite(LineLockPin, HIGH);
		digitalWrite(CruiseLedPin, HIGH);
		digitalWrite(BOVsolenoidPin, HIGH);

		delay(50);

		digitalWrite(GreenLedPin, LOW);
		digitalWrite(AmberLedPin, LOW);
		digitalWrite(LineLockPin, LOW);
		digitalWrite(CruiseLedPin, LOW);
		digitalWrite(BOVsolenoidPin, LOW);
		delay(50);
	}



}
void TestVGT(int repeats) {
	for (int r = 0; r < repeats; r++) {
		Serial.println("Testing... Closing");
		for (int i = 0; i <= 1000; i += 100) {
			FormatForVGT(i);
			delay(25);

		}
		delay(10);
		Serial.println("Testing.. Opening");
		for (int i = 1000; i >= 0; i -= 100) {
			FormatForVGT(i);
			delay(25);
		}
		delay(10);
	}
}


void loop() {
	loopcounts++;
	ReadCanMessage();
	t0.update();
	t1.update();
	t3.update();

	//ProgressiveController(); //reads one variable at a time, maximizing how fast inputs are read without compromising timers
	digitalWrite(LineLockPin, ThumbSwitch);

}
void Timer0Routine() {
	ProgressiveController();

	//Serial.print("!");
}
void Timer1Routine() {
	UpdateRunModes();

	int SendPosition;

	if (Running) {
		if (JakeBrakeMode) {
			SendPosition = 1000;
		}
		else if (!JakeBrakeMode && VgtRealPosition > 960) {
			SendPosition = 900; // "Unsticks" the VGT and makes sure it comes out of brake mode quickly by sending a larger size to deliberately move it
		}
		else {
			SendPosition = constrain(GlobalFloats[FinalPositionIndex], 0, 960);
		}
	}
	else {
		SendPosition = VGTstartPosition; //if not running, set to static position and don't apply jake, etc
	}

	FormatForVGT(SendPosition);

}
void Timer3Routine()/* Selects serial mode */ {
	UpdateModeAndMap();
	SelectSerialMode();

	CanMessagesReceived = 0;
	loopcounts = 0;
	//digitalWrite(CruiseLedPin, !digitalRead(CruiseLedPin));
}
void TssISR() {
	TSScounts++;
	//Serial.println("PING");
}
void RpmISR() {
	RPMcounts++;
	//Serial.println("PONG");
}
void CalculateRPM() {
	unsigned long RPMtimeout = 1000000UL;
	unsigned int RPMminCounts = 10;
	unsigned int CrankPulsePerRev = 2;
	static unsigned long LastMicros = micros();
	unsigned long Now = micros();
	unsigned long Timespan = Now - LastMicros;
	unsigned int NewRPM = 60000000UL / Timespan * RPMcounts / CrankPulsePerRev;
	int counts = RPMcounts;
	if (Timespan > RPMtimeout) { //a second has elapsed without accumulating enough counts, thus the engine is stopped
		RPM.Reset();
		LastMicros = Now;
	}
	else  if (counts > RPMminCounts) {//only update value if we have enough counts to do something with.. rather increase the timespan (by waiting until the next go round) to get a closer approximation
		RPMcounts = 0;
		LastMicros = Now;
		RPM.UpdateValue(NewRPM);
	}
	//else wait for more counts
}
void CalculateTSS() {
	//calculates turbine shaft speed, RPM is to be multiplied by 1000
	unsigned long TSStimeout = 1000000UL;
	unsigned long TSSminCounts = 100;
	static unsigned long LastMicros = micros();
	unsigned long Now = micros();
	unsigned long Timespan = Now - LastMicros;
	float NewRPM = 60000.0f / Timespan * TSScounts;
	if (Timespan > TSStimeout) { //if we haven't gotten a tick in 1 second, set the RPM to zero
		TSS.Reset();
		LastMicros = Now;
	}
	else if (TSScounts > TSSminCounts) {//only update value if we have enough counts to do something with.. rather increase the timespan (by waiting until the next go round) to get a closer approximation
		TSScounts = 0;
		LastMicros = Now;
		TSS.UpdateValue(NewRPM);
	}
}

#pragma endregion


void ReadSwitches() {
	//Serial.println("reading switches");
	//All switches are on internal pullups, thus when switch is on, level is low
	ThumbSwitch = !digitalRead(ThumbSwitchPin); //inverts so it's logical
	JakeSwitch = !digitalRead(JakeSwitchPin);   //inverts so its logical


	if (digitalRead(Switch1HighPin) == 0) {
		Switch1Status = up;
	}
	else if (digitalRead(Switch1LowPin) == 0) {
		Switch1Status = down;
		//Serial.println("Switch1 low");
	}
	else {
		Switch1Status = middle;
	}
	//serialprint("switch1h", digitalRead(Switch1HighPin), 0);
	//serialprint("switch1l", digitalRead(Switch1LowPin), 0);
	//serialprint("switch2h", digitalRead(Switch2HighPin), 0);
	//serialprint("switch2l", digitalRead(Switch2LowPin), 0);

	Switch2Status = up;

	/*if (digitalRead(Switch2LowPin) == 0) {
		Switch2Status = down;
	}
	else 	if (digitalRead(Switch2HighPin) == 0) {
		Switch2Status = up;
	}

	else {
		Switch2Status = middle;
	}
	*/
}
void UpdateModeAndMap() {
	//called from the Timer3 routine
	tristateswitch NewMap = Switch1Status;
	tristateswitch NewMode = Switch2Status;

	if (NewMap != CurrentMap || CurrentMap == undefined) {

		switch (NewMap) {
		case up:
			Serial.println("Switching to UP map");
			//PositionMap.mapData = PositionMapData_Up;
			LilbbMap.MapData = LilbbMapData_Up;
			break;
		case middle:
			Serial.println("Switching to MIDDLE map");
			//PositionMap.mapData = PositionMapData_Middle;
			LilbbMap.MapData = LilbbMapData_Middle;
			break;
		case down:
			Serial.println("Switching to DOWN map");
			//PositionMap.mapData = PositionMapData_Down;
			LilbbMap.MapData = LilbbMapData_Down;
			break;
		default:
			//Serial.println("Somehow we have an undefined new map");
			break;
		}

		CurrentMap = NewMap;//update the "lastmap" variable so it doesn't need to keep updating
	}
	/*
	if (NewMode != CurrentMode || CurrentMode == undefined) {
		switch (NewMode) {
		case up:
			Serial.println("Switching to UP mode (AutoBrake)");
			break;
		case middle:
			Serial.println("Switching to MIDDLE mode");
			break;
		case down:
			Serial.println("Switching to DOWN mode");
			break;
		default:
			//Serial.println("Somehow we have an undefined mode");
			return;
		}
		CurrentMode = Switch2Status;
	}
*/
}
void UpdateRunModes() {// Updates the "running" and "Idling" variables and calculates run and idle times

	//Called from Timer1 routine
	JakeBrakeMode = (JakeSwitch && (TPS.Value < JakeBrakeTPSthreshold));
	JakeBoostMode = (JakeSwitch && (TPS.Value > JakeBrakeTPSthreshold));


	static unsigned long LastUpdateTime;
	float Timespan;  //timespan in SECONDS from last pass through

	if (LastUpdateTime == 0) { //lets not do math with garbage
		LastUpdateTime = millis();
	}
	else {
		unsigned long currentmillis = millis();
		Timespan = float(currentmillis - LastUpdateTime) / 1000.0f;
		LastUpdateTime = currentmillis;

	}

	if (RPM.Value >= MinRunRPM) {

		if (Running) {//if already running, just calculate the start time
			RunningTime += Timespan;
		}
		else {//if not already running, set the start time and running status
			Running = true;
		}
	}
	else {
		Running = false;
		RunningTime = 0;
	}

	if (Running && RPM.Value <= MaxIdleRPM && TPS.Value <= MaxIdleTPS) {
		if (Idling) { //if idling already, calculate idle time
			IdlingTime += Timespan;
		}
		else { //if not already idling, Set idle start time and idle status
			Idling = true;
		}
	}
	else { //if any condition for idling is unmet, stop idling mode
		Idling = false;
		IdlingTime = 0;
	}

}


void UpdatePosition() {
	//called from ProgressiveController
	UpdateBasePosition();


	//Jake mode logic is in Timer1 routine
	GlobalFloats[CompensatedNozzleSizeIndex] = max(GlobalFloats[BaseNozzleSizeIndex], 0) * GlobalFloats[TotalCompensationIndex];
	GlobalFloats[CompensatedPositionIndex] = min(960, GetVGTpositionFromNozzleSize(GlobalFloats[CompensatedNozzleSizeIndex]));

	GlobalFloats[LimitingAmountIndex] = GlobalFloats[CompensatedPositionIndex] - GlobalFloats[MinimumPositionIndex];  // POSITIVE WHEN LIMITING

	float LimitedPosition = min(GlobalFloats[CompensatedPositionIndex], GlobalFloats[MinimumPositionIndex]); // Apply ALL minimum positions except Antisurge which MUST be last
	GlobalFloats[LimitedPositionIndex] = LimitedPosition;

	int AutobrakeMapPosition = AutobrakeMap.Interpolate(RPM.Value);

	GlobalFloats[AutobrakeMapPositionIndex] = AutobrakeMapPosition;


	float AutobrakedPosition = mapf(TPS.Value, 1, 5, AutobrakeMapPosition, LimitedPosition, true);

	AutobrakeActive = (LimitedPosition < AutobrakedPosition);
	GlobalFloats[AutobrakedPositionIndex] = AutobrakedPosition;

	float finalposition = min(AutobrakedPosition, GlobalFloats[AntisurgeMinimumPositionIndex]);
	//float finalposition = min(LimitedPosition, GlobalFloats[AntisurgeMinimumPositionIndex]);

	GlobalFloats[FinalPositionIndex] = finalposition; // 960->0 constraints are applied in Timer1 routine before sending, along with Jakebrake
}


void UpdateBasePosition() {
	//Called from UpdatePosition()
	int BasePosition = LilbbMap.Interpolate(TSS.Value);
	GlobalFloats[BasePositionIndex] = BasePosition;  //Set this to start with and then any unhandled part following will be correct
	//Serialprintint("Base position ", BasePosition);
	GlobalFloats[BaseNozzleSizeIndex] = GetNozzleSizeFromVGTposition(BasePosition);
}

void UpdateMinimumPosition() {
	// This is where you find all the minimum positions and put them into one variable
	//Called from ProgressiveController

	GlobalFloats[RPMminimumPositionIndex] = RPMminimumPositionMap.Interpolate(RPM.Value);
	GlobalFloats[TSSMinimumPositionIndex] = TSSminimumPositionMap.Interpolate(TSS.Value);
	GlobalFloats[BoostRPMminimumPositionIndex] = BoostMinimumPositionMap.Interpolate(MAP.Value);
	GlobalFloats[EGPminimumPositionIndex] = EGPminimumPositionMap.Interpolate(0);//EGP.value if enabled

	int minpos = 1000;
	//comment out any 'minpos' line below to disable using that 

	minpos = min(GlobalFloats[RPMminimumPositionIndex], minpos);
	minpos = min(GlobalFloats[TSSMinimumPositionIndex], minpos);
	minpos = min(GlobalFloats[BoostRPMminimumPositionIndex], minpos);
	minpos = min(GlobalFloats[EGPminimumPositionIndex], minpos);
	minpos = min(GlobalFloats[IdleWalkdownMinimumPositionIndex], minpos); //temporarlily diabled to check other stuff

	GlobalFloats[MinimumPositionIndex] = minpos;

}
void UpdateTotalCompensation() {
	//Called from Progressivecontroller();
	UpdateOverspeedCompensation();
	//UpdateBoostLimitCompensation(); //moved to ProgressiveController in it's own time spot


	float TotalCompensationValue = 1;
	TotalCompensationValue *= GlobalFloats[OverspeedCompIndex];  //apply turbine speed compensation
	TotalCompensationValue *= GlobalFloats[BoostCompIndex]; //Apply boost limit compensation

	GlobalFloats[AutoCompIndex] = TotalCompensationValue;

	TotalCompensationValue *= GlobalFloats[AuxCompIndex]; //Apply manual pot and jakek boost compensation last
	if (JakeBoostMode) { TotalCompensationValue *= JakeBoostFactor; }


	GlobalFloats[TotalCompensationIndex] = constrain(TotalCompensationValue, MinTotalCompensation, MaxTotalCompensation);
}


void UpdateAntiSurgeLimit() {
	//POSITIVE value opens vanes more, but converted to be an absolute position for output
	//Called from ProgressiveController
	static float Carryover;
	//The faster you let off the throttle, and the faster the turbine is going
	//the more chance of surge, so force nozzle open more aggressively
	float Antisurge = 0;
	if (TPS.Slope < -AntisurgeMinimumTPSslope) {
		Antisurge = max(Antisurge, -min(TPS.Slope, 100) * AntisurgeResponseRate * TSS.Value);
	}

	Antisurge = constrain(Antisurge, 0, AntisurgeMaxVal);  //Limit to maximum value
	Antisurge = max(Antisurge, Carryover);        //If the carryover is larger than the current antisurge amount, use the carryover for the math
	Carryover = Antisurge * AntisurgeFalloffRate; //Apply falloff to the carryover


	if (Carryover <= 100 || (TPS.Slope >= 10 && TPS.Value >= 20)) {//Just zero it out if it's smaller than the minimum or getting back on the throttle
		Carryover = 0;
		Antisurge = 0;
	}
	GlobalFloats[AntisurgeCarryoverIndex] = Carryover;
	GlobalFloats[AntisurgeMinimumPositionIndex] = constrain(1000 - Antisurge, 0, 960); //apply a minimum so that it can't go below zero, and reverse the numerical value

}

void UpdateIdleWalkdownLimit() {
	//Called from ProgressiveController
	static float IdleWalkdownLimitPosition = 960;

	if (IdlingTime > IdleWalkdownLimitDelay && VgtRawTemp > IdleWalkdownLimitMinTemp) {
		IdleWalkdownLimitPosition = constrain(IdleWalkdownLimitPosition - IdleWalkdownLimitSpeed, IdleWalkdownLimitMaxPos, 960);
	}
	else {
		IdleWalkdownLimitPosition = 960;
	}
	GlobalFloats[IdleWalkdownMinimumPositionIndex] = IdleWalkdownLimitPosition;
}
void UpdateBOVoutput() {
	AntisurgeActive = GlobalFloats[AntisurgeMinimumPositionIndex] <= BOVantisurgeThreshold;
	digitalWrite(BOVsolenoidPin, AntisurgeActive);
}
void UpdateOverspeedCompensation() {
	//returns a POSITIVE value greater than 1 when limiting is active and 1.00 when inactive
	float OverspeedCompensationValue = 1;
	if (TSS.Value > TurbineOverspeedStartLimitRPM) {
		OverspeedCompensationValue =
			mapf(TSS.Value, TurbineOverspeedStartLimitRPM, TurbineOverspeedStartLimitRPM + TurbineOverspeedLimitingWidth, 1, MaxTSSspeedCompensation, true);
	}
	GlobalFloats[OverspeedCompIndex] = OverspeedCompensationValue;
}
void UpdateAuxCompensation() {
	//called from SelectSerialMode since it doesn't need to be updated often

	//this sets the compensation so that it gradually goes from using the left pot for low RPM to the right pot for high RPM


	float LowCompensationRate = GetGenericCompensationExp(Aux1, .5, 2);
	float HighCompensationRate = GetGenericCompensationExp(Aux2, .5, 2);
	float LowCompensationValue = mapf(TSS.Value, AuxCompTSSLowPoint, AuxCompTSSHighPoint, 1, 0, true) * LowCompensationRate;
	float HighCompensationValue = mapf(TSS.Value, AuxCompTSSLowPoint, AuxCompTSSHighPoint, 0, 1, true) * HighCompensationRate;
	GlobalFloats[AuxCompIndex] = LowCompensationValue + HighCompensationValue;
	//Serialprint("Lowcomp rate   ", LowCompensationRate, 3);
	//Serialprint("Low comp value ", LowCompensationValue,3);

}
void UpdateBoostLimitCompensation() {
	//provides a value > 1 when limiting is active to increase nozzle size
	//Internally it works with values greater than zero
	static float MaxIntegralAccumulator = (BoostLimitIntegralMaxComp - 1) * BoostLimitIntegralRate; //
	static float IntegralAccumulator = 0;

	float Setpoint = BoostRPMMap.Interpolate(RPM.Value);  //Get max allowed boost from map
	float MAPvalue = MAP.Value;


	float OverboostDifference = max(0, MAPvalue - Setpoint);
	float OverboostRatio = MAPvalue / max(Setpoint, 1);  //max there to prevent divide by zero error, Setpoint ought to be > 5 anyhow

	float ProportionalFactor = 1;

	if (OverboostRatio > 1) {
		ProportionalFactor = ((OverboostRatio - 1) * BoostLimitProportionalRate) + 1; //May want to switch this to absolute rather than ratiometric

		IntegralAccumulator += OverboostDifference;
		IntegralAccumulator *= BoostLimitIntegralFalloffRate;

	}
	else {
		// If not limiting boost, cancel out any remaining integral to get a fresh start next time
		IntegralAccumulator = 0;
	}


	float IntegralFactor = 1 + (IntegralAccumulator * BoostLimitIntegralRate);
	if (IntegralFactor > BoostLimitIntegralMaxComp) { //limits integral amount
		IntegralAccumulator = MaxIntegralAccumulator;  //max is set at top of function once when program starts
		IntegralFactor = BoostLimitIntegralMaxComp;
	}

	float TempDiffFactor = MAP.Slope * BoostLimitDifferentialRate + 1; //Get the current slope and multiply by the rate
	float TempDiffFactor2 = mapf(OverboostRatio, BoostLimitDifferentialOverboostRatio, 1, 1, TempDiffFactor, true); //slowly bring it in from a percentage of the underboost
	float DifferentialFactor = constrain(TempDiffFactor2, 1 / BoostLimitDifferentialMaxComp, BoostLimitDifferentialMaxComp); //constrain



	//Save this stuff so we can monitor it outside the function
	GlobalFloats[BoostCompSetpointIndex] = Setpoint;
	GlobalFloats[BoostCompOverboostRatioIndex] = OverboostRatio;

	GlobalFloats[BoostCompProportionalFactorIndex] = ProportionalFactor;

	GlobalFloats[BoostCompIntegralFactorIndex] = IntegralFactor;
	GlobalFloats[BoostCompIntegratorAccumulatorIndex] = IntegralAccumulator;

	GlobalFloats[BoostCompDifferentialFactorIndex] = DifferentialFactor;


	//Actual output
	GlobalFloats[BoostCompIndex] = ProportionalFactor * IntegralFactor * DifferentialFactor;

}

void UpdateLEDs() {
	blinksequencer++;
	if (blinksequencer > MaxBlinkSequencer) { blinksequencer = 0; }
	boolean fastblink = (blinksequencer % 4 > 1);
	boolean slowblink = blinksequencer % 2 > 0;
	if (!Running) {
		digitalWrite(GreenLedPin, fastblink);
		digitalWrite(AmberLedPin, slowblink);
		return;
	}

	//difference between turbine speed compensation and base position (ignoring compensations) 
	int Limit = (GlobalFloats[LimitingAmountIndex]);

	if (Limit > 150) {
		digitalWrite(AmberLedPin, HIGH);
	}
	else if (Limit > 50) {
		digitalWrite(AmberLedPin, fastblink);
	}
	else	if (Limit >= 1) {
		digitalWrite(AmberLedPin, slowblink);
	}
	else {
		digitalWrite(AmberLedPin, LOW);
	}

	float comp = GlobalFloats[AutoCompIndex];
	if (comp > 1.5 || AntisurgeActive) {
		digitalWrite(GreenLedPin, HIGH);
	}
	else if (comp > 1.25) {
		digitalWrite(GreenLedPin, fastblink);
	}
	else if (comp > 1.01) {
		digitalWrite(GreenLedPin, slowblink);
	}
	else {
		digitalWrite(GreenLedPin, LOW);
	}
}

void ProgressiveController() {
	static int ProgressiveLoopNum = 0;
	ProgressiveLoopNum++;
	if (ProgressiveLoopNum > 10) { ProgressiveLoopNum = 0; }

	unsigned long starttime = micros();
	//Serialprintint("loop ", ProgressiveLoopNum);
	switch (ProgressiveLoopNum) {
	case 0:
		//Serial.println("0");
		//Serial.println("UpdateLEDs");
		UpdateLEDs();
		BPS.UpdateValue(Honeywell1000PSI.ReadFromPin(BrakePin));
		break;

	case 1:
		//Serial.println("1");
		ReadSwitches();
		EGP.UpdateValue(Honeywell100PSI.ReadFromPin(EgpPin));
		break;

	case 2:
		//Serial.println("2");
		MAP.UpdateValue(Honeywell100PSI.ReadFromPin(MapPin));
		break;

	case 3:
		//Serial.println("3");
		FPS.UpdateValue(Honeywell100PSI.ReadFromPin(FpsPin));
		break;

	case 4:
		//Serial.println("4");
		TPS.UpdateValue(mapf(analogRead(TpsPin), 24, 560, 0, 100, true));
		CalculateRPM();
		CalculateTSS();
		break;

	case 5:
		//Serial.println("5");

		break;

	case 6:
		//Serial.println("6");
		UpdateBoostLimitCompensation();
		break;

	case 7:

		UpdateTotalCompensation();
		UpdateAntiSurgeLimit();
		UpdateIdleWalkdownLimit();
		UpdateBOVoutput();

		break;

	case 8:
		UpdateMinimumPosition(); // Update all minimums

		break;

	case 9:
		UpdatePosition();
		break;
	case 10: //keep a predictable number of cycles between resets because of timing related functions
		//Serial.println("10");

		CalculationLoops++;

		break;
	}

	unsigned long endtime = micros();
	unsigned long timespan = endtime - starttime;
	ProgressiveTiming[ProgressiveLoopNum] = max(ProgressiveTiming[ProgressiveLoopNum], timespan);


}

void SelectSerialMode() {
	//called from Timer3 routine
	Aux1 = mapf(analogRead(Aux1Pin), 0, 1023, -1, 1, false);
	Aux2 = mapf(analogRead(Aux2Pin), 0, 1023, -1, 1, false);
	UpdateAuxCompensation();// Moved here since it doesn't need to be updated often
	//
	if (LimitingActive) { //ring the bell if limiting or compensators are quite active
		Serial.println("Limiting \a");
	}
	if (GlobalFloats[AutoCompIndex] > 1.2) {
		Serial.print("Comp \a ");
		Serial.println(GlobalFloats[AutoCompIndex]);
	}

	while (Serial.available() > 0) {
		SerialMode = Serial.read();
	}

	switch (SerialMode)
	{
	case 'c':
		//dive deep into the compensators
		CompensatorSerial();
		break;
	case 'l':
		LimitSerial();
		break;
	case 'm':
		ModeSerial();
		break;
	case 'r':
		RawSerial();
		break;
	case 'a':
		AnalogSerial();
		break;
	case ' ': //pauses serial, restart by reselecting mode
		break;
	case 'p':
		break;
	case 't':
		TimingSerial();
		break;
	case 'f':
		FriendlySerial();
		break;
	case 'n':
		MyVarSerial();
		break;
	default:
		//Display help page, pause other output after
		ShowSerialHelp();
		SerialMode = 'p';
		break;
	}

}

void LimitSerial() {
	Serial.println("");
	Serialprintint("RPM ", RPM.Value);
	Serialprintint("TPS ", TPS.Value);
	Serialprint("TSS ", TSS.Value, 3);
	Serialprint("MAP ", TPS.Value, 1);
	Serialprint("EGP ", EGP.Value, 1);
	Serial.println("");

	if (GlobalFloats[LimitingAmountIndex] >= 0) {
		Serial.println("      Limiting Active      ");
		Serial.print("Limiting Factor is ");
		Serial.println(GetLimitingFactorName());
		Serialprintint("Limiting Amount  ", GlobalFloats[LimitingAmountIndex]);

	}

	Serialprintint("Antisurge Pos ", GlobalFloats[AntisurgeMinimumPositionIndex]);


	Serialprintint("Boost Min Pos ", GlobalFloats[BoostRPMminimumPositionIndex]);
	Serialprintint("RPM Min Pos   ", GlobalFloats[RPMminimumPositionIndex]);
	Serialprintint("TSS Min Pos   ", GlobalFloats[TSSMinimumPositionIndex]);
	Serialprintint("EGP Min Pos   ", GlobalFloats[EGPminimumPositionIndex]);
	Serial.println("");
	Serialprintint("BasePosition         ", GlobalFloats[BasePositionIndex]);
	Serialprintint("Compensated Position ", GlobalFloats[CompensatedPositionIndex]);
	Serialprintint("Limited  Position    ", GlobalFloats[MinimumPositionIndex]);
	Serialprintint("Autobrake Position   ", GlobalFloats[AutobrakeMapPositionIndex]);
	Serialprintint("Send Position        ", GlobalFloats[FinalPositionIndex]);

}
void CompensatorSerial() {

	Serialprint("Aux Comp ", GlobalFloats[AuxCompIndex], 3);
	Serialprint("Aux Low  ", GlobalFloats[AuxCompLowIndex], 3);
	Serialprint("Aux High ", GlobalFloats[AuxCompHighIndex], 3);
	Serial.println("");

	if (GlobalFloats[OverspeedCompIndex] > 1) {
		Serialprint("Overspeed comp ", GlobalFloats[OverspeedCompIndex], 3);
		Serial.println("");
	}


	if (GlobalFloats[BoostCompIndex] > 1) {
		Serialprint("MAP            ", MAP.Value, 1);
		Serialprint("Boost Setpoint ", GlobalFloats[BoostCompSetpointIndex], 1);
		Serialprint("OverboostRatio ", GlobalFloats[BoostCompOverboostRatioIndex], 3);
		Serial.println("");
		Serialprint("Boost P Factor ", GlobalFloats[BoostCompProportionalFactorIndex], 3);
		Serialprint("Boost I Factor ", GlobalFloats[BoostCompIntegralFactorIndex], 3);
		Serialprint("Boost D Factor ", GlobalFloats[BoostCompDifferentialFactorIndex], 3);
		Serialprint("Boost Compensation  ", GlobalFloats[BoostCompIndex], 3);

		Serial.println("");
	}
	/*
	if (GlobalFloats[LimitingAmountIndex] > 0) {
		Serial.print("Limiting Active? ");
		Serial.println((GlobalFloats[LimitingAmountIndex] >= 0 ? "TRUE" : "FALSE"));
		Serial.print("Limiting Factor is ");
		Serial.println(GetLimitingFactorName());
		Serialprintint("Limiting Amount  ", GlobalFloats[LimitingAmountIndex]);

		Serialprintint("Antisurge Pos ", GlobalFloats[AntiSurgeMinimumPositionIndex]);
		Serialprintint("Boost Min Pos ", GlobalFloats[BoostRPMminimumPositionIndex]);
		Serialprintint("RPM Min Pos   ", GlobalFloats[RPMminimumPositionIndex]);
		Serialprintint("TSS Min Pos   ", GlobalFloats[TSSMinimumPositionIndex]);
		Serialprintint("EGP Min Pos   ", GlobalFloats[EGPminimumPositionIndex]);

	Serial.println("");

	}*/
	//else {
	//	Serial.print("Nearest limiter is");
	//	Serial.print(GetLimitingFactorName());
	//	Serial.println(GlobalFloats[LimitingAmountIndex]);
	//}
	//Serialprintint("BasePosition         ", GlobalFloats[BasePositionIndex]);
	//Serialprintint("Compensated Position ", GlobalFloats[CompensatedPositionIndex]);
	//Serialprintint("Limited  Position    ", GlobalFloats[MinimumPositionIndex]);
	//Serialprintint("Autobrake Position   ", GlobalFloats[AutobrakePositionIndex]);
	//Serialprintint("Send Position        ", GlobalFloats[FinalPositionIndex]);
	//Serial.println("");
}
void FriendlySerial() {
	Serialprintint("Map is ", CurrentMap);
	Serialprintint("   Mode is ", CurrentMode);

	Serialprintint("RPM = ", RPM.Value);
	Serialprint("TSS = ", TSS.Value, 3);

	Serial.print("TPS = ");
	Serial.print(TPS.Value);
	Serial.print("\t TPSraw = ");
	Serial.println(analogRead(TpsPin));


	Serialprintint("MAP   ", MAP.Value);
	Serialprintint("EGP  ", EGP.Value);
	Serialprint("Fuel  ", FPS.Value, 1);

	Serial.print("Aux Compensation = ");
	Serial.println(GlobalFloats[AuxCompIndex]);

	if (GlobalFloats[BoostCompIndex] > 1) {
		Serialprint("Boost Comp = ", GlobalFloats[BoostCompIndex], 3);
		Serialprint("P term     = ", GlobalFloats[BoostCompProportionalFactorIndex], 3);
		Serialprint("I term     = ", GlobalFloats[BoostCompIntegralFactorIndex], 3);
	}
	if (GlobalFloats[OverspeedCompIndex] > 1) {
		Serial.print("OverspeedComp = ");
		Serial.println((GlobalFloats[OverspeedCompIndex], 2));
	}
	if (GlobalFloats[LimitingAmountIndex] > 0) {  //If there's something limiting the minimum position, lets find out what it is and how much it's limiting
		//Serial.print(GetLimitingFactorName());
		Serial.print("\a");
		Serialprintint("Minimum pos ", GlobalFloats[MinimumPositionIndex]);
		Serialprintint("Limiting by ", GlobalFloats[LimitingAmountIndex]);
		Serial.print("Limiting Factor ");
		Serial.println(GetLimitingFactorName());
	}

	Serial.println("");

	Serialprintint("Base Position        ", GlobalFloats[BasePositionIndex]);
	Serialprintint("Compensated Position ", GlobalFloats[CompensatedPositionIndex]);
	Serialprintint("Limited  Position    ", GlobalFloats[MinimumPositionIndex]);
	Serialprintint("Autobrake Position   ", GlobalFloats[AutobrakedPositionIndex]);
	Serialprintint("Final Position       ", GlobalFloats[FinalPositionIndex]);


	Serial.print("Command / Real Position = ");
	Serial.print(VgtCommandPosition);
	Serial.print(" / ");
	Serial.println(VgtRealPosition);
	Serialprintint("Vgt speed = ", VgtMotorCommandSpeed);
	Serialprintint("Vgt raw temp ", VgtRawTemp);

	//Serial.print("CAN messages per second = ");
	//Serial.println(CanMessagesReceived / float(LogSpeed) * 1000);
	//Serial.print("Loopcounts per second =   ");
	//Serial.println(loopcounts / LogSpeed * 1000);

	Serial.println("\n\n");
}
void RawSerial() {
	Serial.println("");
	Serialprint("MAP", analogRead(MapPin), 0);
	Serialprint("TPS", analogRead(TpsPin), 0);
	Serialprint("EGP", analogRead(EgpPin), 0);
	Serialprint("Aux1", analogRead(Aux1Pin), 0);
	Serialprint("Aux2", analogRead(Aux2Pin), 0);
	Serialprint("FPS", analogRead(FpsPin), 0);
	Serialprint("Brake", analogRead(BrakePin), 0);
	Serialprint("Switch1Status", Switch1Status, 0);
	Serialprint("Switch2Status", Switch2Status, 0);
	Serialprint("VGT pos", SendPosition.Value, 0);

}
void AnalogSerial() {
	static int A[16];
	int B[16];
	for (int j = 0; j < 16; j++) {
		B[j] = analogRead(A0 + j);
	}
	Serial.println("");

	for (int i = 0; i < 16; i++) {
		Serial.print("A");
		Serial.print(i);
		Serial.print(" = ");
		Serial.print(B[i]);
		Serial.print(" \t");
		Serial.println(B[i] - A[i]);
	}


	for (int i = 0; i < 16; i++) {
		A[i] = B[i];
	}

}
void ModeSerial() {
	if (Running) { Serialprintint("Running time ", RunningTime / 1000); }
	if (Idling) { Serialprintint("Idling time ", IdlingTime / 1000); }
	if (JakeBoostMode) { Serial.println("Jake BOOST mode"); }
	if (JakeBrakeMode) { Serial.println("Jake BRAKE mode"); }



}
void TimingSerial() {
	//diagnoses program timing

	Serial.println("\n ProgressiveController timings (microseconds)");

	Serialprintint("Cacluation loops ", CalculationLoops);
	CalculationLoops = 0;
	long int TimeSum = 0;

	for (int i = 0; i < 10; i++) {
		Serial.print(i);
		Serial.print(" ");
		Serial.println(ProgressiveTiming[i]);
		TimeSum += ProgressiveTiming[i];
		ProgressiveTiming[i] = 0;
	}
	Serialprintint("Total Time ", TimeSum);
}
void MyVarSerial() {
	Serial.println("");
	Serial.println(TSS.ToString(3));
	Serial.println(RPM.ToString(0));
	Serial.println(MAP.ToString(1));
	Serial.println(TPS.ToString(1));
}
void ShowSerialHelp() {
	Serial.println(" 'p' to pause output");
	Serial.println(" 'a' for all raw analog input values");
	Serial.println(" 'r' for raw inputs ");
	Serial.println(" 'c' for compensator modifiers");
	Serial.println(" 'l' for limit modifiers");

}

void ReadCanMessage() {
	if (CAN1.msgAvailable()) {                      // Check to see if a valid message has been received.
		CanMessagesReceived++;

		CAN msg;
		CAN1.read(&msg);

		switch (msg.ID)
		{
		case StatusMessageID:
			VgtCommandPosition = ((msg.data[6] << 8) | msg.data[5]);
			VgtRealPosition = ((msg.data[2] << 8) | msg.data[1]);
			VgtMotorCommandSpeed = msg.data[7] - 127;
			VgtRawTemp = msg.data[3];
			break;
		case ErrorMessageID:
			PrintCANmessage(msg);
			break;
		default:
			break;
		}
	}
}
void FormatForVGT(int Position) {
	Position = constrain(Position, VgtMinPosition, VgtMaxPosition);
	SendToVGT(Position);
}
void SendToVGT(int Position) {
	//THIS SHOULD ONLY BE CALLED FROM "FormatForVGT" TO ENSURE limits
	//Serial.print("Sending ");
	//Serial.println(Position);
	byte lo_byte = lowByte(Position);
	byte hi_byte = highByte(Position);
	//SendVgtRecalibrate();

	byte data[] = { lo_byte, hi_byte, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // data message with an added counter
	//byte data[2] = {0x02, 0x00}; //for recalibrating gearbox
	// Load message and send
	CAN1.send(VGTaddress, extID, 8, data);
}
void SendVgtRecalibrate() {

	byte data[4] = { 0x00,0x00, 0x02, 0xFF, }; //for recalibrating gearbox

	CAN1.send(VGTaddress, extID, 8, data);
	//delay(10000);
}
String GetLimitingFactorName() {
	int minval = 1000;

	minval = min(minval, GlobalFloats[BoostRPMminimumPositionIndex]);
	minval = min(minval, GlobalFloats[AntisurgeMinimumPositionIndex]);
	minval = min(minval, GlobalFloats[TSSMinimumPositionIndex]);
	minval = min(minval, GlobalFloats[RPMminimumPositionIndex]);
	minval = min(minval, GlobalFloats[EGPminimumPositionIndex]);
	if (minval >= 960) {
		return " none ";
	}
	if (minval == int(GlobalFloats[AntisurgeMinimumPositionIndex])) {
		return (" Antisurge ");
	}
	if (minval == int(GlobalFloats[BoostRPMminimumPositionIndex])) {
		return (" BOOST ");
	}
	if (minval == int(GlobalFloats[TSSMinimumPositionIndex])) {
		return (" TSS");
	}
	if (minval == int(GlobalFloats[RPMminimumPositionIndex])) {
		return (" RPM ");
	}
	if (minval == int(GlobalFloats[EGPminimumPositionIndex])) {
		return (" EGP ");
	}
	if (minval == int(GlobalFloats[IdleWalkdownMinimumPositionIndex])) {
		return " Idle ";
	}

	return (" Unknown Limit "); //if we've added something else and haven't defined it here
}
int GetLimitingFactorIndex() {
	int minval = 1000;

	minval = min(minval, GlobalFloats[BoostRPMminimumPositionIndex]);
	minval = min(minval, GlobalFloats[AntisurgeMinimumPositionIndex]);
	minval = min(minval, GlobalFloats[TSSMinimumPositionIndex]);
	minval = min(minval, GlobalFloats[RPMminimumPositionIndex]);
	minval = min(minval, GlobalFloats[EGPminimumPositionIndex]);
	if (minval >= 960) {
		return 0;
	}
	if (minval == int(GlobalFloats[AntisurgeMinimumPositionIndex])) {
		return AntisurgeMinimumPositionIndex;
	}
	if (minval == int(GlobalFloats[BoostRPMminimumPositionIndex])) {
		return BoostRPMminimumPositionIndex;
	}
	if (minval == int(GlobalFloats[TSSMinimumPositionIndex])) {
		return TSSMinimumPositionIndex;
	}
	if (minval == int(GlobalFloats[RPMminimumPositionIndex])) {
		return RPMminimumPositionIndex;
	}
	if (minval == int(GlobalFloats[EGPminimumPositionIndex])) {
		return EGPminimumPositionIndex;
	}
	if (minval == int(GlobalFloats[IdleWalkdownMinimumPositionIndex])) {
		return IdleWalkdownMinimumPositionIndex;
	}

	return IndexCount; //if we've added something else and haven't defined it here
}


