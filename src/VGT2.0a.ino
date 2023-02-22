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
#include "MyDataTypes.h"
#include "MyAnalogSensor.h"
#include "VGTstuff.h"
#include "AnalogSensorCalibration.h"
#include "Map1d.h"
#include "TuneClass.h"
#include "TestClass.h"

//For now I'll just change the tune map here, but eventually each tune will have just one map, and different tunes will be enabled
//If there are things that don't change between tunes, change them in tuneclass.h, then only change the variations here (or in initializemaps())
//-----------------  0    20   40   60   80   100  120  140 Turbine RPM
tristateswitch CurrentTune;
TuneClass* Tune;
int TuneMapUp[8] = { 90,  95,  90,  80,  70,  60,  30,  0 };
int TuneMapMid[8] = { 90,  90,  85,  75,  65,  55,  20,  0 };
int TuneMapDown[8] = { 90,  90,  80,  70,  60,  50,  20,  0 };



//#define baudrate 115200
//#define baudrate 115200
#define baudrate 921600

MCP CAN1(CanControllerPin); //define CAN bus
int VgtMaxPosition = 1000; //maximum value to command VGT to
int VgtMinPosition = 0;    //minimum value to command VGT to

int CanMessagesReceived = 0;          //number of CAN messages recieved since last serial display update

//VGT return values
int VgtRealPosition;         //The position the VGT is actually at (read from the CAN messages)
int VgtCommandPosition;      //The position the VGT target
int VgtMotorCommandSpeed;    //The speed and direction the vane motor is set to
int VgtRawTemp;              //Temperature of the VGT control unit... should be pretty close to coolant temp in most situations

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
boolean AutobrakeActive = false;     //Autobrake mode is actually doing something
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



MyVar RPM; //Engine RPM
MyVar TSS; //Turbine shaft speed
MyVar TPS; //Throttle position
MyVar MAP; //Manifold pressure
MyVar EGP; //Exhaust pressure
MyVar BPS; //brake pressure sensor
MyVar FPS; //Fuel pressure sensor

MyVar SendPosition;            //The value we're sending to the VGT



enum GlobalFloatIndexes :byte {
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


int MinRawTPSposition = 25; // this automatically gets reduced to the minimum TPS raw position
int MaxRawTPSposition = 500; // this needs to be manually adjusted to WOT




#pragma region Setup Stuff

void setup() {

	// put your setup code here, to run once:
	Serial.begin(baudrate);
	Serial.println("Welcome to BigBlackBox V1.7e");
	Serial.println("Apply jake switch and hold thumbswitch while powering up to start a VGT recalibrate");

	InitializePins();
	InitializeMyVars();
	InitializeTunes();
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
void InitializeTunes() {
	Serial.println("Initializing Tunes");
	Tune[up].LilbbMap.MapData = TuneMapUp;
	Tune[middle].LilbbMap.MapData = TuneMapMid;
	Tune[down].LilbbMap.MapData = TuneMapDown;


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
		SendPosition = Tune[CurrentTune].VGTstartPosition; //if not running, set to static position and don't apply jake, etc
	}

	FormatForVGT(SendPosition);

}

void Timer3Routine() {// Selects serial mode  
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
	CurrentMap = Switch1Status;

	
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
	JakeBrakeMode = (JakeSwitch && (TPS.Value < Tune[CurrentTune].JakeBrakeTPSthreshold));
	JakeBoostMode = (JakeSwitch && (TPS.Value > Tune[CurrentTune].JakeBrakeTPSthreshold));


	static unsigned long LastUpdateTime;
	float Timespan;  //timespan in SECONDS from last pass through

	if (LastUpdateTime == 0) { //lets not do math with garbage, should only occur the first pass through

		LastUpdateTime = millis();
	}
	else {
		unsigned long currentmillis = millis();
		Timespan = float(currentmillis - LastUpdateTime) / 1000.0f;
		LastUpdateTime = currentmillis;
	}

	if (RPM.Value >= Tune[CurrentTune].MinRunRPM) {
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

	if (Running && RPM.Value <= Tune[CurrentTune].MaxIdleRPM && TPS.Value <= Tune[CurrentTune].MaxIdleTPS) {
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


	int AutobrakeMapPosition = Tune[CurrentTune].AutobrakeMap.Interpolate(RPM.Value);
	GlobalFloats[AutobrakeMapPositionIndex] = AutobrakeMapPosition;
	float AutobrakedPosition = mapf(TPS.Value, 1, 5, AutobrakeMapPosition, LimitedPosition, true);

	float finalposition = min(AutobrakedPosition, GlobalFloats[AntisurgeMinimumPositionIndex]);  //I have an issue somewhere here in the logic
	//float finalposition = min(LimitedPosition, GlobalFloats[AntisurgeMinimumPositionIndex]);

	GlobalFloats[FinalPositionIndex] = finalposition; // 960->0 constraints are applied in Timer1 routine before sending, along with Jakebrake
}


void UpdateBasePosition() {
	//Called from UpdatePosition()
	int BasePosition = Tune[CurrentTune].LilbbMap.Interpolate(TSS.Value);
	GlobalFloats[BasePositionIndex] = BasePosition;  //Set this to start with and then any unhandled part following will be correct
	//Serialprintint("Base position ", BasePosition);
	GlobalFloats[BaseNozzleSizeIndex] = GetNozzleSizeFromVGTposition(BasePosition);
}

void UpdateMinimumPosition() {
	// This is where you find all the minimum positions and put them into one variable
	//Called from ProgressiveController

	GlobalFloats[RPMminimumPositionIndex] = Tune[CurrentTune].RPMminimumPositionMap.Interpolate(RPM.Value);
	GlobalFloats[TSSMinimumPositionIndex] = Tune[CurrentTune].TSSminimumPositionMap.Interpolate(TSS.Value);
	GlobalFloats[BoostRPMminimumPositionIndex] = Tune[CurrentTune].BoostMinimumPositionMap.Interpolate(MAP.Value);
	GlobalFloats[EGPminimumPositionIndex] = Tune[CurrentTune].EGPminimumPositionMap.Interpolate(0);//EGP.value if enabled

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
	if (JakeBoostMode) { TotalCompensationValue *= Tune[CurrentTune].JakeBoostFactor; }


	GlobalFloats[TotalCompensationIndex] = constrain(TotalCompensationValue, Tune[CurrentTune].MinTotalCompensation, Tune[CurrentTune].MaxTotalCompensation);
}


void UpdateAntiSurgeLimit() {
	//POSITIVE value opens vanes more, but converted to be an absolute position for output
	//Called from ProgressiveController
	static float Carryover;
	//The faster you let off the throttle, and the faster the turbine is going
	//the more chance of surge, so force nozzle open more aggressively
	float Antisurge = 0;
	if (TPS.Slope < -Tune[CurrentTune].AntisurgeMinimumTPSslope) {
		Antisurge = max(Antisurge, -min(TPS.Slope, 100) * Tune[CurrentTune].AntisurgeResponseRate * TSS.Value);
	}

	Antisurge = constrain(Antisurge, 0, Tune[CurrentTune].AntisurgeMaxVal);  //Limit to maximum value
	Antisurge = max(Antisurge, Carryover);        //If the carryover is larger than the current antisurge amount, use the carryover for the math
	Carryover = Antisurge * Tune[CurrentTune].AntisurgeFalloffRate; //Apply falloff to the carryover


	if (Carryover <= 100 || (TPS.Slope >= 5 && TPS.Value >= 10 || TSS.Value < Tune[CurrentTune].AntisurgeTSScancelSpeed)) {//Just zero it out if it's smaller than the minimum or getting back on the throttle, or shaft speed drops enough
		Carryover = 0;
		Antisurge = 0;
	}
	GlobalFloats[AntisurgeCarryoverIndex] = Carryover;
	GlobalFloats[AntisurgeMinimumPositionIndex] = constrain(1000 - Antisurge, 0, 960); //apply a minimum so that it can't go below zero, and reverse the numerical value

}

void UpdateIdleWalkdownLimit() {
	//Called from ProgressiveController
	static float IdleWalkdownLimitPosition = 960;

	if (IdlingTime > Tune[CurrentTune].IdleWalkdownLimitDelay && VgtRawTemp > Tune[CurrentTune].IdleWalkdownLimitMinTemp) {
		IdleWalkdownLimitPosition = constrain(IdleWalkdownLimitPosition - Tune[CurrentTune].IdleWalkdownLimitSpeed, Tune[CurrentTune].IdleWalkdownLimitMaxPos, 960);
	}
	else {
		IdleWalkdownLimitPosition = 960;
	}
	GlobalFloats[IdleWalkdownMinimumPositionIndex] = IdleWalkdownLimitPosition;
}
void UpdateBOVoutput() {
	AntisurgeActive = GlobalFloats[AntisurgeMinimumPositionIndex] <= Tune[CurrentTune].BOVantisurgeThreshold;
	digitalWrite(BOVsolenoidPin, AntisurgeActive);
}
void UpdateOverspeedCompensation() {
	//returns a POSITIVE value greater than 1 when limiting is active and 1.00 when inactive
	float OverspeedCompensationValue = 1;
	if (TSS.Value > Tune[CurrentTune].TurbineOverspeedStartLimitRPM) {
		OverspeedCompensationValue =
			mapf(TSS.Value, Tune[CurrentTune].TurbineOverspeedStartLimitRPM, Tune[CurrentTune].TurbineOverspeedStartLimitRPM + Tune[CurrentTune].TurbineOverspeedLimitingWidth, 1, Tune[CurrentTune].MaxTSSspeedCompensation, true);
	}
	GlobalFloats[OverspeedCompIndex] = OverspeedCompensationValue;
}
void UpdateAuxCompensation() {
	//called from SelectSerialMode since it doesn't need to be updated often

	//this sets the compensation so that it gradually goes from using the left pot for low RPM to the right pot for high RPM


	float LowCompensationRate = GetGenericCompensationExp(Aux1, .5, 2);
	float HighCompensationRate = GetGenericCompensationExp(Aux2, .5, 2);
	float LowCompensationValue = mapf(TSS.Value, Tune[CurrentTune].AuxCompTSSLowPoint, Tune[CurrentTune].AuxCompTSSHighPoint, 1, 0, true) * LowCompensationRate;
	float HighCompensationValue = mapf(TSS.Value, Tune[CurrentTune].AuxCompTSSLowPoint, Tune[CurrentTune].AuxCompTSSHighPoint, 0, 1, true) * HighCompensationRate;
	GlobalFloats[AuxCompIndex] = LowCompensationValue + HighCompensationValue;
	//Serialprint("Lowcomp rate   ", LowCompensationRate, 3);
	//Serialprint("Low comp value ", LowCompensationValue,3);

}
void UpdateBoostLimitCompensation() {
	//provides a value > 1 when limiting is active to increase nozzle size
	//Internally it works with values greater than zero
	static float MaxIntegralAccumulator = (Tune[CurrentTune].BoostLimitIntegralMaxComp - 1) * Tune[CurrentTune].BoostLimitIntegralRate; //
	static float IntegralAccumulator = 0;

	float Setpoint = Tune[CurrentTune].BoostRPMMap.Interpolate(RPM.Value);  //Get max allowed boost from map
	float MAPvalue = MAP.Value;


	float OverboostDifference = max(0, MAPvalue - Setpoint);
	float OverboostRatio = MAPvalue / max(Setpoint, 1);  //max there to prevent divide by zero error, Setpoint ought to be > 5 anyhow

	float ProportionalFactor = 1;

	if (OverboostRatio > 1) {
		ProportionalFactor = ((OverboostRatio - 1) * Tune[CurrentTune].BoostLimitProportionalRate) + 1; //May want to switch this to absolute rather than ratiometric

		IntegralAccumulator += OverboostDifference;
		IntegralAccumulator *= Tune[CurrentTune].BoostLimitIntegralFalloffRate;

	}
	else {
		// If not limiting boost, cancel out any remaining integral to get a fresh start next time
		IntegralAccumulator = 0;
	}


	float IntegralFactor = 1 + (IntegralAccumulator * Tune[CurrentTune].BoostLimitIntegralRate);
	if (IntegralFactor > Tune[CurrentTune].BoostLimitIntegralMaxComp) { //limits integral amount
		IntegralAccumulator = MaxIntegralAccumulator;  //max is set at top of function once when program starts
		IntegralFactor = Tune[CurrentTune].BoostLimitIntegralMaxComp;
	}

	float TempDiffFactor = MAP.Slope * Tune[CurrentTune].BoostLimitDifferentialRate + 1; //Get the current slope and multiply by the rate
	float TempDiffFactor2 = mapf(OverboostRatio, Tune[CurrentTune].BoostLimitDifferentialOverboostRatio, 1, 1, TempDiffFactor, true); //slowly bring it in from a percentage of the underboost
	float DifferentialFactor = constrain(TempDiffFactor2, 1 / Tune[CurrentTune].BoostLimitDifferentialMaxComp, Tune[CurrentTune].BoostLimitDifferentialMaxComp); //constrain



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
	Serial.print("Size of tune ");
	Serial.println(sizeof(Tune));

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
	Serialprintint("Tune size ", sizeof(Tune));
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

	}
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
	*/
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
