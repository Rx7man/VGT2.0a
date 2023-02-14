#pragma once
#ifndef VGTstuff_h
#define VGTstuff_h

int VgtMaxPosition = 1000; //maximum value to command VGT to
int VgtMinPosition = 0;    //minimum value to command VGT to

int CanMessagesReceived = 0;          //number of CAN messages recieved since last serial display update

									  //VGT return values
int VgtRealPosition;         //The position the VGT is actually at (read from the CAN messages)
int VgtCommandPosition;      //The position the VGT target
int VgtMotorCommandSpeed;    //The speed and direction the vane motor is set to
int VgtRawTemp;              //Temperature of the VGT control unit... should be pretty close to coolant temp in most situations


float GetNozzleSizeFromVGTposition(int VGTposition) {
	//returns the nozzle size in Cm2 * 10
	/* Turbo Size (cm^2) vs Position
	*   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23   24  25
	*  960 918 876 835 793 751 709 667 625 584 542 500 458 416 375 333 291 249 207 165 124  82  40  */
	return mapf(VGTposition, 40, 960, 25, 3, false);
}
int GetVGTpositionFromNozzleSize(float NozzleSize) {
	return map(NozzleSize, 25, 3, 40, 960); //using integer math on this for speed
}
void PrintCANmessage(CAN msg) {
	Serial.print("CanMessage = ID");
	Serial.print(" | 0x");
	Serial.print(msg.ID, HEX);                                 // Displays received ID
	Serial.print(" | ");
	Serial.print("Data Length DEC");
	Serial.print(" | ");
	Serial.print(msg.length);                               // Displays message length
	Serial.print(" | ");
	Serial.print("Data");
	for (byte i = 0; i < msg.length; i++) {
		Serial.print(" | ");
		if (msg.data[i] < 0x10)                                   // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
		{
			Serial.print("0");
		}
		Serial.print(msg.data[i], HEX);                          // Displays message data
	}
	Serial.println();
}
struct Caninfo {
public:
	int8_t MotorSpeed;
	uint16_t CommandPos;
	uint16_t RealPos;
	uint8_t RawTemp;

	void Update(CAN &msg) {
		if (msg.ID == StatusMessageID) {
			CommandPos = ((msg.data[6] << 8) | msg.data[5]);
			RealPos = ((msg.data[2] << 8) | msg.data[1]);
			MotorSpeed = msg.data[7] - 127;
			RawTemp = msg.data[3];
		}
	}
	void Print() {
		Serial.print("\nCommand = ");
		Serial.print(CommandPos);
		Serial.print("\t Real = ");
		Serial.print(RealPos);
		Serial.print("\t  Speed = ");
		Serial.print(MotorSpeed);
		Serial.print("\t Raw Temp = ");
		Serial.println(RawTemp);
	}
};

#endif // !VGTstuff_h