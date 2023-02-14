#pragma once
#ifndef MyDataTypes_h
#define MyDataTypes_h


union twobyte {
	byte b[2];
	unsigned int uint16;
	int int16;
};
union fourbyte {
	byte b[4];
	float f;
	uint32_t uint32;
	int32_t int32;
};
union eightbyte {
	byte b[8];
	uint64_t uint64;
};

enum tristateswitch {undefined = -100, down = -1, middle = 0, up = 1};

#endif // !MyDataTypes_h

