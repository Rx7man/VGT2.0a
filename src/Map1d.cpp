/* 
* Map1d.cpp
*
* Created: 1/8/2016 9:15:59 PM
* Author: Y2D
This remaps a value, useful for all sorts of stuff.. like reading a non-linear value (approximates linearly between points).. 
With carefully chosen divisions the approximation is fairly accurate (chose more division in non-linear areas)
If the X value given is out of the bounds of your index data division points, it will extrapolate based on the slope of the nearest 2 cells

It currently works with integers, but could be made to work with smaller or larger data sizes depending on memory constraints

Some uses are to map:
analog voltage from a resistance temperature sensor to a temperature
non-linear PWM duty based on a linear input
Anything where you want a custom response rate
*/

#include "Map1d.h"

//int CellCount;   // how many cells we have
//int* Values;   // values
//int* Divisions;  // Cell index values aka division points


//constructor
Map1d::Map1d(int cellcount){
	MapData = new int[cellcount];
	Divisions = new int[cellcount];
	CellCount = cellcount;
	Scaling = 1;
	Offset = 0;
}

int Map1d::GetCellCount() {
	return CellCount;
}

float Map1d::Interpolate(int Xvalue){
	if (CellCount < 2) {return Xvalue;} // We can't determine a slope with 1 point
	if (Xvalue < Divisions[0]) { return Divisions[0]; }
	if (Xvalue > Divisions[CellCount - 1]) { return Divisions[CellCount - 1]; }
	int LowIndex = GetLowerCellIndex(Xvalue);
	int LowDivision = Divisions[LowIndex];
	int HighDivision = Divisions[LowIndex+1];
	int LowCellValue = MapData[LowIndex];
	int HighCellValue = MapData[LowIndex+1];
	
	return (mapf(Xvalue,LowDivision, HighDivision, LowCellValue, HighCellValue)+ Offset)*Scaling;
}

int Map1d::GetLowerCellIndex(int Xvalue){
	for(int i = 0; i < CellCount-1; i++){
		if (Divisions[i+1] > Xvalue){
			return i;
		}
	}
	//return CellCount-1;
}

float Map1d::mapf(float value, float oldlow, float oldhigh, float newlow, float newhigh) {
	//I think this is identical to the arduino map(), but I had to write it for VB, so I included it here
	float rval = 0;
	if ((oldhigh - oldlow) != 0) {
		rval = float(value - oldlow) / float(oldhigh - oldlow) * float(newhigh - newlow) + newlow;
	}//if
	else {
		rval = value;
	}//else
	return rval;
} //Remap