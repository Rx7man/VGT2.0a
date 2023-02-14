/* 
* Map1d.h
*
* Created: 1/8/2016 9:16:00 PM
* Author: Y2D
*/
#include "Arduino.h"

#ifndef __MAP1D_H__
#define __MAP1D_H__

class Map1d
{
//variables
public:
Map1d(int CellCount);

int* MapData ; // values 
int* Divisions;  // Cell index values aka division points
float Offset = 0; //applied to the interpolation
float Scaling = 1; //applied to the interpolation
float Interpolate(int);
int GetCellCount();

private:
int CellCount;   // how many cells we have
int GetLowerCellIndex(int);
float mapf(float, float, float, float, float);

}; //Map1d

#endif //__MAP1D_H__
