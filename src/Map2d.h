/* Created May 25, 2015 by Efrem Hug, rx7man@gmail.com
Feel free to use it in your projects, not for commercial use.
USE AT OWN RISK*/
#include <EEPROM.h>
#ifndef Map2d_h
#define Map2d_h

#include "Arduino.h"


class Map2d
{
  public:
    Map2d(byte rows, byte columns);
    Map2d(int EEPROMaddress);

    byte Rows;
    byte Columns;

    byte* mapData ;
    int* rowData;
    int* columnData;
    int cellCount;

    float Interpolate(int, int);
    void Debug();
    void DebugMapIndices();
    int SaveToEEPROM(int);
    int ReadFromEEPROM(int);

  private:
    byte GetMapValue(byte, byte);
    byte GetMapIndex(byte, byte);
    byte GetColumnLabelIndex(int);
    byte GetRowLabelIndex(int);
    byte* GetFourCornersValues(byte, byte);
    int* GetFourCornersLabels(byte, byte);
    float Remap(float, float, float, float, float);
    float Remap2d(int, int, byte*, int*);
    //int SaveArrayToEEPROM(int, byte*, int);
    //int SaveArrayToEEPROM(int, int*, int);
    //int ReadArrayFromEEPROM(int, byte*, int);
    //int ReadArrayFromEEPROM(int, int*, int);
};
#endif


