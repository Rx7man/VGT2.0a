/* Created May 25, 2015 by Efrem Hug, rx7man@gmail.com
Feel free to use it in your projects, not for commercial use.
USE AT OWN RISK*/

#include "Arduino.h"
#include "Map2d.h"
#include <EEPROM.h>


byte* mapData ; //the actual map data
int* rowData; //labels for rows
int* columnData; //labels for columns
byte cellCount; //Number of cells

byte Rows;
byte Columns;

//Need some enumerators to remember what parts of which arrays correspond to what!!
enum FCV {XlowYlow = 0, XlowYhigh = 1, XhighYlow = 2, XhighYhigh = 3}; //Enumerator for FourCornersVALUES
enum FCL {ColumnLow = 0, ColumnHigh = 1, RowLow = 2, RowHigh = 3 }; //Enumerator for FourCornersLABELS

Map2d::Map2d(byte _rows, byte _columns) {
  Rows = _rows;
  Columns = _columns;

  rowData = new int[Rows];
  columnData = new int[Columns];

  int size = Rows * Columns;
  mapData = new  byte[size];
  cellCount = size;
}
Map2d::Map2d(int EEPROMaddress) {
  ReadFromEEPROM(EEPROMaddress);
}

//End of the basics for a 2d map backed by a 1D array.. The following is for cell interpolation


float Map2d::Interpolate(int Xvalue, int Yvalue) {
  //this is the public function that gets called when you want to interpolate to get the value
  //needed for your current RPM/TPS
  byte ColumnIndex = GetColumnLabelIndex(Xvalue);
  byte RowIndex =    GetRowLabelIndex(Yvalue);
  byte* FourCornersValues = GetFourCornersValues(RowIndex, ColumnIndex);
  int* FourCornersLabels = GetFourCornersLabels(RowIndex, ColumnIndex);

  float Outval = Remap2d(Xvalue, Yvalue, FourCornersValues, FourCornersLabels);
  return Outval;
}//Interpolate

byte Map2d::GetRowLabelIndex(int Value) {
  //You start from Rows-2, you subtract one because Rows is the count, not the upper bound, and another
  // because you don't want to stay one below (I use the number returned from here, and add one to it
  //when I get the 'four corners', I have to stay within bounds, and this is one way of doing it.
  //You keep checking to see if Value is greater than the contents of the array at index(i), when it is
  //you've found the row just below your value for the 'four corners'
  for (byte i = Rows - 2; i > 0; i--) {
    if (Value >= rowData[i]) {
      return i;
    }//if
  }//for
  return 0;
}//GetRowLabelIndex

byte Map2d::GetColumnLabelIndex(int Value) {
  //See comments above
  for (byte i = Columns - 2; i > 0; i--) {
    if (Value >= columnData[i]) {
      return i;
    }//if
  }//for
  return 0;
}//GetColumnLabelIndex

byte* Map2d::GetFourCornersValues(byte RowIndex, byte ColumnIndex) {
  //This gets the map values which are on the 4 sides of your current RPM/TPS in your map
  static byte Outbytes[4]; //MUST BE STATIC
  Outbytes[XlowYlow] = GetMapValue(RowIndex, ColumnIndex);
  Outbytes[XlowYhigh] = GetMapValue(RowIndex + 1, ColumnIndex);
  Outbytes[XhighYlow] = GetMapValue(RowIndex, ColumnIndex + 1);
  Outbytes[XhighYhigh] = GetMapValue(RowIndex + 1, ColumnIndex + 1);
  return Outbytes;
}//GetFourCornersValues


int* Map2d::GetFourCornersLabels(byte RowIndex, byte ColumnIndex) {
  //This gets the row and column labels corresponding to your current RPM/TPS
  static int Outints[4]; //MUST BE STATIC
  Outints[ColumnLow] = columnData[ColumnIndex];
  Outints[ColumnHigh] = columnData[ColumnIndex + 1];
  Outints[RowLow] = rowData[RowIndex];
  Outints[RowHigh] = rowData[RowIndex + 1];

  return Outints;
}//GetFourCornersLabels


float Map2d::Remap(float value, float oldlow, float oldhigh, float newlow, float newhigh) {
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


float Map2d::Remap2d(int Xvalue, int Yvalue, byte *FourCornersValues, int *FourCornersLabels) {
  //This is what really does the interpolation...

  //First you find it between the bottom edges of the four corners (X = low, Y low to high)
  //enum FCL{RowLow = 0, RowHigh = 1, ColumnLow = 2, ColumnHigh = 3};
  //enum FCV{XlowYlow = 0, XlowYhigh = 1, XhighYlow = 2, XhighYhigh = 3};

  float a = Remap(Xvalue, FourCornersLabels[ColumnLow], FourCornersLabels[ColumnHigh], FourCornersValues[XlowYlow], FourCornersValues[XhighYlow]);

  //then along the top edges (X = high, y=low to high)
  float b = Remap(Xvalue, FourCornersLabels[ColumnLow], FourCornersLabels[ColumnHigh], FourCornersValues[XlowYhigh], FourCornersValues[XhighYhigh]);

  //Finally you find the mapped value... X=low to high, Y = a to b)
  float c = Remap(Yvalue, FourCornersLabels[RowLow], FourCornersLabels[RowHigh], a, b);

  return c;
}

byte Map2d::GetMapValue(byte Row, byte Column) {
  //returns the value of the map at a given row/column
  return mapData[GetMapIndex(Row, Column)];
} //GetMapValue
byte Map2d::GetMapIndex(byte Row, byte Column) {
  //returns the index for the 1D map, given the row/colum
  return Row * Columns + Column;
}//GetMapIndex

//
//int Map2d::SaveToEEPROM(int StartAddress) {
  ////Saves rowcount, columncount, rowdata, columndata, and mapdata to EEPROM starting at BaseAddress
  ////Returns the next available address
  //int WriteAddress = StartAddress;
  //EEPROM.put(WriteAddress, int(0)); //total length of data for this.. to be figured out later
  //WriteAddress += sizeof(int);
  //EEPROM.put(WriteAddress, Rows);
  //WriteAddress += sizeof(Rows);
  //EEPROM.put(WriteAddress, Columns);
  //WriteAddress += sizeof(Columns);
//
  //WriteAddress = SaveArrayToEEPROM(WriteAddress, rowData, Rows);
  //WriteAddress = SaveArrayToEEPROM(WriteAddress, columnData, Columns);
  //WriteAddress = SaveArrayToEEPROM(WriteAddress, mapData, cellCount);
//
  //return WriteAddress;
//}
//int Map2d::ReadFromEEPROM(int StartAddress) {
  //int ReadAddress = StartAddress;
  //int TotalLength;
  //EEPROM.get(ReadAddress, TotalLength);
  //ReadAddress += sizeof(TotalLength);
  //EEPROM.get(ReadAddress, Rows);
  //ReadAddress += sizeof(Rows);
  //EEPROM.get(ReadAddress, Columns);
  //ReadAddress += sizeof(Columns);
//
//
  //cellCount = Rows * Columns;
//
  //static int *newRowData = (int*)calloc(sizeof(int), Rows);
  //static int *newColumnData = (int*)calloc(sizeof(int), Columns);
  //static byte *newMapData = (byte*)calloc(sizeof(int), cellCount);
//
//
  //ReadAddress = ReadArrayFromEEPROM(ReadAddress, newRowData, Rows);
  //ReadAddress = ReadArrayFromEEPROM(ReadAddress, newColumnData, Columns);
  //ReadAddress = ReadArrayFromEEPROM(ReadAddress, newMapData, cellCount);
//
  //rowData = newRowData;
  //columnData = newColumnData;
  //mapData = newMapData;
//
  //return ReadAddress;
//}
//int Map2d::SaveArrayToEEPROM(int StartAddress, byte* Array, int ArraySize) {
  //int WriteAddress = StartAddress;
  //int Size = sizeof(byte);
  //for (int i = 0; i < ArraySize; i++) {
    //EEPROM.put(WriteAddress, Array[i]);
    //WriteAddress += Size;
  //}
  //return WriteAddress;
//}
//int Map2d::ReadArrayFromEEPROM(int StartAddress, byte* Array, int ArraySize) {
  //int ReadAddress = StartAddress;
  //int Size = sizeof(byte);
  //for (int i = 0; i < ArraySize; i++) {
    //EEPROM.get(ReadAddress, Array[i]);
    //ReadAddress += Size;
  //}
  //return ReadAddress;
//}
//int Map2d::ReadArrayFromEEPROM(int StartAddress, int* Array, int ArraySize) {
  //int ReadAddress = StartAddress;
  //int Size = sizeof(int);
  //for (int i = 0; i < ArraySize; i++) {
    //EEPROM.get(ReadAddress, Array[i]);
    //ReadAddress += Size;
  //}
  //return ReadAddress;
//}
//int Map2d::SaveArrayToEEPROM(int StartAddress, int* Array, int ArraySize) {
  //int WriteAddress = StartAddress;
  //int Size = sizeof(int);
  //for (int i = 0; i < ArraySize; i++) {
    //EEPROM.put(WriteAddress, Array[i]);
    //WriteAddress += Size;
  //}
  //return WriteAddress;
//}



void Map2d::DebugMapIndices() {
  Serial.println("Here's how the map indices are laid out");
  for (int r = Rows - 1; r >= 0; r--) {
    Serial.print(rowData[r]);
    Serial.print("\t |\t");
    for (byte c = 0; c < Columns; c++) {
      Serial.print(GetMapIndex(r, c));
      Serial.print("\t");
    }//for c
    Serial.println();
  }//for r
  Serial.println("\t - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ");
  Serial.print("\t \t");

  for (byte c2 = 0; c2 < Columns; c2++) {
    Serial.print(columnData[c2]);
    Serial.print("\t");

  } // for c2
  Serial.println("\n \n");
  delay(500);
}


void Map2d::Debug() {
  //Debug map values
  Serial.println("Here's the map values");
  for (int r = Rows - 1; r >= 0; r--) {
    Serial.print(rowData[r]);
    Serial.print("\t |\t");
    for (byte c = 0; c < Columns; c++) {

      Serial.print(GetMapValue(r, c));
      Serial.print("\t");
    }//for c
    Serial.println();
  }//for r
  Serial.println("\t - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ");
  Serial.print("\t \t");
  for (byte c2 = 0; c2 < Columns; c2++) {
    Serial.print(columnData[c2]);
    Serial.print("\t");
  }//for c2
  Serial.println();
  Serial.println("\n");
  // delay(3000);
}

