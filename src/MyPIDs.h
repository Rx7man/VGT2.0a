#pragma once
//Turbo stuff
#define VGTaddress  0x0CFFC600UL      //CAN bus address of the VGT controller
#define StatusMessageID  0x18FFC502UL //this is the message id of the messages we're concerned with
#define BreakMessageID  0x18FF0A02UL  //This is the message id of a 'terminator' string, or something... no relevant information it seems
#define ErrorMessageID  0x18EEFF02UL  //Seems to be an error message ID when low voltage is triggered?


#define UpdateSpeedPID 10001 //use this to change the update speed from one box to the other
