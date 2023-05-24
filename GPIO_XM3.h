//#############################################################################
//
// FILE:    GPIO_XM3.h
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: March 2023
//
// REVISION HISTORY:
//
//  V0.1 -2/10/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################

#ifndef GPIO_XM3_H_
#define GPIO_XM3_H_

//*****************
// Included Files
//*****************
#include "driverlib.h"
#include "device.h"
#include <math.h>
#include "gpio.h"

//*****************
// Defines
//*****************
#define XM3GDV2
//#define XM3GDV1

//***********************
// Function Prototypes
//***********************
void initGPIO(void);
void TestLEDS(void);
void initGateDriverGPIO(void);

//phase A gate driver control
void GD_A_PSEnable(void);
void GD_A_PSDisable(void);
void GD_A_LogicEnable(void);
void GD_A_LogicDisable(void);
void GD_A_OCEnable(void);
void GD_A_OCDisable(void);

//phase B gate driver control
void GD_B_PSEnable(void);
void GD_B_PSDisable(void);
void GD_B_LogicEnable(void);
void GD_B_LogicDisable(void);
void GD_B_OCEnable(void);
void GD_B_OCDisable(void);

//phase C gate driver control
void GD_C_PSEnable(void);
void GD_C_PSDisable(void);
void GD_C_LogicEnable(void);
void GD_C_LogicDisable(void);
void GD_C_OCEnable(void);
void GD_C_OCDisable(void);

//ALL gate driver control
void GD_ALL_PSEnable(void);
void GD_ALL_PSDisable(void);
void GD_ALL_LogicEnable(void);
void GD_ALL_LogicDisable(void);
void GD_ALL_OCEnable(void);
void GD_ALL_OCDisable(void);
void GD_ALL_Reset(void);

//read Fault state from gate driver
//return true if there is a fault
//return false if there is NO fault
bool GD_A_getFault(void);
bool GD_B_getFault(void);
bool GD_C_getFault(void);
bool GD_Global_getFault(void);


#endif /* GPIO_XM3_H_ */
//*************************************************************
//********************End of file******************************
//*************************************************************

