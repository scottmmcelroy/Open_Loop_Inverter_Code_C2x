//#############################################################################
//
// FILE:    TIMER_XM3.h
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: March 2023
//
// REVISION HISTORY:
//
//  V0.1 -2/20/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################


#ifndef TIMER_XM3_H_
#define TIMER_XM3_H_

//*******
// Included Files
//********
#include "driverlib.h"
#include "device.h"
#include "cputimer.h"

//*********
// Defines
//*********
extern float ISRperiod;
//***********************
// Function Prototypes
//***********************
void timer0_init(void);


#endif /* TIMER_XM3_H_ */

//*************************************************************
//********************End of file******************************
//*************************************************************
