//#############################################################################
//
// FILE:    TIMER_XM3.c
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

//*******
// Included Files
//********
#include "driverlib.h"
#include "device.h"
#include "cputimer.h"
#include <stdint.h>
//*********
// Defines
//*********
#define TIMER0_PERIOD 2000 //200MHz/2000 = 100kHz (10us)

float ISRperiod = 100000;

//***********************
// Function Prototypes
//***********************
void timer0_init(void){

    //disable the timer interrupt first
    CPUTimer_disableInterrupt(CPUTIMER0_BASE);
    //Want to configure CPUTIMER0_BASE
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    CPUTimer_selectClockSource(CPUTIMER0_BASE, CPUTIMER_CLOCK_SOURCE_SYS, CPUTIMER_CLOCK_PRESCALER_1);

    CPUTimer_setPeriod(CPUTIMER0_BASE, TIMER0_PERIOD);

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);

}


//*************************************************************
//********************End of file******************************
//*************************************************************
