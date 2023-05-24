//#############################################################################
//
// FILE:    pwm_gen.c
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: April 2023
//
// REVISION HISTORY:
//
//  V0.1 -4/7/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################

//*******************
// Includes
//*******************
#include "pwm_gen.h"
#include <stdint.h>
#include "driverlib.h"
#include "device.h"

//******************
// Defines
//********************

//***********
// Global
//***********
PWMGEN_F pwm1_f = PWM_GEN_DEFAULTS_F;
uint16_t compA = 0;
uint16_t compB = 0;
uint16_t compC = 0;
uint16_t compA_min = 0;
uint16_t compA_max = 0;
uint16_t compB_min = 0;
uint16_t compB_max = 0;
uint16_t compC_min = 0;
uint16_t compC_max = 0;
//****************************************
// Function Declarations
// input: pointer to pwm_gen_f variable
//****************************************
void pwm_gen_f(PWMGEN_F *pwm_gen_f){
    /*  Mfuncx range is (-1,1)                                                      */
    /*  The code below changes PeriodMax*Mfuncx range ....                          */
    /*  from (-PeriodMax,PeriodMax) to (0,PeriodMax) where HalfPerMax=PeriodMax/2   */

    //HALF PERIOD is the one that functions within bounds
    //calculate new values
    compA = (uint16_t)((pwm_gen_f->DCR_A * (float)(pwm_gen_f->HalfPerMax)) + (float)(pwm_gen_f->HalfPerMax));
    compB = (uint16_t)((pwm_gen_f->DCR_B * (float)(pwm_gen_f->HalfPerMax)) + (float)(pwm_gen_f->HalfPerMax));
    compC = (uint16_t)((pwm_gen_f->DCR_C * (float)(pwm_gen_f->HalfPerMax)) + (float)(pwm_gen_f->HalfPerMax));

    //testing the comp values
    //compA
    if(compA > compA_max){
        compA_max = compA;
    }

    if(compA < compA_min){
        compA_min = compA;
    }
    //compB
    if(compB > compB_max){
        compB_max = compB;
    }

    if(compB < compB_min){
        compB_min = compB;
    }
    //compC
    if(compC > compC_max){
        compC_max = compC;
    }

    if(compC < compC_min){
        compC_min = compC;
    }

    //translate and store new values
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, compA);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, compB);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, compC);
}

//*************************************************************
//********************End of file******************************
//*************************************************************
