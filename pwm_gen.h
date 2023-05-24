//#############################################################################
//
// FILE:    pwm_gen.h
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

#ifndef PWM_GEN_H_
#define PWM_GEN_H_

//*******************
// Includes
//*******************
#include "IQmathLib.h"
#include <stdint.h>

//******************
// Defines
//********************
/*-----------------------------------------------------------------------------
    Define the structure of the PWM Driver Object
-----------------------------------------------------------------------------*/
typedef struct {
        uint16_t PeriodMax;   // Parameter: PWM Half-Period in CPU clock cycles (Q0)
        uint16_t HalfPerMax;  // Parameter: Half of PeriodMax                   (Q0)
        uint16_t Deadband;    // Parameter: PWM deadband in CPU clock cycles    (Q0)
        _iq DCR_A;        // Input: EPWM1 A&B Duty cycle ratio (Q24)
        _iq DCR_B;        // Input: EPWM2 A&B Duty cycle ratio (Q24)
        _iq DCR_C;        // Input: EPWM3 A&B Duty cycle ratio (Q24)
}PWMGEN;


typedef struct {
        uint16_t PeriodMax;   // Parameter: PWM Half-Period in CPU clock cycles (Q0)
        uint16_t HalfPerMax;  // Parameter: Half of PeriodMax                   (Q0)
        uint16_t Deadband;    // Parameter: PWM deadband in CPU clock cycles    (Q0)
        float DCR_A;        // Input: EPWM1 A&B Duty cycle ratio (Q24)
        float DCR_B;        // Input: EPWM2 A&B Duty cycle ratio (Q24)
        float DCR_C;        // Input: EPWM3 A&B Duty cycle ratio (Q24)
} PWMGEN_F;

//*****************
// Defaults
//******************
//*****************************************************
//    Default Initializers for the F2803X PWMGEN Object
//*****************************************************
#define PWMGEN_DEFAULTS   {  0,         \
                             0,         \
                             _IQ(0),    \
                             _IQ(0),    \
                             _IQ(0),    \
                          }

#define PWM_GEN_DEFAULTS_F {  1000, \
                               500, \
                               100, \
                            0x4000, \
                            0x4000, \
                            0x4000, \
                          }

//***********
// Global
//***********
extern PWMGEN_F pwm1_f;
extern uint16_t compA;
extern uint16_t compB;
extern uint16_t compC;

extern uint16_t compA_min;
extern uint16_t compA_max;
extern uint16_t compB_min;
extern uint16_t compB_max;
extern uint16_t compC_min;
extern uint16_t compC_max;
//****************************************
// Function Declarations
// input: pointer to pwm_gen_f variable
//****************************************
void pwm_gen_f(PWMGEN_F *pwm_gen_f);

#endif /* PWM_GEN_H_ */

//*************************************************************
//********************End of file******************************
//*************************************************************
