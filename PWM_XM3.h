//#############################################################################
//
// FILE:    PWM_XM3.h
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

#ifndef PWM_XM3_H_
#define PWM_XM3_H_

//*****************
// Included Files
//*****************
#include "driverlib.h"
#include "device.h"
#include <math.h>
#include <stdlib.h>
#include "svgen.h"
#include "ipark.h"
#include "IQmathLib.h"
#include "rampgen.h"
#include "rmp_cntl.h"
#include "pwm_gen.h"

//*********
// Defines
//*********
//CASELED1 PWM for effect
#define EPWM_CMP_UP           1U
#define EPWM_CMP_DOWN         0U
#define EPWM6_TIMER_TBPRD  2000U
#define EPWM6_MAX_CMPA     1950U
#define EPWM6_MIN_CMPA       50U

#define PI 3.141592654  // Pi

//*******************
// Globals Structures
//*******************
typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmPeriod;
    float epwmPwmPhase;
    uint16_t epwmDeadTime;
    float epwmRadian;
    uint16_t epwmCompVal;
}epwmInformation;

typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmCompADirection;
    uint16_t epwmTimerIntCount;
    uint16_t epwmMaxCompA;
    uint16_t epwmMinCompA;
}LEDepwmInformation;

//*************************************
// Globals (can be used in other files)
//*************************************
extern IPARK ipark1;
extern RAMPGEN rg1;
extern RMPCNTL rc1;
extern PWMGEN pwm1;

extern _iq VdTesting;
extern _iq VqTesting;
extern float VdTesting_f;
extern float VqTesting_f;
extern float FUND_FREQ;
extern uint16_t SWITCHING_FREQ;
extern float FundamentalFrequencyMax;

//***********************
// Function Prototypes
//***********************
void initMotorBlocks(void);
void initEPWM(void);
void initEPWM1(void);
void initEPWM2(void);
void initEPWM3(void);
void initCaseLEDPWM(void);
void TestLEDPulse(void);
void updatePWM(epwmInformation *epwmInfo);
void updateLED(LEDepwmInformation *epwmInfo);

void updatePWM_IQ(PWMGEN pwm);

__interrupt void epwm1ISR(void);
__interrupt void epwm6ISR(void);
__interrupt void epwm1TZISR(void);
__interrupt void epwm2TZISR(void);
__interrupt void epwm3TZISR(void);

#endif /* PWM_XM3_H_ */

//*************************************************************
//********************End of file******************************
//*************************************************************

