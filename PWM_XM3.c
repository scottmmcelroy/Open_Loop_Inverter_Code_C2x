//#############################################################################
//
// FILE:    PWM_XM3.c
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

//*******
// Included Files
//********
#include "driverlib.h"
#include "device.h"
#include "PWM_XM3.h"
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include "svgen.h"
#include "ipark.h"
#include "IQmathLib.h"
#include "rampgen.h"
#include "rmp_cntl.h"
#include "pwm_gen.h"
#include "TIMER_XM3.h"
//*********
// Defines
//*********

//***************************************
// Globals to hold the ePWM information used in this example
//*******************************************
epwmInformation epwm1Info;
epwmInformation epwm2Info;
epwmInformation epwm3Info;
LEDepwmInformation epwmLEDInfo;

IPARK ipark1 = IPARK_DEFAULTS;
RAMPGEN rg1 = RAMPGEN_DEFAULTS;
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
PWMGEN pwm1 = PWMGEN_DEFAULTS;

//initialize switching parameters
uint16_t SWITCHING_FREQ = 10e3;   // Default of 20 kHz switching frequency
uint16_t DEAD_TIME = 100;         // 1.3us of dead time by default 1=6.67ns
float FUND_FREQ = 120;         // Default of 300 Hz fundamental frequency
float FundamentalFrequencyMax = 600;
float FUND_FREQ_F = 10.0;  //floating point version of the fundamental for _IQ porting
float MF = 1;                // Default of 0.9 modulation depth, modulation amplitude (0-1)
float radian = 0;              // Initialize radian to 0
float Ts = 0;                        // Switching frequency
float Tsample = 0;
float Sine = 0;                     //compare value period

_iq VdTesting = _IQ(0);
_iq VqTesting = _IQ(0.2);
float VdTesting_f = 0;
float VqTesting_f = 0.2;

//************
// Functions
//************

//********
// initEPWM - Configure all ePWMs
//*******
void initMotorBlocks(void)
{
    //initialize switching parameters
    Ts = 2*PI/(SWITCHING_FREQ/FUND_FREQ);  // switching frequency period
    Tsample = 1/(ISRperiod); //ISRperiod is 100kHz
    //this will determine fundamental frequency max
    //2PI divided by the ISR/fundamental give max steps for each cycle
    rg1_f.StepAngleMax = (2*PI)/(ISRperiod/FundamentalFrequencyMax); //min resolution of an angle step

    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2;
    pwm1_f.PeriodMax = EPwm_TBPRD;
    pwm1_f.HalfPerMax = EPwm_TBPRD/2;
}

//********
// initEPWM - Configure all ePWMs
//*******
void initEPWM()
{
    //epwm 1-3
    initEPWM1();
    initEPWM2();
    initEPWM3();
    initCaseLEDPWM();
}

//***
// initEPWM1 - Configure ePWM1
//***
void initEPWM1()
{

    // Set-up TBCLK
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0); //if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO); //module EPWM1 is master

    // Set Compare values
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPwm_TBPRD*0.5); //start with 50%

    // Set up counter mode
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE); //master has no phase shift
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // Set up shadowing
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Set actions
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);


    // Set Deadband Active High Complimentary
    EPWM_setDeadBandControlShadowLoadMode(EPWM1_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM1_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    // Enable TZ1 as one shot trip sources
    EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_OSHT1);

    // Action on TZ1 set both outputs low
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);

    // Enable TZ interrupt
    EPWM_enableTripZoneInterrupt(EPWM1_BASE, EPWM_TZ_INTERRUPT_OST);

    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U); //interupt every switching cycle for (required for sine math)

    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm1Info.epwmModule = EPWM1_BASE;
    epwm1Info.epwmPeriod = EPwm_TBPRD;
    epwm1Info.epwmPwmPhase = 0;
    epwm1Info.epwmDeadTime = DEAD_TIME;
    epwm1Info.epwmRadian = 0;
    epwm1Info.epwmCompVal = (uint16_t)(EPwm_TBPRD*0.5);

}//initEPWM1


//***
// initEPWM2 - Configure ePWM2
//***
void initEPWM2()
{

    // Set-up TBCLK
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM2_BASE, 0);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    // Set Compare values
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, EPwm_TBPRD*0.5); //start with 50%

    // Set-up counter mode
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setClockPrescaler(EPWM2_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // Set-up shadowing
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Set Action qualifier
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    // Set Deadband Active High Complimentary
    EPWM_setDeadBandControlShadowLoadMode(EPWM2_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM2_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    // Enable TZ1 as one shot trip sources
    EPWM_enableTripZoneSignals(EPWM2_BASE, EPWM_TZ_SIGNAL_OSHT1);

    // Action on TZ1 set both outputs low
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);

    // Enable TZ interrupt
    EPWM_enableTripZoneInterrupt(EPWM2_BASE, EPWM_TZ_INTERRUPT_OST);

    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //EPWM_setInterruptSource(EPWM2_BASE, EPWM_INT_TBCTR_ZERO);
    // EPWM_enableInterrupt(EPWM2_BASE);
    // EPWM_setInterruptEventCount(EPWM2_BASE, 3U);

    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers

    epwm2Info.epwmModule = EPWM2_BASE;
    epwm2Info.epwmPeriod = EPwm_TBPRD;
    epwm2Info.epwmPwmPhase = 0;
    epwm2Info.epwmDeadTime = DEAD_TIME;
    epwm2Info.epwmRadian = 2*PI/3;
    epwm2Info.epwmCompVal = (uint16_t)(EPwm_TBPRD*0.5);

}//initEPWM2

//****
// initEPWM3 - Configure ePWM3
//****
void initEPWM3()
{

    // Set-up TBCLK
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM3_BASE, 0);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    // Set Compare values
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPwm_TBPRD*0.5); //start with 50%

    // Set-up counter mode
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // Set up shadowing
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Set Action qualifier
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    // Set Deadband Active High Complimentary
    EPWM_setDeadBandControlShadowLoadMode(EPWM3_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    // Enable TZ1 as one shot trip sources
    EPWM_enableTripZoneSignals(EPWM3_BASE, EPWM_TZ_SIGNAL_OSHT1);

    // Action on TZ1 set both outputs low
    EPWM_setTripZoneAction(EPWM3_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM3_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);

    // Enable TZ interrupt
    EPWM_enableTripZoneInterrupt(EPWM3_BASE, EPWM_TZ_INTERRUPT_OST);

    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //EPWM_setInterruptSource(EPWM3_BASE, EPWM_INT_TBCTR_ZERO);
    // EPWM_enableInterrupt(EPWM3_BASE);
    // EPWM_setInterruptEventCount(EPWM3_BASE, 3U);

    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm3Info.epwmModule = EPWM3_BASE;
    epwm3Info.epwmPeriod = EPwm_TBPRD;
    epwm3Info.epwmPwmPhase = 0;
    epwm3Info.epwmDeadTime = DEAD_TIME;
    epwm3Info.epwmRadian = 4*PI/3;
    epwm3Info.epwmCompVal = (uint16_t)(EPwm_TBPRD*0.5);
}

//*****
// updateCompare - Function to update the frequency
//****
void updatePWM(epwmInformation *epwmInfo)
{

    Ts = 2*PI/(SWITCHING_FREQ/FUND_FREQ);
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(epwmInfo->epwmModule, EPwm_TBPRD);
    EPWM_setPhaseShift(epwmInfo->epwmModule, EPwm_TBPRD*epwmInfo->epwmPwmPhase);
    epwmInfo->epwmDeadTime = DEAD_TIME;
    EPWM_setRisingEdgeDelayCount(epwmInfo->epwmModule, epwmInfo->epwmDeadTime);
    EPWM_setFallingEdgeDelayCount(epwmInfo->epwmModule, epwmInfo->epwmDeadTime);

    // Sine wave math
    //For 50% duty cycle comment out below two lines
    Sine = ((((MF*sin(epwmInfo->epwmRadian))+1.0))/2);
    epwmInfo->epwmRadian += Ts;

    if(epwmInfo->epwmRadian > 2*PI){
        epwmInfo->epwmRadian -= (2*PI);
    }
    //Use below for 50% duty cycle
//    Sine = 0.5;

//    EPWM_setCounterCompareValue(epwmInfo->epwmModule, EPWM_COUNTER_COMPARE_A, Sine*EPwm_TBPRD);
//    //below code shows what the current compare value is
    epwmInfo->epwmCompVal = (uint16_t)(Sine*EPwm_TBPRD);

    //only update on the ePWM1 value
    if(epwmInfo->epwmModule == EPWM1_BASE){
        NOP;
    }


}//updatePWM


//***
// updateLED - Function to update the LED
//***
void updateLED(LEDepwmInformation *epwmInfo)
{
    uint16_t compAValue;

    compAValue = EPWM_getCounterCompareValue(epwmInfo->epwmModule, EPWM_COUNTER_COMPARE_A);

    //  Change the CMPA values every 10th interrupt.
    if(epwmInfo->epwmTimerIntCount == 10U)
    {
        epwmInfo->epwmTimerIntCount = 0U;

        // If we were increasing CMPA, check to see if we reached the max
        // value. If not, increase CMPA else, change directions and decrease
        // CMPA
        if(epwmInfo->epwmCompADirection == EPWM_CMP_UP)
        {
            if(compAValue < (epwmInfo->epwmMaxCompA))
            {
                EPWM_setCounterCompareValue(epwmInfo->epwmModule, EPWM_COUNTER_COMPARE_A, ++compAValue);
            }
            else
            {
                epwmInfo->epwmCompADirection = EPWM_CMP_DOWN;
                EPWM_setCounterCompareValue(epwmInfo->epwmModule, EPWM_COUNTER_COMPARE_A, --compAValue);
            }
        }

        // If we were decreasing CMPA, check to see if we reached the min
        // value. If not, decrease CMPA else, change directions and increase
        // CMPA
        else
        {
            if( compAValue == (epwmInfo->epwmMinCompA))
            {
                epwmInfo->epwmCompADirection = EPWM_CMP_UP;
                EPWM_setCounterCompareValue(epwmInfo->epwmModule, EPWM_COUNTER_COMPARE_A, ++compAValue);
            }
            else
            {
                EPWM_setCounterCompareValue(epwmInfo->epwmModule, EPWM_COUNTER_COMPARE_A, --compAValue);
            }
        }

    }
    else
    {
        epwmInfo->epwmTimerIntCount++;
    }
}//updateLED

//*****
// TestLEDS - cycle led pins to test functionality
//*****
void TestLEDPulse(){
    Interrupt_register(INT_EPWM6, &epwm6ISR);
    initCaseLEDPWM();
    Interrupt_enable(INT_EPWM6);

}//TestLEDPulse

//****
// initCaseLEDPWM
//****
void initCaseLEDPWM()
{
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_setPinConfig(GPIO_10_EPWM6A);              // GPIO10 = EPWM6A

    // Set-up TBCLK
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM6_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM6_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);

    // Set Compare values
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, 1000U);

    // Set up counter mode
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setClockPrescaler(EPWM6_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // Set up shadowing
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Set actions
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    EPWM_setInterruptSource(EPWM6_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM6_BASE);
    EPWM_setInterruptEventCount(EPWM6_BASE, 6U);

    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwmLEDInfo.epwmCompADirection = EPWM_CMP_UP;
    epwmLEDInfo.epwmTimerIntCount = 0U;
    epwmLEDInfo.epwmModule = EPWM6_BASE;
    epwmLEDInfo.epwmMaxCompA = EPWM6_MAX_CMPA;
    epwmLEDInfo.epwmMinCompA = EPWM6_MIN_CMPA;
}//initCaseLEDPWM

//*******************************
//  PWM Macro update for function
// deprecated
//*******************************
void updatePWM_IQ(PWMGEN pwm){
    /*  Mfuncx range is (-1,1)                                                      */
    /*  The code below changes PeriodMax*Mfuncx range ....                          */
    /*  from (-PeriodMax,PeriodMax) to (0,PeriodMax) where HalfPerMax=PeriodMax/2   */

    //calculate new values
    uint16_t compA = _IQmpy(pwm.HalfPerMax,pwm.DCR_A) + pwm.HalfPerMax;
    uint16_t compB = _IQmpy(pwm.HalfPerMax,pwm.DCR_B) + pwm.HalfPerMax;
    uint16_t compC = _IQmpy(pwm.HalfPerMax,pwm.DCR_C) + pwm.HalfPerMax;

    //translate and store new values
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, compA);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, compB);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, compC);
}

//************
// ISR
//************

//***
// epwm1ISR - ePWM 1 ISR
//***
__interrupt void epwm1ISR(void)
{

    // Update the CMPA and CMPB values
    updatePWM(&epwm1Info);
//    updatePWM(&epwm2Info);
//    updatePWM(&epwm3Info);

    // Clear INT flag for this timer
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    // Acknowledge interrupt group
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//*****
// epwm1TZISR - ePWM1 TZ ISR
//*****
__interrupt void epwm1TZISR(void)
{
    //FAULT1 =1; //TZ is global fault so might be wrong channel

    // To re-enable the OST Interrupt, uncomment the below code:
    // EPWM_clearTripZoneFlag(EPWM1_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    // Acknowledge this interrupt to receive more interrupts from group 2
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//****
// epwm2TZISR - ePWM2 TZ ISR
//****
__interrupt void epwm2TZISR(void)
{
    //FAULT2 =1; //TZ is global fault so might be wrong channel

    // To re-enable the OST Interrupt, uncomment the below code:
    // EPWM_clearTripZoneFlag(EPWM2_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    // Acknowledge this interrupt to receive more interrupts from group 2
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//****
// epwm3TZISR - ePWM3 TZ ISR
//****
__interrupt void epwm3TZISR(void)
{

    //FAULT3 =1; //TZ is global fault so might be wrong channel

    // To re-enable the OST Interrupt, uncomment the below code:
    // EPWM_clearTripZoneFlag(EPWM3_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    // Acknowledge this interrupt to receive more interrupts from group 2
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//****
// epwm6ISR - ePWM 6 ISR
//****
__interrupt void epwm6ISR(void)
{
    // Update the CMPA and CMPB values
//    updateLED(&epwmLEDInfo);

    // Clear INT flag for this timer
//    EPWM_clearEventTriggerInterruptFlag(EPWM6_BASE);

    // Acknowledge interrupt group
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


//*************************************************************
//********************End of file******************************
//*************************************************************


