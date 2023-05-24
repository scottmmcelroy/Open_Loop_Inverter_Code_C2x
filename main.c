//#############################################################################
//
// FILE:    main.c
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: March 2023
//
// PURPOSE:
//  This software is designed for the evaluation of the XM3 3-phase Inverter
//  Reference Design (CRD300DA12E-XM3) and provides only the basic code
//  required to create an open-loop inverter.
//
// REVISION HISTORY:
//
//  V0.1 -2/10/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################

//***************
// Included Files
//***************
#include "driverlib.h"
#include "device.h"
//#include "GATEDRIVER.h"
//#include "CANSetup.h"
#include "PWM_XM3.h"
#include "GPIO_XM3.h"
#include "ipark.h"
#include "IQmathLib.h"
#include <math.h>
#include "rampgen.h"
#include "rmp_cntl.h"
#include "cputimer.h"
#include "TIMER_XM3.h"
#include <stdbool.h>
#include "svgen.h"
#include "pwm_gen.h"
#include "Analog.h"
#include "TEMPERATURE.h"
#include "Current.h"
#include "Voltage.h"
//********
// Defines
//********

//********
// Globals
//********
float Ta = 0;
float Ta_max = 0;
float Ta_min = 0;
float Tb = 0;
float Tb_max = 0;
float Tb_min = 0;
float Tc = 0;
float Tc_max = 0;
float Tc_min = 0;

float VD = 0;
float VQ = 0.01;
//currently 600Hz
float FundamentalMaxPercentage = 0.00167;
float alphaMax = 0;
float betaMax = 0;
float alphaMin = 0;
float betaMin = 0;

_iq testValue = _IQ(0);
_iq testOut = _IQ(0);

//****************
//Voltage
int16_t Voltage_A_High = 0;
int16_t Voltage_A_Low = 0;
int16_t Voltage_B_High = 0;
int16_t Voltage_B_Low = 0;
int16_t Voltage_C_High = 0;
int16_t Voltage_C_Low = 0;
int16_t Voltage_DC_High = 0;
int16_t Voltage_DC_Low = 0;
//Current
int16_t Current_A_High = 0;
int16_t Current_A_Low = 0;
int16_t Current_B_High = 0;
int16_t Current_B_Low = 0;
int16_t Current_C_High = 0;
int16_t Current_C_Low = 0;
int16_t Current_Ext_High = 0;
int16_t Current_Ext_Low = 0;
//Temperature
int16_t Temperature_A_High = 0;
int16_t Temperature_A_Low = 0;
int16_t Temperature_B_High = 0;
int16_t Temperature_B_Low = 0;
int16_t Temperature_C_High = 0;
int16_t Temperature_C_Low = 0;
int16_t Temperature_Case_High = 0;
int16_t Temperature_Case_Low = 0;


//********************
// Function Prototypes
//********************
//void initGPIO(void);
//void TestShutdown(void);
//void enableNeg15V(void);
//void enablePos15V(void);
//void disableNeg15V(void);
//void disablePos15V(void);

__interrupt void MainISR(void);

//*************************
// Main
//*************************
void main(void)
{
    // test variables
    float freq_set = 0;
    float theta = 0;
    float alpha_f = 0;
    float beta_f = 0;

    // Initializes system control, device clock, and peripherals
    Device_init();
    // Disable pin locks and enable internal pull ups.
    Device_initGPIO();

    //enable and setup all the GPIO
    initGPIO();
    initGateDriverGPIO();

    //initialize the motor macro blocks
    initMotorBlocks();
    // Initializes PIE and clear PIE registers. Disables CPU interrupts.
    // and clear all CPU interrupt flags.
    Interrupt_initModule();
    // Initialize the PIE vector table with pointers to the shell interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();

    // Assign interrupt service routines
    Interrupt_register(INT_TIMER0, &MainISR);
    // Assign the interrupt service routines to ePWM interrupts
//    Interrupt_register(INT_EPWM1, &epwm1ISR);
//    Interrupt_register(INT_EPWM6, &epwm6ISR);
//    Interrupt_register(INT_EPWM1_TZ, &epwm1TZISR);
//    Interrupt_register(INT_EPWM2_TZ, &epwm2TZISR);
//    Interrupt_register(INT_EPWM3_TZ, &epwm3TZISR);

    // Disable sync(Freeze clock to PWM as well)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //setup the ePWMs
    initEPWM();
    //enable the timer0
    timer0_init();
    // Enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //initialize the ADC
    initADCs();
    initADCSOCs();

    // Enable ePWM interrupts
    Interrupt_enable(INT_TIMER0);
//    Interrupt_enable(INT_EPWM1);
    // Enable Global Interrupt (INTM) and real-time interrupt (DBGM)
    EINT;
    ERTM;

    while(1){

        //************************************************
        //************************************************
        //  Create Infinite loop wait state
        //************************************************
        //************************************************
        //************************************************
        //  Start Gathering System values
        //
        //************************************************
        //ADC-A values
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);   //A-Current
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);   //Ext-Current
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER2);   //B-Current
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER3);   //Vsense-B
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER4);   //A-temp
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER5);   //C-temp
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER6);   //Case-temp
        //ADC-B values
        ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER0);   //C-current
        ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER1);   //Vsense-C
        //ADC-C values
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);   //Vsense-A
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);   //Vsense-DC
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER2);   //B-temp

        //****
        // Wait for ADCA to complete, then acknowledge flag
        //****
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

        //****
        // Wait for ADCB to complete, then acknowledge flag
        //****
        while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
        //****
        // Wait for ADCC to complete, then acknowledge flag
        //****
        while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);


        //********************************
        //Acquiring voltages/currents/temps
        //********************************
        //********Temperatures*******
        Temperature_A_High = (uint16_t)getECAPTempA()>>8; //A-Temp
        Temperature_A_Low = (uint16_t)getECAPTempA();

        Temperature_B_High = (uint16_t)getECAPTempB()>>8; //B-Temp
        Temperature_B_Low = (uint16_t)getECAPTempB();

        Temperature_C_High = (uint16_t)getECAPTempC()>>8; //C-Temp
        Temperature_C_Low = (uint16_t)getECAPTempC();

        Temperature_Case_High = (uint16_t)getCaseTemp()>>8; //CASE-Temp
        Temperature_Case_Low = (uint16_t)getCaseTemp();

        //**********Currents**********
        Current_A_High = (int16_t)getCurrentA()>>8; //A-Current
        Current_A_Low = (int16_t)getCurrentA();

        Current_B_High = (int16_t)getCurrentB()>>8; //B-Current
        Current_B_Low = (int16_t)getCurrentB();

        Current_C_High = (int16_t)getCurrentC()>>8; //C-Current
        Current_C_Low = (int16_t)getCurrentC();

        Current_Ext_High = (int16_t)getCurrentEXT()>>8; //EXT-Current
        Current_Ext_Low = (int16_t)getCurrentEXT();

        //**********Voltages***********
        Voltage_A_High = (int16_t)getVoltageA()>>8; //Vsense-A
        Voltage_A_Low = (int16_t)getVoltageA();

        Voltage_B_High = (int16_t)getVoltageB()>>8; //Vsense-B
        Voltage_B_Low = (int16_t)getVoltageB();

        Voltage_C_High = (int16_t)getVoltageC()>>8; //Vsense-C
        Voltage_C_Low = (int16_t)getVoltageC();

        Voltage_DC_High = (int16_t)getVoltageDC()>>8; //Vsense-DC
        Voltage_DC_Low = (int16_t)getVoltageDC();
        //*********************************

    }
}//end of main


//*********************************************
// MainISR - Triggered based on the CPU1_Timer0
//*********************************************
__interrupt void MainISR(void)
{
    //clear the overflow flag for the interrupt
    //CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    //****************************************************
    //                  RC MACRO
    // Ramp Generator to generate slow start frequency
    // input: Target Value
    // output: rc1.SetpointValue -> ramp1.Freq
    // Max Fundamental = 600Hz
    //****************************************************
    rc1_f.TargetValue = FundamentalMaxPercentage; //1 => fundamentalFrequencyMax
    rampControl(&rc1_f);

    //****************************************************
    //                  RG MACRO
    // Ramp Gen to generate IPARK angle
    // input: rc1.SetpointValue -> ramp1.Freq
    // output: ramp1.Out -> ipark1.angle
    //****************************************************
    rg1_f.Freq = rc1_f.SetpointValue; //this is the frequency set point output
    rampGen_f(&rg1_f);

    //****************************************************
    //                  IPARK MACRO
    // Inverse Park Formula to calculate Alpha/Beta Clark
    // input: ramp1.Out -> ipark1.angle
    // Output: ipark1.Alpha, ipark1.Beta
    //****************************************************
//    ipark1_f.Ds = VdTesting_f;
//    ipark1_f.Qs = VqTesting_f;
    ipark1_f.Ds = VD; //Vd and Vq are percentages of the max value
    ipark1_f.Qs = VQ;
    ipark1_f.Angle = rg1_f.Out;
    invPark_f(&ipark1_f);

    //Change Alpha and Beta into Comp values
//    ipark1_f.Alpha = ((ipark1_f.Alpha * (float)(pwm1_f.HalfPerMax)) + (float)(pwm1_f.HalfPerMax));
//    ipark1_f.Beta = ((ipark1_f.Beta * (float)(pwm1_f.HalfPerMax)) + (float)(pwm1_f.HalfPerMax));
//    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(ipark1_f.Alpha));
//    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(ipark1_f.Beta));

    //*********************************************
    //                  SVGEN MACRO
    //  Clark Formula to calculate PWM timings
    // input: ipark1.Alpha -> svgen1.Ualpha
    // input: ipark1.Beta -> svgen1.Ubeta
    // Output: ipark1.Alpha, ipark1.Beta
    //**********************************************
    //Alpha and Beta should be between -1 and 1 as long as Vd and Vq are <1
    svgen1_f.Ualpha = ipark1_f.Alpha;
    svgen1_f.Ubeta = ipark1_f.Beta;
    svGen_f(&svgen1_f);

    //watch window (converted) for Ta,Tb,Tc rotating
    Ta = svgen1_f.Ta;
    Tb = svgen1_f.Tb;
    Tc = svgen1_f.Tc;

    //*********************************************
    //                  PWMGEN MACRO
    //  Formula to calculate PWM timings compares using the Ta, Tb, Tc
    //
    // input: pwm1.DCR_A = svgen1.Ta;
    // input: pwm1.DCR_B = svgen1.Tb;
    // input: pwm1.DCR_C = svgen1.Tc;
    // Output: Outputs values to the compare registers
    //************************************************
    pwm1_f.DCR_A = svgen1_f.Ta;
    pwm1_f.DCR_B = svgen1_f.Tb;
    pwm1_f.DCR_C = svgen1_f.Tc;
    pwm_gen_f(&pwm1_f);

}//end of the main ISR


//*************************************************************
//********************End of file******************************
//*************************************************************

