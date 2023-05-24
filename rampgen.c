//#############################################################################
//
// FILE:    rampgen.c
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: March 2023
//
// REVISION HISTORY:
//
//  V0.1 -2/23/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################

//*******
// Included Files
//********
#include "rampgen.h"
#include "gpio.h"
#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include "PWM_XM3.h"
//*********
// Defines
//*********

//***********
// Global
//***********
RAMPGEN_F rg1_f = RAMPGEN_DEFAULTS_F;
float angleValue = 0;
float angleMax = 0;
float angleMin = 0;
uint16_t count = 0;
//****************************************
// Function
// input: pointer to ramp control variable
// Function creates a sawtooth generator output based on freq
// ISR sample rate = 100kHz
// Max number of Theta steps = 2/(StepAngleMax) = 2/0.012 = 166.667
// Min Time Period = Tisr_sample * max number of theta steps = 0.00001 * 166.667 = 0.00166667
// Max Theta Fundamental period = 1/0.00166667 = 600Hz (Electrical Fundamental)
// Angle Steps = StepAngleMax * Freq = 0.012 * 0.3 = 0.036 per Theta Angle
// Theta Frequency = 2/AngleSteps = 555.556 * 0.00001 = 0.00555556 => 1/0.0055556 = 180Hz
//****************************************
void rampGen_f(RAMPGEN_F *rg_f){

    //calculate the angle (multiply step angle by freq)
    rg_f->Angle += rg_f->StepAngleMax * rg_f->Freq;

    //testing the RG Angle
    if(rg_f->Angle >= (0.95)*(2*PI)){
        GPIO_writePin(95, 1);
    }

    //function of the angle direction
    if(rg_f->Angle > 2*PI){
        rg_f->Angle = 0;
    }

    if(rg_f->Angle <= 0){
        //GPIO_writePin(95, 0);
    }

    //output the angle results
    rg_f->Out = rg_f->Angle;
}



//*************************************************************
//********************End of file******************************
//*************************************************************
