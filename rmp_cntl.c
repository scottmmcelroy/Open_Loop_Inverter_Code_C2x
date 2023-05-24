//#############################################################################
//
// FILE:    rmp_cntrl.c
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: March 2023
//
// REVISION HISTORY:
//
//  V0.1 -2/22/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################

//*******************
// Includes
//*******************
#include "rmp_cntl.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

//*******************
// Defines
//*******************

//*******************
// Globals
//*******************
RMPCNTL_F rc1_f = RMPCNTL_DEFAULTS_F;
extern bool gpio_value = false;

//*******************
// Functions
// Frequency Output Setpoint should be a percent amount <=1 of the max potential fundamental frequency
// FundamentalMax = 600Hz so 0.3 is 180Hz
// Max Frequency = 1/ISRsamplingTime * 2/StepAngleMax
//*******************
void rampControl(RMPCNTL_F *rc_f){
    //store the output vs setpoint into temp (how far away from target current value is)
    rc_f->Tmp = (rc_f->TargetValue) - (rc_f->SetpointValue);

    //*****************************
    //****Ramping down
    if(rc_f->Tmp < 0){
        //checking if the set point is close to value (or within a single resolution)
        if( (-(rc_f->Tmp)) >= rc_f->Resolution ){
            //makes sure the flag is down
            rc_f->EqualFlag = false;
            //increase delay count
            rc_f->RampDelayCount++;

            //if the delay count is complete
            if(rc_f->RampDelayCount >= rc_f->RampDelayMax){
                //decrease target value
                if(rc_f->TargetValue <= rc_f->SetpointValue){
                    rc_f->SetpointValue -= rc_f->Resolution;
                    //saturate the value if above target
                    if(rc_f->SetpointValue < rc_f->RampLowLimit){
                        rc_f->SetpointValue = rc_f->RampLowLimit;
                    }
                }else{
                    //this function can technically never run or can only run once to bring an overshoot down
                    rc_f->SetpointValue += rc_f->Resolution;
                    //if setpoint is below limit saturate values
                    if(rc_f->SetpointValue > rc_f->RampHighLimit){
                        rc_f->SetpointValue = rc_f->RampHighLimit;
                    }
                }
                //reset delay
                rc_f->RampDelayCount = 0;
            }

        }else{
            //if the resolution is close set flag
            rc_f->EqualFlag = true;
        }
    }//if statement for down going ramp

    //*******************************
    //****Ramping up
    if(rc_f->Tmp > 0){
        //checking if the set point is close to value (or within a single resolution)
        if(rc_f->Tmp >= rc_f->Resolution){
            //makes sure the flag is down
            rc_f->EqualFlag = false;
            //increase delay count
            rc_f->RampDelayCount++;

            //if the delay count is complete
            if(rc_f->RampDelayCount >= rc_f->RampDelayMax){
                //increase target value
                if(rc_f->TargetValue >= rc_f->SetpointValue){
                    rc_f->SetpointValue += rc_f->Resolution;
                    //saturate the value if above target
                    if(rc_f->SetpointValue >= rc_f->RampHighLimit){
                        rc_f->SetpointValue = rc_f->RampHighLimit;
                    }
                }else{
                    //this function can technically never run or can only run once to bring an overshoot down
                    rc_f->SetpointValue -= rc_f->Resolution;
                    //if setpoint is below limit saturate values
                    if(rc_f->SetpointValue < rc_f->RampLowLimit){
                        rc_f->SetpointValue = rc_f->RampLowLimit;
                    }
                }
                //reset delay
                rc_f->RampDelayCount = 0;
            }

        }else{
            //if the resolution is close set flag
            rc_f->EqualFlag = true;
        }
        //*********************
    }//if statement for up going ramp
    //************
}//end of function

//*************************************************************
//********************End of file******************************
//*************************************************************

