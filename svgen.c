//#############################################################################
//
// FILE:    svgen.c
//
// TITLE:   XM3 Controller For SVPWM Open-Loop Motor Control
//
// AUTHOR: Scott McElroy
// DATE: March 2023
//
// REVISION HISTORY:
//
//  V0.1 -3/1/2023- Initial test code (partial code pulled in from released XM3 Inverter Code)
//
//#############################################################################

//*******************
// Includes
//*******************
#include "svgen.h"
#include <stdint.h>
#include "PWM_XM3.h"
#include "math.h"
//******************
// Defines
//********************

//***********
// Global
//***********
SVGEN_F svgen1_f = SVGEN_DEFAULTS_F;
SVGEN svgen1 = SVGEN_DEFAULTS;

extern uint16_t EPwm_TBPRD = 0;
//****************************************
// Function
// input: pointer to svgen variable
// Function creates the timing needed for svgen operation
// NOTE: Output is a percent of the EPwm_TPRD (needs to be multiplied)
//****************************************
void svGen_f(SVGEN_F *svgen_f){

    //reference values to determine which vector
    svgen_f->tmp1 = svgen_f->Ubeta;
    svgen_f->tmp2 = (0.5)*(((svgen_f->Ubeta)) + (svgen_f->Ualpha * sqrt(3)));
    svgen_f->tmp3 = (0.5)*((-(svgen_f->Ubeta)) + (svgen_f->Ualpha * sqrt(3)));

    //******************************
    //determining sector for which motor vector is in
    svgen_f->VecSector = 3;
    if(svgen_f->tmp2 > 0){
        svgen_f->VecSector -= 1;
    }

    if(svgen_f->tmp3 > 0){
        svgen_f->VecSector -= 1;
    }

    if(svgen_f->tmp1 < 0){
        svgen_f->VecSector = 7-svgen_f->VecSector;
    }
    //****Calculating temps*****
    if( (svgen_f->VecSector == 1) || (svgen_f->VecSector == 4) ){
        svgen_f->Ta = svgen_f->tmp2;
        svgen_f->Tb = svgen_f->tmp1 - svgen_f->tmp3;
        svgen_f->Tc = -(svgen_f->tmp2);

    }else if( (svgen_f->VecSector == 2) || (svgen_f->VecSector == 5) ){
        svgen_f->Ta = svgen_f->tmp3 + svgen_f->tmp2;
        svgen_f->Tb = svgen_f->tmp1;
        svgen_f->Tc = -(svgen_f->tmp1);

    }else{
        svgen_f->Ta = svgen_f->tmp3;
        svgen_f->Tb = -(svgen_f->tmp3);
        svgen_f->Tc = -((svgen_f->tmp1) + (svgen_f->tmp2));

    }

}//end of function

//*************************************************************
//********************End of file******************************
//*************************************************************
