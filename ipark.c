//#############################################################################
//
// FILE:    ipark.c
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

//*******************
// Includes
//*******************
#include "ipark.h"
#include "math.h"

//******************
// Defines
//******************

//***********
// Global
//***********
IPARK_F ipark1_f = IPARK_DEFAULTS_F;

//****************************************
// Function Declarations
// input: pointer to ipark variable
// function also calculates cosine and sine
// excepts angle value from -1 to 1
//****************************************
void invPark_f(IPARK_F *ipark_f){
    //calculate sine/cosine
    //multiply angle (which is -1 to 1) by 2*PI to do COS and SIN
    ipark_f->Cosine = cos(ipark_f->Angle);
    ipark_f->Sine = sin(ipark_f->Angle);
    //calculate and store the alpha/beta
    ipark_f->Alpha = (ipark_f->Ds * ipark_f->Cosine) - (ipark_f->Qs * ipark_f->Sine);
    ipark_f->Beta = (ipark_f->Qs * ipark_f->Cosine) + (ipark_f->Ds * ipark_f->Sine);
}


//*************************************************************
//********************End of file******************************
//*************************************************************
