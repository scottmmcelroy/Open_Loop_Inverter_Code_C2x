//#############################################################################
//
// FILE:    ipark.h
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

#ifndef __IPARK_H__
#define __IPARK_H__

//*******************
// Includes
//*******************
#include "IQmathLib.h"

//******************
// Defines
//********************

//*****************
// Old IQ math based structure
//******************
typedef struct {  _iq  Alpha;  		// Output: stationary d-axis stator variable
				  _iq  Beta;		// Output: stationary q-axis stator variable
				  _iq  Angle;		// Input: rotating angle (pu)
				  _iq  Ds;			// Input: rotating d-axis stator variable
				  _iq  Qs;			// Input: rotating q-axis stator variable
				  _iq  Sine;		// Input: Sine term
				  _iq  Cosine;		// Input: Cosine term
}IPARK;

//*****************
// Structure to be used for floating point
//******************
typedef struct {  float  Alpha;       // Output: stationary d-axis stator variable
                  float  Beta;        // Output: stationary q-axis stator variable
                  float  Angle;       // Input: rotating angle (pu)
                  float  Ds;          // Input: rotating d-axis stator variable
                  float  Qs;          // Input: rotating q-axis stator variable
                  float  Sine;        // Input: Sine term
                  float  Cosine;      // Input: Cosine term
}IPARK_F;


/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/                     
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
                          0, \
              		   }

//***********************
//floating point defaults
//***********************
#define IPARK_DEFAULTS_F {  0, \
                            0, \
                            0, \
                            0, \
                            0, \
                            0, \
                            0, \
                         }

//***********
// Global
//***********
extern IPARK_F ipark1_f;

/*------------------------------------------------------------------------------
	Inverse PARK Transformation Macro Definition
------------------------------------------------------------------------------*/
#define IPARK_MACRO(v)										\
															\
v.Alpha = _IQmpy(v.Ds,v.Cosine) - _IQmpy(v.Qs,v.Sine);		\
v.Beta  = _IQmpy(v.Qs,v.Cosine) + _IQmpy(v.Ds,v.Sine);


//****************************************
// Function Declarations
// input: pointer to ipark variable
//****************************************
void invPark_f(IPARK_F *ipark_f);


#endif // __IPARK_H__

//*************************************************************
//********************End of file******************************
//*************************************************************
