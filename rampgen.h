//#############################################################################
//
// FILE:    rampgen.h
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

#ifndef __RAMPGEN_H__
#define __RAMPGEN_H__

//*******************
// Includes
//*******************
#include "IQmathLib.h"
#include <stdint.h>

//*****************
// Old IQ math based structure
//******************
typedef struct { _iq  Freq; 		// Input: Ramp frequency (pu) 	
		 	     _iq  StepAngleMax;	// Parameter: Maximum step angle (pu)		
	 	 	     _iq  Angle;		// Variable: Step angle (pu)					  
			     _iq  Gain;			// Input: Ramp gain (pu)
			     _iq  Out;  	 	// Output: Ramp signal (pu) ramp
			     _iq  Offset;		// Input: Ramp offset (pu) 				 
}RAMPGEN;

//*****************
// Structure to be used for floating point
//******************
typedef struct { float  Freq;         // Input: Ramp frequency (pu)
	  	  	     float  StepAngleMax; // Parameter: Maximum step angle (pu)
	  	  	     float  Angle;        // Variable: Step angle (pu)
	  	  	     float  Gain;         // Input: Ramp gain (pu)
	  	  	     float  Out;          // Output: Ramp signal (pu) ramp
	  	  	     float  Offset;       // Input: Ramp offset (pu)
}RAMPGEN_F;

/*------------------------------------------------------------------------------
      Object Initializers
------------------------------------------------------------------------------*/                       
#define RAMPGEN_DEFAULTS {0,		\
						  0,		\
						  0,		\
						  _IQ(1),	\
						  0,		\
						  _IQ(1), 	\
                         }

//***********************
//floating point defaults
//***********************
#define RAMPGEN_DEFAULTS_F {  0,   \
                              0,   \
                              0,   \
                              1,   \
                              0,   \
                              1,   \
                           }
//***********
// Global
//***********
extern RAMPGEN_F rg1_f;
//testing values
extern float angleValue;
extern float anlgeMax;
extern float angleMin;
extern uint16_t count;

/*------------------------------------------------------------------------------
	RAMP(Sawtooh) Generator Macro Definition
------------------------------------------------------------------------------*/                                               
#define RG_MACRO(v)									\
													\
/* Compute the angle rate */						\
	v.Angle += _IQmpy(v.StepAngleMax,v.Freq);		\
													\
/* Saturate the angle rate within (-1,1) */			\
	if (v.Angle>_IQ(1.0))							\
		v.Angle -= _IQ(1.0);						\
	else if (v.Angle<_IQ(-1.0))						\
		v.Angle += _IQ(1.0);						\
		v.Out=v.Angle;

// Use the code snippet below if gain/offset needed.													\
/* Compute the ramp output */						\
	v.Out = _IQmpy(v.Angle,v.Gain) + v.Offset;		\
/* Saturate the ramp output within (-1,1) */		\
	if (v.Out>_IQ(1.0))								\
		v.Out -= _IQ(1.0);							\
	else if (v.Out<_IQ(-1.0))						\
		v.Out += _IQ(1.0);

//****************************************
// Function Declarations
// input: pointer to ramp control variable
//****************************************
void rampGen_f(RAMPGEN_F *rg_f);

#endif // __RAMPGEN_H__

//*************************************************************
//********************End of file******************************
//*************************************************************
