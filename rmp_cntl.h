//#############################################################################
//
// FILE:    rmp_cntrl.h
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

#ifndef __RMP_CNTL_H__
#define __RMP_CNTL_H__

//*******************
// Includes
//*******************
#include "IQmathLib.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

//********
//defines
//********
extern bool gpio_value;

//****************
//Old structure for fixed-point DSP
//****************
typedef struct { _iq    TargetValue; 	// Input: Target input (pu)
				 uint32_t RampDelayMax;	// Parameter: Maximum delay rate (Q0) - independently with global Q
		 	 	 _iq    RampLowLimit;	// Parameter: Minimum limit (pu)				  
				 _iq    RampHighLimit;	// Parameter: Maximum limit (pu)
				 uint32_t RampDelayCount; // Variable: Incremental delay (Q0) - independently with global Q
				 _iq    SetpointValue;	// Output: Target output (pu)				 
				 uint32_t EqualFlag;		// Output: Flag output (Q0) - independently with global Q
				 _iq	Tmp;			// Variable: Temp variable
}RMPCNTL;

//****************
//New structure setup for a floating point DSP
//****************
typedef struct { float    TargetValue;    // Input: Target input (pu)
                 uint32_t RampDelayMax; // Parameter: Maximum delay rate (Q0) - independently with global Q
		  	     float    RampLowLimit;   // Parameter: Minimum limit (pu)
		  	     float    RampHighLimit;  // Parameter: Maximum limit (pu)
		  	     uint32_t RampDelayCount; // Variable: Incremental delay (Q0) - independently with global Q
		  	     float    SetpointValue;  // Output: Target output (pu)
		  	     bool     EqualFlag;        // Output: Flag output (Q0) - independently with global Q
                 float    Resolution;      //input: resolution of ramp steps
		  	     float    Tmp;            // Variable: Temp variable
}RMPCNTL_F;

/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/                     
#define RMPCNTL_DEFAULTS {  0, 		 \
                            5,		 \
                           _IQ(-1),  \
                           _IQ(1),   \
                            0,       \
                          	0,       \
                          	0,       \
                          	0,       \
                   		  }

//***********************
//floating point defaults
//***********************
#define RMPCNTL_DEFAULTS_F {  0,   \
                              5,   \
                             -1,   \
                              1,   \
                              0,   \
                              0,   \
                              0,   \
                              0.0000305,   \
                           }

//***********
// Global
//***********
extern RMPCNTL_F rc1_f;

//------------------------------------------------------------------------------
// 	RAMP Controller Macro Definition
// 	Old ControlSuite Macro definition for _IQ math
//------------------------------------------------------------------------------*/
#define RC_MACRO(v)																	\
	v.Tmp = v.TargetValue - v.SetpointValue;										\
/*  0.0000305 is resolution of Q15 */												\
if (_IQabs(v.Tmp) >= _IQ(0.0000305))				    							\
{																					\
	v.RampDelayCount++	;															\
		if (v.RampDelayCount >= v.RampDelayMax)										\
		{																			\
			if (v.TargetValue >= v.SetpointValue)									\
				v.SetpointValue += _IQ(0.0000305);									\
			else																	\
				v.SetpointValue -= _IQ(0.0000305);									\
																					\
			v.SetpointValue=_IQsat(v.SetpointValue,v.RampHighLimit,v.RampLowLimit);	\
			v.RampDelayCount = 0;													\
																					\
		}																			\
}																					\
else v.EqualFlag = 0x7FFFFFFF;


//****************************************
// Function Declarations
// input: pointer to ramp control variable
//****************************************
void rampControl(RMPCNTL_F *rc_f);


#endif // __RMP_CNTL_H__

//*************************************************************
//********************End of file******************************
//*************************************************************

