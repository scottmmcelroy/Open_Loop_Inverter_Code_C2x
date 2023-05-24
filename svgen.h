//#############################################################################
//
// FILE:    svgen.h
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

#ifndef __SVGEN_H__
#define __SVGEN_H__

//*******************
// Includes
//*******************
#include "IQmathLib.h"
#include <stdint.h>
#include "PWM_XM3.h"

//******************
// Defines
//********************

//*****************
// Old IQ math based structure
//******************
typedef struct 	{ _iq  Ualpha; 			// Input: reference alpha-axis phase voltage 
				  _iq  Ubeta;			// Input: reference beta-axis phase voltage 
				  _iq  Ta;				// Output: reference phase-a switching function		
				  _iq  Tb;				// Output: reference phase-b switching function 
				  _iq  Tc;				// Output: reference phase-c switching function
				  _iq  tmp1;			// Variable: temp variable
				  _iq  tmp2;			// Variable: temp variable
				  _iq  tmp3;			// Variable: temp variable
				  uint16_t VecSector;		// Space vector sector
}SVGEN;

//*****************
// Structure to be used for floating point
//******************
typedef struct  { float  Ualpha;          // Input: reference alpha-axis phase voltage
                  float  Ubeta;           // Input: reference beta-axis phase voltage
                  float  Ta;              // Output: reference phase-a switching function
                  float  Tb;              // Output: reference phase-b switching function
                  float  Tc;              // Output: reference phase-c switching function
                  float  tmp1;            // Variable: temp variable
                  float  tmp2;            // Variable: temp variable
                  float  tmp3;            // Variable: temp variable
                  uint16_t VecSector;       // Space vector sector
                  float  taon;
                  float  tbon;
                  float  tcon;
                  float  t1;
                  float  t2;
}SVGEN_F;

/*-----------------------------------------------------------------------------
Default initializer for the SVGEN object.
-----------------------------------------------------------------------------*/                     
#define SVGEN_DEFAULTS { 0,0,0,0,0 }                       

//***********************
//floating point defaults
//***********************
#define SVGEN_DEFAULTS_F { 0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                         }

//***********
// Global
//***********
extern SVGEN_F svgen1_f;
extern SVGEN svgen1;
extern uint16_t EPwm_TBPRD;
/*------------------------------------------------------------------------------
	Space Vector  Generator (SVGEN) Macro Definition
------------------------------------------------------------------------------*/

#define SVGENDQ_MACRO(v)														\
	v.tmp1= v.Ubeta;															\
	v.tmp2= _IQdiv2(v.Ubeta) + (_IQmpy(_IQ(0.866),v.Ualpha));					\
    v.tmp3= v.tmp2 - v.tmp1;													\
																				\
	v.VecSector=3;																\
	v.VecSector=(v.tmp2> 0)?( v.VecSector-1):v.VecSector;						\
	v.VecSector=(v.tmp3> 0)?( v.VecSector-1):v.VecSector;						\
	v.VecSector=(v.tmp1< 0)?(7-v.VecSector) :v.VecSector;						\
																				\
	if(v.VecSector==1 || v.VecSector==4)                                   \
      {     v.Ta= v.tmp2; 														\
      		v.Tb= v.tmp1-v.tmp3; 												\
      		v.Tc=-v.tmp2;														\
      }								    										\
   																				\
    else if(v.VecSector==2 || v.VecSector==5)                                   \
      {     v.Ta= v.tmp3+v.tmp2; 												\
      		v.Tb= v.tmp1; 														\
      		v.Tc=-v.tmp1;														\
      }																	   		\
   																				\
    else                                                                        \
      {     v.Ta= v.tmp3; 														\
      		v.Tb=-v.tmp3; 														\
      		v.Tc=-(v.tmp1+v.tmp2);												\
      }																	   		\
																				\
//****************************************
// Function
// input: pointer to svgen variable
// Function creates the timing needed for svgen operation
//****************************************
void svGen_f(SVGEN_F *svgen_f);

#endif // __SVGEN_H__

//*************************************************************
//********************End of file******************************
//*************************************************************
