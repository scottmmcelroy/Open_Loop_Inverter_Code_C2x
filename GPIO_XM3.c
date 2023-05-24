//#############################################################################
//
// FILE:    PWM_XM3.h
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

//************
// Includes
//************
#include "GPIO_XM3.h"
#include "driverlib.h"
#include "device.h"
#include "gpio.h"
#include "PWM_XM3.h"

//************
// Functions
//************

//***
// initGPIO - this initializes the GPIO for controller not in external files.
//***
void initGPIO(void)
{
    //
    // LEDS
    //
    // Enable GPIO outputs on GPIO10,31,34,41,52,65, set it LOW
    /*
    10  CASE-LED
    31  LP-LED-BLUE
    34  LP-LED-RED
    41  LED-G
    52  LED-Y
    65  LED-R
    */

    //timing testing gpio
    GPIO_setPadConfig(95, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO10
    GPIO_setPinConfig(GPIO_95_GPIO95);
    GPIO_writePin(95, 0);                           // Output low
    GPIO_setDirectionMode(95, GPIO_DIR_MODE_OUT); //GPIO95 = output

    //CASELED1 is PWM to do fade
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO10
    GPIO_setPinConfig(GPIO_10_EPWM6A);              // GPIO10 = EPWM6A


    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO31
    GPIO_writePin(31, 0);                           // Output low
    GPIO_setPinConfig(GPIO_31_GPIO31);              // GPIO31 = GPIO31
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);   // GPIO31 = output

    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO34
    GPIO_writePin(34, 0);                           // Output low
    GPIO_setPinConfig(GPIO_34_GPIO34);              // GPIO34 = GPIO34
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);   // GPIO34 = output

    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO41
    GPIO_writePin(41, 0);                           // Output low
    GPIO_setPinConfig(GPIO_41_GPIO41);              // GPIO41 = GPIO41
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_OUT);   // GPIO41 = output

    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO52
    GPIO_writePin(52, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_52_GPIO52);              // GPIO52 = GPIO52
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_OUT);   // GPIO52 = output

    GPIO_setPadConfig(65, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO65
    GPIO_writePin(65, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_65_GPIO65);              // GPIO65 = GPIO65
    GPIO_setDirectionMode(65, GPIO_DIR_MODE_OUT);   // GPIO65 = output

    // PS-CONTROL +15V and -15V
    //
    // Enable GPIO outputs on GPIO124,125, set it LOW

    //
    //124 SHUTDOWN+15V
    //125 SHUTDOWN-15V
    //
    GPIO_setPadConfig(124, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO124
    GPIO_writePin(124, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_124_GPIO124);              // GPIO124 = GPIO124
    GPIO_setDirectionMode(124, GPIO_DIR_MODE_OUT);   // GPIO124 = output

    GPIO_setPadConfig(125, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO125
    GPIO_writePin(125, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_125_GPIO125);              // GPIO125 = GPIO125
    GPIO_setDirectionMode(125, GPIO_DIR_MODE_OUT);   // GPIO125 = output

    //
    // Enable EQEP1 on GPIO's 20,21,99
    /*
    20  QEA_A
    21  QEA_B
    99  QEA_I
    */
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO20
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO21
    GPIO_setPadConfig(99, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO23
    GPIO_setQualificationMode(20, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(21, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(99, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_20_EQEP1A);              // GPIO20 = EQEP1A
    GPIO_setPinConfig(GPIO_21_EQEP1B);              // GPIO21 = EQEP1B
    GPIO_setPinConfig(GPIO_99_EQEP1I);              // GPIO99 = EQEP1I

    //
    // Enable EQEP2 on GPIO's 54,55,57
    /*
    54  QEB_A
    55  QEB_B
    57  QEB_I
    */
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO54
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO55
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO57
    GPIO_setQualificationMode(54, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(55, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(57, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_54_EQEP2A);              // GPIO54 = EQEP1A
    GPIO_setPinConfig(GPIO_55_EQEP2B);              // GPIO55 = EQEP1B
    GPIO_setPinConfig(GPIO_57_EQEP2I);              // GPIO57 = EQEP1I


}

//*****
// TestLEDS - cycle led pins to test functionality
//*****
void TestLEDS(){

    GPIO_togglePin(10);
    GPIO_togglePin(31);
    GPIO_togglePin(34);
    GPIO_togglePin(41);
    GPIO_togglePin(52);
    GPIO_togglePin(65);

    DEVICE_DELAY_US(200000);
}//TestLEDS

//*********
// InitGateDriverGPIO - enable the gate driver outputs
//********
void initGateDriverGPIO()
{
    //
    // Enable PWM1-3 on GPIO0-GPIO5
    /*
    0   AHS-PWM
    1   ALS-PWM
    2   BHS-PWM
    3   BLS-PWM
    4   CHS-PWM
    5   CLS-PWM
    */
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO0
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO1
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO2
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO3
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO4
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO5
    GPIO_setPinConfig(GPIO_0_EPWM1A);               // GPIO0 = PWM1A
    GPIO_setPinConfig(GPIO_1_EPWM1B);               // GPIO1 = PWM1B
    GPIO_setPinConfig(GPIO_2_EPWM2A);               // GPIO2 = PWM2A
    GPIO_setPinConfig(GPIO_3_EPWM2B);               // GPIO3 = PWM2B
    GPIO_setPinConfig(GPIO_4_EPWM3A);               // GPIO4 = PWM3A
    GPIO_setPinConfig(GPIO_5_EPWM3B);               // GPIO5 = PWM3B

    // FAULTS
    //
    // Enable a GPIO input on GPIO6-8,15
    /*
    6   A-FAULT
    7   B-FAULT
    8   C-FAULT
    15  GLOBAL-FAULT *TripZone
    */
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO6
    GPIO_setPinConfig(GPIO_6_GPIO6);                // GPIO6 = GPIO6
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_IN);    // GPIO6 = input
    GPIO_setQualificationMode(6, GPIO_QUAL_ASYNC); // asynch input
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO
    GPIO_setPinConfig(GPIO_7_GPIO7);                // GPIO = GPIO
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);    // GPIO = input
    GPIO_setQualificationMode(7, GPIO_QUAL_ASYNC); // asynch input
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO
    GPIO_setPinConfig(GPIO_8_GPIO8);                // GPIO = GPIO
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);    // GPIO = input
    GPIO_setQualificationMode(8, GPIO_QUAL_ASYNC); // asynch input
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO
    GPIO_setPinConfig(GPIO_15_GPIO15);                // GPIO = GPIO
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);    // GPIO = input
    GPIO_setQualificationMode(15, GPIO_QUAL_ASYNC); // asynch input
    XBAR_setInputPin(XBAR_INPUT1, 15);              // GPIO15 = TZ1

    // RTD TEMPs
    // Digital temperature measurement either PWM or FM using eCAP
    // Enable a GPIO input on GPIO 94,95,97
    /*
    94   A-RTD -> eCAP1
    95   C-RTD -> eCAP3
    97   B-RTD -> eCAP2
    */
    XBAR_setInputPin(XBAR_INPUT7, 94); //config XBAR to route eCAP1 input from GPIO 94
    GPIO_setPinConfig(GPIO_94_GPIO94);
    GPIO_setDirectionMode(94, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(94, GPIO_QUAL_ASYNC);

    //need to disable to use PIN9 for testing timer
    XBAR_setInputPin(XBAR_INPUT9, 95); //config XBAR to route eCAP3 input from GPIO 95
    GPIO_setPinConfig(GPIO_95_GPIO95);
    GPIO_setDirectionMode(95, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(95, GPIO_QUAL_ASYNC);

    XBAR_setInputPin(XBAR_INPUT8, 97); //config XBAR to route eCAP2 input from GPIO 97
    GPIO_setPinConfig(GPIO_97_GPIO97);
    GPIO_setDirectionMode(97, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(97, GPIO_QUAL_ASYNC);

    // GD-Control
    //
    // Enable GPIO outputs on GPIO14,25,26,27,64,64,66,130,131, set it LOW
    /*
    14  C-PSDIS-OUT
    25  A-OCEN-OUT
    26  B-OCEN-OUT
    27  A-LEN-OUT
    63  C-OCEN-OUT
    64  B-LEN-OUT
    66  B-PSDIS-OUT
    130 C-LEN-OUT
    131 A-PSDIS-OUT
     */
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(14, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_14_GPIO14);              // GPIO = GPIO
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(25, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_25_GPIO25);              // GPIO = GPIO
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(26, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_26_GPIO26);              // GPIO = GPIO
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(27, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_27_GPIO27);              // GPIO = GPIO
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(63, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(63, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_63_GPIO63);              // GPIO = GPIO
    GPIO_setDirectionMode(63, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(64, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(64, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_64_GPIO64);              // GPIO = GPIO
    GPIO_setDirectionMode(64, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(66, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(66, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_66_GPIO66);              // GPIO = GPIO
    GPIO_setDirectionMode(66, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(130, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(130, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_130_GPIO130);              // GPIO = GPIO
    GPIO_setDirectionMode(130, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(131, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(131, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_131_GPIO131);              // GPIO = GPIO
    GPIO_setDirectionMode(131, GPIO_DIR_MODE_OUT);   // GPIO = output
}

//****
//phase A gate driver control
//****
void GD_A_PSEnable(void)
{
    GPIO_writePin(131, 1);
}
void GD_A_PSDisable(void)
{
    GPIO_writePin(131, 0);
}
void GD_A_LogicEnable(void)
{
    GPIO_writePin(27, 1);
}
void GD_A_LogicDisable(void)
{
    GPIO_writePin(27, 0);
}
void GD_A_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(25, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(25, 0);
#endif

}
void GD_A_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(25, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(25, 1);
#endif
}

//****
//phase B gate driver control
//****
void GD_B_PSEnable(void)
{
    GPIO_writePin(66, 1);
}
void GD_B_PSDisable(void)
{
    GPIO_writePin(66, 0);
}
void GD_B_LogicEnable(void)
{
    GPIO_writePin(64, 1);
}
void GD_B_LogicDisable(void)
{
    GPIO_writePin(64, 0);
}
void GD_B_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(26, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(26, 0);
#endif
}
void GD_B_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(26, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(26, 1);
#endif
}

//******
//phase C gate driver control
//******
void GD_C_PSEnable(void)
{
    GPIO_writePin(14, 1);
}
void GD_C_PSDisable(void)
{
    GPIO_writePin(14, 0);
}
void GD_C_LogicEnable(void)
{
    GPIO_writePin(130, 1);
}
void GD_C_LogicDisable(void)
{
    GPIO_writePin(130, 0);
}
void GD_C_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(63, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(63, 0);
#endif
}
void GD_C_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(63, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(63, 1);
#endif
}

//******
//ALL gate driver control
//******
void GD_ALL_PSEnable(void)
{
    GD_A_PSEnable();
    GD_B_PSEnable();
    GD_C_PSEnable();
}
void GD_ALL_PSDisable(void)
{
    GD_A_PSDisable();
    GD_B_PSDisable();
    GD_C_PSDisable();
}
void GD_ALL_LogicEnable(void)
{
    GD_A_LogicEnable();
    GD_B_LogicEnable();
    GD_C_LogicEnable();
}
void GD_ALL_LogicDisable(void)
{
    GD_A_LogicDisable();
    GD_B_LogicDisable();
    GD_C_LogicDisable();
}
void GD_ALL_OCEnable(void)
{
    GD_A_OCEnable();
    GD_B_OCEnable();
    GD_C_OCEnable();
}
void GD_ALL_OCDisable(void)
{
    GD_A_OCDisable();
    GD_B_OCDisable();
    GD_C_OCDisable();
}
void GD_ALL_Reset(void)
{
    GD_ALL_OCDisable();
    DEVICE_DELAY_US(100);
    GD_ALL_OCEnable();
}

//read Fault state from gate driver
//return true if there is a fault
//return false if there is NO fault
bool GD_A_getFault(void)
{
    uint32_t val;
    val = GPIO_readPin(6);
    if(val == 0)
        return true;
    return false;
}

bool GD_B_getFault(void)
{
    uint32_t val;
    val = GPIO_readPin(7);
    if(val == 0)
        return true;
    return false;
}

bool GD_C_getFault(void)
{
    uint32_t val;
    val = GPIO_readPin(8);
    if(val == 0)
        return true;
    return false;
}

bool GD_Global_getFault(void)
{
    uint32_t val;
    val = GPIO_readPin(15);
    if(val == 0)
        return true;
    return false;
}

//*************************************************************
//********************End of file******************************
//*************************************************************





