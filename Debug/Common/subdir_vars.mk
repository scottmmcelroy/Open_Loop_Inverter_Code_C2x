################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../Common/F2837xD_CodeStartBranch.asm \
../Common/F2837xD_DBGIER.asm \
../Common/F2837xD_usDelay.asm 

C_SRCS += \
../Common/F2837xD_Adc.c \
../Common/F2837xD_CpuTimers.c \
../Common/F2837xD_DefaultISR.c \
../Common/F2837xD_Dma.c \
../Common/F2837xD_ECap.c \
../Common/F2837xD_EPwm.c \
../Common/F2837xD_EQep.c \
../Common/F2837xD_Emif.c \
../Common/F2837xD_Gpio.c \
../Common/F2837xD_I2C.c \
../Common/F2837xD_Ipc.c \
../Common/F2837xD_Ipc_Driver.c \
../Common/F2837xD_Ipc_Driver_Lite.c \
../Common/F2837xD_Ipc_Driver_Util.c \
../Common/F2837xD_Mcbsp.c \
../Common/F2837xD_PieCtrl.c \
../Common/F2837xD_PieVect.c \
../Common/F2837xD_Sci.c \
../Common/F2837xD_Spi.c \
../Common/F2837xD_SysCtrl.c \
../Common/F2837xD_TempSensorConv.c \
../Common/F2837xD_Upp.c \
../Common/F2837xD_can.c \
../Common/F2837xD_sci_io.c \
../Common/F2837xD_sdfm_drivers.c \
../Common/F2837xD_struct.c \
../Common/adc.c \
../Common/asysctl.c \
../Common/can.c \
../Common/cla.c \
../Common/clb.c \
../Common/cmpss.c \
../Common/cputimer.c \
../Common/dac.c \
../Common/dcsm.c \
../Common/device.c \
../Common/dma.c \
../Common/ecap.c \
../Common/emif.c \
../Common/epwm.c \
../Common/eqep.c \
../Common/flash.c \
../Common/gpio.c \
../Common/hrpwm.c \
../Common/i2c.c \
../Common/interrupt.c \
../Common/ipc.c \
../Common/mcbsp.c \
../Common/memcfg.c \
../Common/sci.c \
../Common/sdfm.c \
../Common/spi.c \
../Common/sysctl.c \
../Common/upp.c \
../Common/usb.c \
../Common/version.c \
../Common/xbar.c 

C_DEPS += \
./Common/F2837xD_Adc.d \
./Common/F2837xD_CpuTimers.d \
./Common/F2837xD_DefaultISR.d \
./Common/F2837xD_Dma.d \
./Common/F2837xD_ECap.d \
./Common/F2837xD_EPwm.d \
./Common/F2837xD_EQep.d \
./Common/F2837xD_Emif.d \
./Common/F2837xD_Gpio.d \
./Common/F2837xD_I2C.d \
./Common/F2837xD_Ipc.d \
./Common/F2837xD_Ipc_Driver.d \
./Common/F2837xD_Ipc_Driver_Lite.d \
./Common/F2837xD_Ipc_Driver_Util.d \
./Common/F2837xD_Mcbsp.d \
./Common/F2837xD_PieCtrl.d \
./Common/F2837xD_PieVect.d \
./Common/F2837xD_Sci.d \
./Common/F2837xD_Spi.d \
./Common/F2837xD_SysCtrl.d \
./Common/F2837xD_TempSensorConv.d \
./Common/F2837xD_Upp.d \
./Common/F2837xD_can.d \
./Common/F2837xD_sci_io.d \
./Common/F2837xD_sdfm_drivers.d \
./Common/F2837xD_struct.d \
./Common/adc.d \
./Common/asysctl.d \
./Common/can.d \
./Common/cla.d \
./Common/clb.d \
./Common/cmpss.d \
./Common/cputimer.d \
./Common/dac.d \
./Common/dcsm.d \
./Common/device.d \
./Common/dma.d \
./Common/ecap.d \
./Common/emif.d \
./Common/epwm.d \
./Common/eqep.d \
./Common/flash.d \
./Common/gpio.d \
./Common/hrpwm.d \
./Common/i2c.d \
./Common/interrupt.d \
./Common/ipc.d \
./Common/mcbsp.d \
./Common/memcfg.d \
./Common/sci.d \
./Common/sdfm.d \
./Common/spi.d \
./Common/sysctl.d \
./Common/upp.d \
./Common/usb.d \
./Common/version.d \
./Common/xbar.d 

OBJS += \
./Common/F2837xD_Adc.obj \
./Common/F2837xD_CodeStartBranch.obj \
./Common/F2837xD_CpuTimers.obj \
./Common/F2837xD_DBGIER.obj \
./Common/F2837xD_DefaultISR.obj \
./Common/F2837xD_Dma.obj \
./Common/F2837xD_ECap.obj \
./Common/F2837xD_EPwm.obj \
./Common/F2837xD_EQep.obj \
./Common/F2837xD_Emif.obj \
./Common/F2837xD_Gpio.obj \
./Common/F2837xD_I2C.obj \
./Common/F2837xD_Ipc.obj \
./Common/F2837xD_Ipc_Driver.obj \
./Common/F2837xD_Ipc_Driver_Lite.obj \
./Common/F2837xD_Ipc_Driver_Util.obj \
./Common/F2837xD_Mcbsp.obj \
./Common/F2837xD_PieCtrl.obj \
./Common/F2837xD_PieVect.obj \
./Common/F2837xD_Sci.obj \
./Common/F2837xD_Spi.obj \
./Common/F2837xD_SysCtrl.obj \
./Common/F2837xD_TempSensorConv.obj \
./Common/F2837xD_Upp.obj \
./Common/F2837xD_can.obj \
./Common/F2837xD_sci_io.obj \
./Common/F2837xD_sdfm_drivers.obj \
./Common/F2837xD_struct.obj \
./Common/F2837xD_usDelay.obj \
./Common/adc.obj \
./Common/asysctl.obj \
./Common/can.obj \
./Common/cla.obj \
./Common/clb.obj \
./Common/cmpss.obj \
./Common/cputimer.obj \
./Common/dac.obj \
./Common/dcsm.obj \
./Common/device.obj \
./Common/dma.obj \
./Common/ecap.obj \
./Common/emif.obj \
./Common/epwm.obj \
./Common/eqep.obj \
./Common/flash.obj \
./Common/gpio.obj \
./Common/hrpwm.obj \
./Common/i2c.obj \
./Common/interrupt.obj \
./Common/ipc.obj \
./Common/mcbsp.obj \
./Common/memcfg.obj \
./Common/sci.obj \
./Common/sdfm.obj \
./Common/spi.obj \
./Common/sysctl.obj \
./Common/upp.obj \
./Common/usb.obj \
./Common/version.obj \
./Common/xbar.obj 

ASM_DEPS += \
./Common/F2837xD_CodeStartBranch.d \
./Common/F2837xD_DBGIER.d \
./Common/F2837xD_usDelay.d 

OBJS__QUOTED += \
"Common\F2837xD_Adc.obj" \
"Common\F2837xD_CodeStartBranch.obj" \
"Common\F2837xD_CpuTimers.obj" \
"Common\F2837xD_DBGIER.obj" \
"Common\F2837xD_DefaultISR.obj" \
"Common\F2837xD_Dma.obj" \
"Common\F2837xD_ECap.obj" \
"Common\F2837xD_EPwm.obj" \
"Common\F2837xD_EQep.obj" \
"Common\F2837xD_Emif.obj" \
"Common\F2837xD_Gpio.obj" \
"Common\F2837xD_I2C.obj" \
"Common\F2837xD_Ipc.obj" \
"Common\F2837xD_Ipc_Driver.obj" \
"Common\F2837xD_Ipc_Driver_Lite.obj" \
"Common\F2837xD_Ipc_Driver_Util.obj" \
"Common\F2837xD_Mcbsp.obj" \
"Common\F2837xD_PieCtrl.obj" \
"Common\F2837xD_PieVect.obj" \
"Common\F2837xD_Sci.obj" \
"Common\F2837xD_Spi.obj" \
"Common\F2837xD_SysCtrl.obj" \
"Common\F2837xD_TempSensorConv.obj" \
"Common\F2837xD_Upp.obj" \
"Common\F2837xD_can.obj" \
"Common\F2837xD_sci_io.obj" \
"Common\F2837xD_sdfm_drivers.obj" \
"Common\F2837xD_struct.obj" \
"Common\F2837xD_usDelay.obj" \
"Common\adc.obj" \
"Common\asysctl.obj" \
"Common\can.obj" \
"Common\cla.obj" \
"Common\clb.obj" \
"Common\cmpss.obj" \
"Common\cputimer.obj" \
"Common\dac.obj" \
"Common\dcsm.obj" \
"Common\device.obj" \
"Common\dma.obj" \
"Common\ecap.obj" \
"Common\emif.obj" \
"Common\epwm.obj" \
"Common\eqep.obj" \
"Common\flash.obj" \
"Common\gpio.obj" \
"Common\hrpwm.obj" \
"Common\i2c.obj" \
"Common\interrupt.obj" \
"Common\ipc.obj" \
"Common\mcbsp.obj" \
"Common\memcfg.obj" \
"Common\sci.obj" \
"Common\sdfm.obj" \
"Common\spi.obj" \
"Common\sysctl.obj" \
"Common\upp.obj" \
"Common\usb.obj" \
"Common\version.obj" \
"Common\xbar.obj" 

C_DEPS__QUOTED += \
"Common\F2837xD_Adc.d" \
"Common\F2837xD_CpuTimers.d" \
"Common\F2837xD_DefaultISR.d" \
"Common\F2837xD_Dma.d" \
"Common\F2837xD_ECap.d" \
"Common\F2837xD_EPwm.d" \
"Common\F2837xD_EQep.d" \
"Common\F2837xD_Emif.d" \
"Common\F2837xD_Gpio.d" \
"Common\F2837xD_I2C.d" \
"Common\F2837xD_Ipc.d" \
"Common\F2837xD_Ipc_Driver.d" \
"Common\F2837xD_Ipc_Driver_Lite.d" \
"Common\F2837xD_Ipc_Driver_Util.d" \
"Common\F2837xD_Mcbsp.d" \
"Common\F2837xD_PieCtrl.d" \
"Common\F2837xD_PieVect.d" \
"Common\F2837xD_Sci.d" \
"Common\F2837xD_Spi.d" \
"Common\F2837xD_SysCtrl.d" \
"Common\F2837xD_TempSensorConv.d" \
"Common\F2837xD_Upp.d" \
"Common\F2837xD_can.d" \
"Common\F2837xD_sci_io.d" \
"Common\F2837xD_sdfm_drivers.d" \
"Common\F2837xD_struct.d" \
"Common\adc.d" \
"Common\asysctl.d" \
"Common\can.d" \
"Common\cla.d" \
"Common\clb.d" \
"Common\cmpss.d" \
"Common\cputimer.d" \
"Common\dac.d" \
"Common\dcsm.d" \
"Common\device.d" \
"Common\dma.d" \
"Common\ecap.d" \
"Common\emif.d" \
"Common\epwm.d" \
"Common\eqep.d" \
"Common\flash.d" \
"Common\gpio.d" \
"Common\hrpwm.d" \
"Common\i2c.d" \
"Common\interrupt.d" \
"Common\ipc.d" \
"Common\mcbsp.d" \
"Common\memcfg.d" \
"Common\sci.d" \
"Common\sdfm.d" \
"Common\spi.d" \
"Common\sysctl.d" \
"Common\upp.d" \
"Common\usb.d" \
"Common\version.d" \
"Common\xbar.d" 

ASM_DEPS__QUOTED += \
"Common\F2837xD_CodeStartBranch.d" \
"Common\F2837xD_DBGIER.d" \
"Common\F2837xD_usDelay.d" 

C_SRCS__QUOTED += \
"../Common/F2837xD_Adc.c" \
"../Common/F2837xD_CpuTimers.c" \
"../Common/F2837xD_DefaultISR.c" \
"../Common/F2837xD_Dma.c" \
"../Common/F2837xD_ECap.c" \
"../Common/F2837xD_EPwm.c" \
"../Common/F2837xD_EQep.c" \
"../Common/F2837xD_Emif.c" \
"../Common/F2837xD_Gpio.c" \
"../Common/F2837xD_I2C.c" \
"../Common/F2837xD_Ipc.c" \
"../Common/F2837xD_Ipc_Driver.c" \
"../Common/F2837xD_Ipc_Driver_Lite.c" \
"../Common/F2837xD_Ipc_Driver_Util.c" \
"../Common/F2837xD_Mcbsp.c" \
"../Common/F2837xD_PieCtrl.c" \
"../Common/F2837xD_PieVect.c" \
"../Common/F2837xD_Sci.c" \
"../Common/F2837xD_Spi.c" \
"../Common/F2837xD_SysCtrl.c" \
"../Common/F2837xD_TempSensorConv.c" \
"../Common/F2837xD_Upp.c" \
"../Common/F2837xD_can.c" \
"../Common/F2837xD_sci_io.c" \
"../Common/F2837xD_sdfm_drivers.c" \
"../Common/F2837xD_struct.c" \
"../Common/adc.c" \
"../Common/asysctl.c" \
"../Common/can.c" \
"../Common/cla.c" \
"../Common/clb.c" \
"../Common/cmpss.c" \
"../Common/cputimer.c" \
"../Common/dac.c" \
"../Common/dcsm.c" \
"../Common/device.c" \
"../Common/dma.c" \
"../Common/ecap.c" \
"../Common/emif.c" \
"../Common/epwm.c" \
"../Common/eqep.c" \
"../Common/flash.c" \
"../Common/gpio.c" \
"../Common/hrpwm.c" \
"../Common/i2c.c" \
"../Common/interrupt.c" \
"../Common/ipc.c" \
"../Common/mcbsp.c" \
"../Common/memcfg.c" \
"../Common/sci.c" \
"../Common/sdfm.c" \
"../Common/spi.c" \
"../Common/sysctl.c" \
"../Common/upp.c" \
"../Common/usb.c" \
"../Common/version.c" \
"../Common/xbar.c" 

ASM_SRCS__QUOTED += \
"../Common/F2837xD_CodeStartBranch.asm" \
"../Common/F2837xD_DBGIER.asm" \
"../Common/F2837xD_usDelay.asm" 


