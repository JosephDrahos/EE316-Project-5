/******************************************************************************
*
* Copyright (C) 2002 - 2019 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/
/*****************************************************************************/
/**
* @file xgpio_intr_tapp_example.c
*
* This file contains a design example using the GPIO driver (XGpio) in an
* interrupt driven mode of operation. This example does assume that there is
* an interrupt controller in the hardware system and the GPIO device is
* connected to the interrupt controller.
*
* This file is used in the Peripheral Tests Application in SDK to include a
* simplified test for gpio interrupts.

* The buttons and LEDs are on 2 separate channels of the GPIO so that interrupts
* are not caused when the LEDs are turned on and off.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date	 Changes
* ----- ---- -------- -------------------------------------------------------
* 2.01a sn   05/09/06 Modified to be used by TestAppGen to include test for
*		      interrupts.
* 3.00a ktn  11/21/09 Updated to use HAL Processor APIs and minor changes
*		      as per coding guidelines.
* 3.00a sdm  02/16/11 Updated to support ARM Generic Interrupt Controller
* 4.1   lks  11/18/15 Updated to use canonical xparameters and
*		      clean up of the comments and code for CR 900381
* 4.3   ms   01/23/17 Modified xil_printf statement in main function to
*                     ensure that "Successfully ran" and "Failed" strings
*                     are available in all examples. This is a fix for
*                     CR-965028.
*
*</pre>
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xgpio.h"
#include "xtmrctr.h"
#include <stdio.h>
#include <stdlib.h>
#include "xparameters.h"
#include "xgpio.h"
#include "xgpiops.h"
#include "xparameters_ps.h"
#include "xil_cache.h"
#include "xbasic_types.h"
#include "xil_printf.h"
#include "Xil_exception.h"
#include "Xscugic.h"

#ifdef XPAR_INTC_0_DEVICE_ID
 #include "xintc.h"
#else
 #include "xscugic.h"
 #include "xil_printf.h"
#endif

/************************** Constant Definitions *****************************/
#ifndef TESTAPP_GEN
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define GPIO_DEVICE_ID		XPAR_GPIO_0_DEVICE_ID
#define GPIO_CHANNEL1		1

//timer int
#define TMRCTR_DEVICE_ID	XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID	XPAR_FABRIC_TMRCTR_0_VEC_ID
#define TMRCTR_MOTOR_DEVICE_ID        XPAR_TMRCTR_1_DEVICE_ID
#define TMRCTR_MOTOR_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_1_VEC_ID

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_GPIO_INTERRUPT_ID	XPAR_INTC_0_GPIO_0_VEC_ID
 #define INTC_DEVICE_ID	XPAR_INTC_0_DEVICE_ID
#else
 #define INTC_GPIO_INTERRUPT_ID	XPAR_FABRIC_AXI_GPIO_0_IP2INTC_IRPT_INTR
 #define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
#endif /* XPAR_INTC_0_DEVICE_ID */

/*
 * The following constants define the positions of the buttons and LEDs each
 * channel of the GPIO
 */
#define GPIO_ALL_LEDS		0xFFFF
#define GPIO_ALL_BUTTONS	0xFFFF
#define TIMER_CNTR_0	 0


/*
 * The following constants define the GPIO channel that is used for the buttons
 * and the LEDs. They allow the channels to be reversed easily.
 */
#define BUTTON_CHANNEL	 1	/* Channel 1 of the GPIO Device */
#define LED_CHANNEL	 2	/* Channel 2 of the GPIO Device */
#define BUTTON_INTERRUPT XGPIO_IR_CH1_MASK  /* Channel 1 Interrupt Mask */

/*
 * The following constant determines which buttons must be pressed at the same
 * time to cause interrupt processing to stop and start
 */
#define INTERRUPT_CONTROL_VALUE 0x7

/*
 * The following constant is used to wait after an LED is turned on to make
 * sure that it is visible to the human eye.  This constant might need to be
 * tuned for faster or slower processor speeds.
 */
#define LED_DELAY	1000000

#endif /* TESTAPP_GEN */

#define INTR_DELAY	0x0FFFFFFF

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_DEVICE_ID	XPAR_INTC_0_DEVICE_ID
 #define INTC		XIntc
 #define INTC_HANDLER	XIntc_InterruptHandler
#else
 #define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
 #define INTC		XScuGic
 #define INTC_HANDLER	XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

#define RESET_VALUE	 0xFF000000

#define PWM_PERIOD              20000000    /* PWM period in (20 ms) */
#define TMRCTR_0                0            /* Timer 0 ID */
#define TMRCTR_1                1            /* Timer 1 ID */
#define CYCLE_PER_DUTYCYCLE     10           /* Clock cycles per duty cycle */
#define MAX_DUTYCYCLE           100          /* Max duty cycle */
#define DUTYCYCLE_DIVISOR       10           /* Duty cycle Divisor */
#define WAIT_COUNT              10000000     //PWM_PERIOD   /* Interrupt wait counter */
/************************** Function Prototypes ******************************/
void GpioHandler(void *CallBackRef);

int GpioIntrExample(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId,
			u16 IntrMask, u32 *DataRead);

int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask);

void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 IntrId, u16 IntrMask);
//timer
int TmrCtrIntrExample(INTC *IntcInstancePtr,
			XTmrCtr *InstancePtr,
			u16 DeviceId,
			u16 IntrId,
			u8 TmrCtrNumber);

static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr,
				XTmrCtr *InstancePtr,
				u16 DeviceId,
				u16 IntrId,
				u8 TmrCtrNumber);

static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);
static void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId);

void LCD_nibble_write(unsigned char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_Setup(void);
void delay(int);
void lcd_output(int, int);

//timer
#ifndef TESTAPP_GEN
INTC InterruptController;  /* The instance of the Interrupt Controller */

XTmrCtr TimerCounterInst;   /* The instance of the Timer Counter */
#endif
/*
 * The following variables are shared between non-interrupt processing and
 * interrupt processing such that they must be global.
 */
volatile int TimerExpired;
/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */
XGpio Gpio; /* The Instance of the GPIO Driver */

INTC Intc; /* The Instance of the Interrupt Controller Driver */

Xuint32 *baseaddr_lcd = (Xuint32 *)XPAR_LCD_0_S00_AXI_BASEADDR; //lcd base address pointer
Xuint32 *baseaddr_gpio = (Xuint32 *)XPAR_AXI_GPIO_0_BASEADDR; //axi gpio pmod base address pointer
Xuint32 *baseaddr_pwm = (Xuint32 *)XPAR_CUSTOM_PWM_0_S00_AXI_BASEADDR; //pwm baseaddr
static u16 GlobalIntrMask; /* GPIO channel mask that is needed by
			    * the Interrupt Handler */

static volatile u32 IntrFlag; /* Interrupt Handler Flag */

int state = 0;
int analog_source = 1;


float ADC_in = 2.4;
/****************************************************************************/
/**
* This function is the main function of the GPIO example.  It is responsible
* for initializing the GPIO device, setting up interrupts and providing a
* foreground loop such that interrupt can occur in the background.
*
* @param	None.
*
* @return
*		- XST_SUCCESS to indicate success.
*		- XST_FAILURE to indicate failure.
*
* @note		None.
*
*****************************************************************************/
#ifndef TESTAPP_GEN
int main(void)
{
	int Status;
	u32 DataRead;
	LCD_Setup();
	lcd_output(state, analog_source);

	print(" Press button to Generate Interrupt\r\n");
	Status = GpioIntrExample(&Intc, &Gpio,
				   GPIO_DEVICE_ID,
				   INTC_GPIO_INTERRUPT_ID,
				   GPIO_CHANNEL1, &DataRead);

	if (Status != XST_SUCCESS) {
			xil_printf("Tmrctr interrupt Example Failed\r\n");
			return XST_FAILURE;
		}
	/*Status = TmrCtrIntrExample(&InterruptController,
						  &TimerCounterInst,
						  TMRCTR_DEVICE_ID,
						  TMRCTR_INTERRUPT_ID,
						  TIMER_CNTR_0);

	if (Status != XST_SUCCESS) {
			xil_printf("Tmrctr interrupt Example Failed\r\n");
			return XST_FAILURE;
		}*/
	//xil_printf("Successfully ran Tmrctr interrupt Example\r\n");
	while(1){

	}

	//GpioDisableIntr(&Intc, &Gpio, INTC_GPIO_INTERRUPT_ID, GPIO_CHANNEL1);
	return XST_SUCCESS;
}
#endif

/******************************************************************************/
/**
*
* This is the entry function from the TestAppGen tool generated application
* which tests the interrupts when enabled in the GPIO
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a reference to the GPIO driver Instance
* @param	DeviceId is the XPAR_<GPIO_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_IP2INTC_IRPT_INTR
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
* @param	DataRead is the pointer where the data read from GPIO Input is
*		returned
*
* @return
*		- XST_SUCCESS if the Test is successful
*		- XST_FAILURE if the test is not successful
*
* @note		None.
*
******************************************************************************/
int GpioIntrExample(INTC *IntcInstancePtr, XGpio* InstancePtr, u16 DeviceId,
			u16 IntrId, u16 IntrMask, u32 *DataRead)
{
	int Status;
	u32 delay;

	/* Initialize the GPIO driver. If an error occurs then exit */
	Status = XGpio_Initialize(InstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = GpioSetupIntrSystem(IntcInstancePtr, InstancePtr, DeviceId,
					IntrId, IntrMask);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	IntrFlag = 0;
	delay = 0;

	/*while(!IntrFlag && (delay < INTR_DELAY)) {
		delay++;
	}*/

	//GpioDisableIntr(IntcInstancePtr, InstancePtr, IntrId, IntrMask);

	*DataRead = IntrFlag;

	return Status;
}


/******************************************************************************/
/**
*
* This function performs the GPIO set up for Interrupts
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a reference to the GPIO driver Instance
* @param	DeviceId is the XPAR_<GPIO_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_IP2INTC_IRPT_INTR
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
*
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
*
* @note		None.
*
******************************************************************************/
int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask)
{
	int Result;

	GlobalIntrMask = IntrMask;

#ifdef XPAR_INTC_0_DEVICE_ID

#ifndef TESTAPP_GEN
	/*
	 * Initialize the interrupt controller driver so that it's ready to use.
	 * specify the device ID that was generated in xparameters.h
	 */
	Result = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Result != XST_SUCCESS) {
		return Result;
	}
#endif /* TESTAPP_GEN */

	/* Hook up interrupt service routine */
	XIntc_Connect(IntcInstancePtr, IntrId,
		      (Xil_ExceptionHandler)GpioHandler, InstancePtr);

	/* Enable the interrupt vector at the interrupt controller */
	XIntc_Enable(IntcInstancePtr, IntrId);

#ifndef TESTAPP_GEN
	/*
	 * Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor
	 */
	Result = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Result != XST_SUCCESS) {
		return Result;
	}
#endif /* TESTAPP_GEN */

#else /* !XPAR_INTC_0_DEVICE_ID */

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Result = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Result != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif /* TESTAPP_GEN */

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
					0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Result = XScuGic_Connect(IntcInstancePtr, IntrId,
				 (Xil_ExceptionHandler)GpioHandler, InstancePtr);
	if (Result != XST_SUCCESS) {
		return Result;
	}

	/* Enable the interrupt for the GPIO device.*/
	XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */

	/*
	 * Enable the GPIO channel interrupts so that push button can be
	 * detected and enable interrupts for the GPIO device
	 */
	XGpio_InterruptEnable(InstancePtr, IntrMask);
	XGpio_InterruptGlobalEnable(InstancePtr);

	/*
	 * Initialize the exception table and register the interrupt
	 * controller handler with the exception table
	 */
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler)INTC_HANDLER, IntcInstancePtr);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/******************************************************************************/
/**
*
* This is the interrupt handler routine for the GPIO for this example.
*
* @param	CallbackRef is the Callback reference for the handler.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void GpioHandler(void *CallbackRef)
{
	XGpio *GpioPtr = (XGpio *)CallbackRef;

	IntrFlag = 1;
	printf("Inside GPIO Handler\n");
	int btn_value = *(baseaddr_gpio);

	//reset
	if(btn_value == 1){
		state = 0;
	}else if(btn_value == 2){ //switch analog source
		if(analog_source == 1){
			analog_source = 0;
		}else{
			analog_source = 1;
		}
	}else if(btn_value == 4){ //enable/disable switch
		if(state == 1){
			state = 2;
		}else{
			state = 1;
		}
	}
	char snum[20];
	printf("Data read: %s\n", itoa(btn_value,snum, 20));
	lcd_output(state, analog_source);

	/* Clear the Interrupt */
	XGpio_InterruptClear(GpioPtr, GlobalIntrMask);

}

/******************************************************************************/
/**
*
* This function disables the interrupts for the GPIO
*
* @param	IntcInstancePtr is a pointer to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a pointer to the GPIO driver Instance
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_VEC
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
*
* @return	None
*
* @note		None.
*
******************************************************************************/
void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 IntrId, u16 IntrMask)
{
	XGpio_InterruptDisable(InstancePtr, IntrMask);
#ifdef XPAR_INTC_0_DEVICE_ID
	XIntc_Disable(IntcInstancePtr, IntrId);
#else
	/* Disconnect the interrupt */
	XScuGic_Disable(IntcInstancePtr, IntrId);
	XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
	return;
}

/*****************************************************************************/
/**
* This function does a minimal test on the timer counter device and driver as a
* design example.  The purpose of this function is to illustrate how to use the
* XTmrCtr component.  It initializes a timer counter and then sets it up in
* compare mode with auto reload such that a periodic interrupt is generated.
*
* This function uses interrupt driven mode of the timer counter.
*
* @param	IntcInstancePtr is a pointer to the Interrupt Controller
*		driver Instance
* @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance
* @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_INTERRUPT_INTR
*		value from xparameters.h
* @param	TmrCtrNumber is the number of the timer to which this
*		handler is associated with.
*
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
*
* @note		This function contains an infinite loop such that if interrupts
*		are not working it may never return.
*
*****************************************************************************/
int TmrCtrIntrExample(INTC *IntcInstancePtr,
			XTmrCtr *TmrCtrInstancePtr,
			u16 DeviceId,
			u16 IntrId,
			u8 TmrCtrNumber)
{
	int Status;
	int LastTimerExpired = 0;


	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly, use the 1st timer in the device (0)
	 */
	Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TmrCtrNumber);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the timer counter to the interrupt subsystem such that
	 * interrupts can occur.  This function is application specific.
	 */
	Status = TmrCtrSetupIntrSystem(IntcInstancePtr,
					TmrCtrInstancePtr,
					DeviceId,
					IntrId,
					TmrCtrNumber);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the timer counter that will be called from the
	 * interrupt context when the timer expires, specify a pointer to the
	 * timer counter driver instance as the callback reference so the
	 * handler is able to access the instance data
	 */

	XTmrCtr_SetHandler(TmrCtrInstancePtr, TimerCounterHandler,
					   TmrCtrInstancePtr);

	/*
	 * Enable the interrupt of the timer counter so interrupts will occur
	 * and use auto reload mode such that the timer counter will reload
	 * itself automatically and continue repeatedly, without this option
	 * it would expire once only
	 */
	XTmrCtr_SetOptions(TmrCtrInstancePtr, TmrCtrNumber,
				XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);

	/*
	 * Set a reset value for the timer counter such that it will expire
	 * eariler than letting it roll over from 0, the reset value is loaded
	 * into the timer counter when it is started
	 */
	XTmrCtr_SetResetValue(TmrCtrInstancePtr, TmrCtrNumber, RESET_VALUE);

	/*
	 * Start the timer counter such that it's incrementing by default,
	 * then wait for it to timeout a number of times
	 */
	XTmrCtr_Start(TmrCtrInstancePtr, TmrCtrNumber);

	while (TimerExpired < 3) {
		/*
		 * Wait for the first timer counter to expire as indicated
		 * by the shared variable which the handler will increment
		 */
		while (TimerExpired == LastTimerExpired) {
		}
		LastTimerExpired = TimerExpired;

		/*
		 * If it has expired a number of times, then stop the
		 * timer counter and stop this example

		if (TimerExpired == 3) {

			XTmrCtr_Stop(TmrCtrInstancePtr, TmrCtrNumber);
			break;
		}*/
	}



	//TmrCtrDisableIntr(IntcInstancePtr, IntrId);
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This function is the handler which performs processing for the timer counter.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized.  It is called when the timer counter expires
* if interrupts are enabled.
*
* This handler provides an example of how to handle timer counter interrupts
* but is application specific.
*
* @param	CallBackRef is a pointer to the callback function
* @param	TmrCtrNumber is the number of the timer to which this
*		handler is associated with.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
	XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;
	u8  DutyCycle;
	float DutyCycle_percent;
//	u8  NoOfCycles;
	int  Div;
	u32 Period;
	u32 HighTime;
//	u64 WaitCount;
	/*
	 * Check if the timer counter has expired, checking is not necessary
	 * since that's the reason this function is executed, this just shows
	 * how the callback reference can be used as a pointer to the instance
	 * of the timer counter that expired, increment a shared variable so
	 * the main thread of execution can see the timer expired
	 */

	xil_printf("Tmrctr interrupt Example in handler\r\n");


	/*
	 * We start with the fixed divisor and after every CYCLE_PER_DUTYCYCLE
	 * decrement the divisor by 1, as a result Duty cycle increases
	 * proportionally. This is done until duty cycle is reached upto
	 * MAX_DUTYCYCLE
	 */
	//Div = DUTYCYCLE_DIVISOR;
	if(state == 1){ //enabled
		Div = ADC_in*300;
		*(baseaddr_pwm + 1) = ADC_in*757575;
	}else{
		Div = 0;
		*(baseaddr_pwm + 1) = ADC_in*0;
	}
	/* Configure PWM */
	/* Disable PWM for reconfiguration */
	XTmrCtr_PwmDisable(InstancePtr);//TmrCtrInstancePtr);

	/* Configure PWM */
	Period = PWM_PERIOD;
	//HighTime = (0.025+0.001*Div)*PWM_PERIOD; // (0.025 + .1*ADC/2047)*PWM_PERIOD;
	//(ADC_in/2047)*PWM_PERIOD;

	//Buzzer
	Period = 1000000 - (ADC_in/2047)*666666;
	HighTime = Period/2;

	DutyCycle = XTmrCtr_PwmConfigure(InstancePtr, Period, HighTime);

	DutyCycle_percent = ((float)HighTime /(float)Period)*100;
	printf("PWM percent Duty Cycle = %5.2F\r\n", DutyCycle_percent);
	printf("Button data: %i\n", (int)*(baseaddr_gpio));
	/* Enable PWM */
	XTmrCtr_PwmEnable(InstancePtr);//TmrCtrInstancePtr);


}


/*****************************************************************************/
/**
* This function setups the interrupt system such that interrupts can occur
* for the timer counter. This function is application specific since the actual
* system may or may not have an interrupt controller.  The timer counter could
* be directly connected to a processor without an interrupt controller.  The
* user should modify this function to fit the application.
*
* @param	IntcInstancePtr is a pointer to the Interrupt Controller
*		driver Instance.
* @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance.
* @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
*		xparameters.h.
* @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_VEC_ID
*		value from xparameters.h.
* @param	TmrCtrNumber is the number of the timer to which this
*		handler is associated with.
*
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE.
*
* @note		This function contains an infinite loop such that if interrupts
*		are not working it may never return.
*
******************************************************************************/
static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr,
				 XTmrCtr *TmrCtrInstancePtr,
				 u16 DeviceId,
				 u16 IntrId,
				 u8 TmrCtrNumber)
{
	 int Status;

#ifdef XPAR_INTC_0_DEVICE_ID
#ifndef TESTAPP_GEN
	/*
	 * Initialize the interrupt controller driver so that
	 * it's ready to use, specify the device ID that is generated in
	 * xparameters.h
	 */
	Status = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;
#endif
	/*
	 * Connect a device driver handler that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the
	 * specific interrupt processing for the device
	 */
	Status = XIntc_Connect(IntcInstancePtr, IntrId,
				(XInterruptHandler)XTmrCtr_InterruptHandler,
				(void *)TmrCtrInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

#ifndef TESTAPP_GEN
	/*
	 * Start the interrupt controller such that interrupts are enabled for
	 * all devices that cause interrupts, specific real mode so that
	 * the timer counter can cause interrupts thru the interrupt controller.
	 */
	Status = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif

	/*
	 * Enable the interrupt for the timer counter
	 */
	XIntc_Enable(IntcInstancePtr, IntrId);

#else

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif /* TESTAPP_GEN */

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
					0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, IntrId,
				 (Xil_ExceptionHandler)XTmrCtr_InterruptHandler,
				 TmrCtrInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the Timer device.
	 */
	XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */


#ifndef TESTAPP_GEN
	/*
	 * Initialize the exception table.
	 */
	Xil_ExceptionInit();

	/*
	 * Register the interrupt controller handler with the exception table.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
					(Xil_ExceptionHandler)
					INTC_HANDLER,
					IntcInstancePtr);

	/*
	 * Enable non-critical exceptions.
	 */
	Xil_ExceptionEnable();

#endif
	return XST_SUCCESS;
}


/******************************************************************************/
/**
*
* This function disables the interrupts for the Timer.
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance.
* @param	IntrId is XPAR_<INTC_instance>_<Timer_instance>_VEC_ID
*		value from xparameters.h.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId)
{
	/*
	 * Disable the interrupt for the timer counter
	 */
#ifdef XPAR_INTC_0_DEVICE_ID
	XIntc_Disable(IntcInstancePtr, IntrId);
#else
	/* Disconnect the interrupt */
	XScuGic_Disable(IntcInstancePtr, IntrId);
	XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
}

//lcd initialzation sequence
void LCD_Setup(){
	delay(30);                /* initialization sequence */
	LCD_nibble_write(0x30, 0);
	delay(10);
	LCD_nibble_write(0x30, 0);
	delay(1);
	LCD_nibble_write(0x30, 0);
	delay(1);
	LCD_nibble_write(0x20, 0);  /* use 4-bit data mode */
	delay(1);
    LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          /* move cursor right */
    LCD_command(0x06);
    LCD_command(0x01);          /* clear screen, move cursor to home */
    LCD_command(0x0E);          /* turn on display, cursor blinking */

}

//lcd nibble write with adjusted port locations
void LCD_nibble_write(unsigned char data, unsigned char control)
{
    data &= 0xF0;       /* clear lower nibble for control */
    control &= 0x0F;	    /* clear upper nibble for data */


    *(baseaddr_lcd+1) = ((data | control | 0x8) >> 2) ;
    delay(1);
    *(baseaddr_lcd+1) &= ((data | control) >> 2);
	delay(1);
}

void LCD_command(unsigned char command)
{
    LCD_nibble_write(command & 0xF0, 0);   /* upper nibble first */
    LCD_nibble_write(command << 4, 0);     /* then lower nibble */

    if (command < 4)
        delay(4);         /* commands 1 and 2 need up to 1.64ms */
    else
        delay(1);         /* all others 40 us */
}

///function to write data to lcd
void LCD_data(unsigned char data)
{
    LCD_nibble_write(data & 0xF0, 0x4);    /* upper nibble first */
    LCD_nibble_write(data << 4, 0x4);      /* then lower nibble  */

    delay(1);
}

//function for n ms delay
void delay(int n) {
	    int i, j;
	    for(i = 0 ; i < n; i++)
	        for (j = 0; j < 125000; j++) {}
	}

void lcd_output(int state, int analog_source){
	if(state == 0){
		LCD_command(0x01);
		LCD_command(0x14);
		LCD_command(0x14);
		LCD_command(0x14);
		LCD_command(0x14);
		LCD_command(0x14);
		LCD_data(82); //R
		LCD_data(101); //e
		LCD_data(115); //s
		LCD_data(101); //e
		LCD_data(116); //t
		}else if(state == 1){
			LCD_command(0x01);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_data(69); //E
			LCD_data(110); //n
			LCD_data(97); //a
			LCD_data(98); //b
			LCD_data(108); //l
			LCD_data(101); //e

			LCD_command(0xC0); //2nd line
			if(analog_source == 0){
				LCD_command(0x14);
				LCD_data(80); //P
				LCD_data(111); //o
				LCD_data(116); //t
				LCD_data(101); //e
				LCD_data(110); //n
				LCD_data(116); //t
				LCD_data(105); //i
				LCD_data(111); //o
				LCD_data(109); //m
				LCD_data(101); //e
				LCD_data(116); //t
				LCD_data(101); //e
				LCD_data(114); //r
			}else if (analog_source == 1){
				LCD_command(0x14);
				LCD_data(80); //P
				LCD_data(104); //h
				LCD_data(111); //o
				LCD_data(116); //t
				LCD_data(111); //o
				LCD_data(114); //r
				LCD_data(101); //e
				LCD_data(115); //s
				LCD_data(105); //i
				LCD_data(115); //s
				LCD_data(116); //t
				LCD_data(101); //e
				LCD_data(114); //r
			}

		}else if(state == 2){
			LCD_command(0x01);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_command(0x14);
			LCD_data(68); //D
			LCD_data(105); //i
			LCD_data(115); //s
			LCD_data(97); //a
			LCD_data(98); //b
			LCD_data(108); //l
			LCD_data(101); //e

			LCD_command(0xC0); //2nd line
			if(analog_source == 0){
				LCD_command(0x14);
				LCD_data(80); //P
				LCD_data(111); //o
				LCD_data(116); //t
				LCD_data(101); //e
				LCD_data(110); //n
				LCD_data(116); //t
				LCD_data(105); //i
				LCD_data(111); //o
				LCD_data(109); //m
				LCD_data(101); //e
				LCD_data(116); //t
				LCD_data(101); //e
				LCD_data(114); //r
			}else if (analog_source == 1){
				LCD_command(0x14);
				LCD_data(80); //P
				LCD_data(104); //h
				LCD_data(111); //o
				LCD_data(116); //t
				LCD_data(111); //o
				LCD_data(114); //r
				LCD_data(101); //e
				LCD_data(115); //s
				LCD_data(105); //i
				LCD_data(115); //s
				LCD_data(116); //t
				LCD_data(101); //e
				LCD_data(114); //r
			}
		}else{

		}
}


