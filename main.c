/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"
#include "../FreeRTOS_Source/include/event_groups.h"

/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/*-----------------------------------------------------------*/

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );
void vTrafficLightCallback(void* arg);

#include <stdio.h>
#include <time.h>
#include <stdlib.h>

static unsigned int get_next_light_colour(unsigned int);
static void enumerate_traffic(uint32_t traffic_to_display);
uint32_t shift_traffic(int add_car);

static void traffic_flow_task();
static void traffic_creator_task();
static void traffic_light_task();
static void traffic_display_task();

static void prvSetupHardware( void );
static void setUpGPIO();
static void setUpADC();

static unsigned int get_pot_val();

static void Traffic_Flow_Task();
static void Traffic_Creator_Task();
static void Traffic_Light_Task();
static void Traffic_Display_Task();

void sleep(unsigned int mseconds);

GPIO_InitTypeDef  GPIO_InitStructureC;

int MAX_RANGE_POT_VAL = 1500;
int MED_RANGE_POT_VAL = 1000;
int MIN_RANGE_POT_VAL = 500;

int MAX_TRAFFIC = 3;
int MED_TRAFFIC = 2;
int MIN_TRAFFIC = 1;
int NO_TRAFFIC = 0;

int MAX_TRAFFIC_DELAY = 2000;
int MED_TRAFFIC_DELAY = 4000;
int MIN_TRAFFIC_DELAY = 6000;

int GREEN_LIGHT = 0b001;
int YELLOW_LIGHT = 0b010;
int RED_LIGHT = 0b100;

unsigned int ADD_CAR = 1;
unsigned int DO_NOT_ADD_CAR = 0;

unsigned int CHANGE_TRAFFIC_LIGHT = 1;
unsigned int DO_NOT_CHANGE_TRAFFIC_LIGHT = 0;

xQueueHandle Traffic_Flow_Queue = 0;
xQueueHandle Traffic_Light_Delay_Queue = 0;
xQueueHandle Traffic_Display_Queue = 0;
xQueueHandle Prev_Traffic_Queue = 0;

TimerHandle_t Traffic_Light_Timer;

xQueueHandle Change_Light_Flag;
SemaphoreHandle_t Flow_Semaphore;

static void setUpADC() {
	/* PA1 -> POT */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructureC.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructureC.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructureC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructureC.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOC, &GPIO_InitStructureA);

	 ADC_DeInit();

	// ADC1 Structure
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	// Initializing ADC
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	// Set Channel for ADC
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles);
}

static void setUpGPIO(){
	/* PC6 -> A, B*/
	/* PC8 -> CLR*/
	/* PC7 -> CLK */

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructureC.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructureC.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructureC.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureC.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructureC.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &GPIO_InitStructureC);
}

static unsigned int get_pot_val() {
	ADC_SoftwareStartConv(ADC1);

	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	return ADC_GetConversionValue(ADC1);
}

int main(void) {
	srand(time(NULL));

	setUpGPIO();
	setUpADC();

	Traffic_Flow_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	Traffic_Light_Delay_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	Traffic_Display_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	Prev_Traffic_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

	Traffic_Light_Timer = xTimerCreate( "Traffic_Light_Timer", pdMS_TO_TICKS( 5000 ), pdFALSE, ( void * ) 0, vTrafficLightCallback);

	Change_Light_Flag =  xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

	// Start tasks
	xTaskCreate( Traffic_Flow_Task, "Traffic_Flow", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Creator_Task, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Light_Task, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Display_Task, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// set the initial traffic value to 0 i.e. no traffic
	int initial_traffic = 0;
	xQueueSend(Prev_Traffic_Queue, &initial_traffic, 1000);

	xTimerStart(Traffic_Light_Timer, 100);

	//Start tasks and timers
	 vTaskStartScheduler();
}

static void Traffic_Flow_Task() {
	uint32_t flow_rate;
	while (1) {
		uint16_t pot_val = get_pot_val();

         if(pot_val >= MAX_RANGE_POT_VAL) {
			// set the traffic flow to max
			flow_rate = MAX_TRAFFIC;
		}
		else if (pot_val > MED_RANGE_POT_VAL) {
			// set the traffic flow to med
			flow_rate = MED_TRAFFIC;
		}
		else if (pot_val > MIN_RANGE_POT_VAL) {
			// set the traffic flow to min
			flow_rate = MIN_TRAFFIC;
		}
		else {
			// set no traffic
			flow_rate = NO_TRAFFIC;
		}
		// Send Flow Rate to queue twice
		for (int k = 0; k < 2; k++) {
			xQueueSend(Traffic_Flow_Queue, &flow_rate, 1000);
		}
		vTaskDelay(1000);
	}


}

static void Traffic_Creator_Task() {
	uint32_t flow_rate;

	int rounds_since_last_added_car = 0b00;
	uint32_t traffic_to_display;

	while (1) {
		if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 500) ) {

			if (flow_rate >= MAX_TRAFFIC) {
				if (rounds_since_last_added_car > 1 || rand() % 2 == 0) {
					traffic_to_display = shift_traffic(ADD_CAR);
					rounds_since_last_added_car = 0;
				}
				else {
					traffic_to_display = shift_traffic(DO_NOT_ADD_CAR);
					rounds_since_last_added_car++ ;
				}
			}
			else if (flow_rate >= MED_TRAFFIC || rand() % 3 == 0) {
				if (rounds_since_last_added_car > 4) {
					traffic_to_display = shift_traffic(ADD_CAR);
					rounds_since_last_added_car = 0;
				}
				else {
					traffic_to_display = shift_traffic( DO_NOT_ADD_CAR);
					rounds_since_last_added_car++ ;
				}
			}
			else if(flow_rate >= MIN_TRAFFIC || rand() % 5 == 0) {
				if (rounds_since_last_added_car > 8){
					traffic_to_display = shift_traffic(ADD_CAR);
					rounds_since_last_added_car = 0;

				}
				else {
					traffic_to_display = shift_traffic(DO_NOT_ADD_CAR);
					rounds_since_last_added_car++ ;
				}

			}
			else {
				traffic_to_display = shift_traffic(DO_NOT_ADD_CAR);
				rounds_since_last_added_car++ ;
			}
			
			xQueueSend(Traffic_Display_Queue, &traffic_to_display, 1000);
		}

		vTaskDelay(1000);
	}
}
static void Traffic_Light_Task() {
	uint32_t flow_rate;

	int light_delay = 5000;

	while(1)
	{
		// Get flow rate from queue
		if (xQueueReceive(Traffic_Flow_Queue, &flow_rate, 500)) {
			// light change every 2 seconds for max traffic
			if (xTimerIsTimerActive(Traffic_Light_Timer) == pdFALSE ) {
				// update flow rate to minimum value to avoid very long periods for small flow rates
				if (flow_rate < 750) {
					flow_rate = 750;
				}
				light_delay = 8000 / (flow_rate/500);
				// update the traffic light period
				xTimerChangePeriod(Traffic_Light_Timer, pdMS_TO_TICKS(light_delay), 0);
			}

		}
		vTaskDelay(1000);
	}

}
static void Traffic_Display_Task() {
	uint32_t traffic;
	
	while (1) {
		if (xQueueReceive(Traffic_Display_Queue, &traffic, 500)) {
			enumerate_traffic(traffic & 0x3FFFFF);
		}
		vTaskDelay(1000);
	}

}

uint32_t shift_traffic(int add_car) {
	uint32_t prev_traffic;

	if (xQueueReceive(Prev_Traffic_Queue, &prev_traffic, 500)) {
		int change_light;
		
		// checks if the traffic light should be changed
		if(!xQueueReceive(Change_Light_Flag, &change_light, 500)) {
			change_light = 0;
		}
		// seperate traffic into logical groups
		unsigned int first_eight_cars = prev_traffic & 0xFF;
		unsigned int middle_three_cars = (prev_traffic >> 8 & 0x3);
		unsigned int last_eight_cars = (prev_traffic >> 14) & 0xFF;

		unsigned int light_colour = prev_traffic >> 11 & 0x7;

		unsigned int new_light_colour = light_colour;
		// transition the light colour if light should be changed or on first round (when traffic is 0)
		if(change_light == 1 || light_colour == 0) {
			new_light_colour = get_next_light_colour(light_colour);
		}

		//shift the last eight bits
		unsigned int shifted_last_eight_cars = ((last_eight_cars << 1) | (prev_traffic >> 10 & 0x1)) & 0xFF;
		// shift the middle three cars
		unsigned int shifted_middle_three_cars = (middle_three_cars << 1);

		unsigned int shifted_first_eight_cars = 0;
		
		// shift the first eight lights accordingly if red light
		if(light_colour == RED_LIGHT) {
			// set prev_bit to 1, indicating that the most significant bit cannot be shifted through light
			uint8_t prev_bit = 1;

			int shift = 7;
			// loop from MSB to LSB to determine whether each bit can be shifted or not
			while (shift >= 0) {
				// if previous bit is 0, and current bit is 1, then bit can be shifted by 1
				if ((((first_eight_cars >> shift) & 1) == 1) && (prev_bit == 0)) {
					shifted_first_eight_cars = shifted_first_eight_cars |= (1 << shift + 1);
					prev_bit = 1;
				}
				// if previous bit is 1, and current bit is 1, can't be shifted so write back to same position
				else if((((first_eight_cars >> shift) & 1) == 1) && prev_bit == 1) {
					shifted_first_eight_cars = shifted_first_eight_cars |= (1 << shift);
					prev_bit = 1;
				}
				else {
					prev_bit = 0;
				}
				shift--;
			}
			// add a car if there's room, i.e. LSB is 0
			if (add_car == 1) {
				if((shifted_first_eight_cars & 1) == 0) {
					shifted_first_eight_cars |= 1;
				}
			}
		}
		// green or yellow light
		else {
			shifted_first_eight_cars = first_eight_cars << 1 | add_car;
			shifted_middle_three_cars = shifted_middle_three_cars | (first_eight_cars >> 8 & 0x1);
		}

		uint32_t updated_traffic;
		updated_traffic = (((shifted_last_eight_cars | 0x00) << 14) | (new_light_colour << 11)| (shifted_middle_three_cars << 8)| (shifted_first_eight_cars)) & 0x3fffff;
		
		// write updated traffic to queue to be used as a history for next round
		xQueueSend(Prev_Traffic_Queue, &updated_traffic, 1000);

		return updated_traffic;
	}
	// should never hit this line
	return 0;
}



static void enumerate_traffic(uint32_t traffic_to_display) {
	// clear -> 0
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);

	unsigned int number_lights = 22;
	unsigned int current_light = 0;
	
	// send data to shift register from LSB to MSB
	while(current_light <= number_lights) {
		unsigned int emulated_light = (traffic_to_display >> (number_lights - current_light)) & 0x1;

		// write emulated_light val to PC6
		 GPIO_WriteBit(GPIOC, GPIO_Pin_6, emulated_light);

		// clock tick
		GPIO_ResetBits(GPIOC, GPIO_Pin_7);
		sleep(1000);
		GPIO_SetBits(GPIOC, GPIO_Pin_7);

		current_light = current_light + 1;
	}
}

static unsigned int get_next_light_colour(unsigned int light_colour) {
	if (light_colour == GREEN_LIGHT || light_colour == YELLOW_LIGHT) {
		return light_colour << 1;
	}
	// if red, return green
	return GREEN_LIGHT;
}

void sleep(unsigned int mseconds)
{
	int j = 0;
	while(++j < mseconds);

}


/*-----------------------------------------------------------*/
void vTrafficLightCallback(void* arg) {
	int change_traffic = 1;
	xQueueSend(Change_Light_Flag, &change_traffic, 1000);
}

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

