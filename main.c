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

/// Traffic 10101010 001 (green yellow) 010 01010101
// 01010100 010 (yellow light) 100 10101011
// 01010100 010 			   100 10101011
// green light 001, yellow 010, red 100
// value in decimal is
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

static unsigned int get_next_light_colour(unsigned int);
static void enumerate_traffic(unsigned int traffic_to_display);
unsigned int shift_traffic(unsigned int prev_traffic, int change_light, int add_car);

static void traffic_flow_task();
static void traffic_creator_task();
static void traffic_light_task();
static void traffic_display_task();

unsigned int traffic = 2787925;

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
GPIO_InitTypeDef  GPIO_InitStructureA;

// set these vals
int MAX_RANGE_POT_VAL = 3;
int MED_RANGE_POT_VAL = 2;
int MIN_RANGE_POT_VAL = 1;

int MAX_TRAFFIC = 3;
int MED_TRAFFIC = 2;
int MIN_TRAFFIC = 1;
int NO_TRAFFIC = 0;

int MAX_TRAFFIC_DELAY = 2000;
int MED_TRAFFIC_DELAY = 4000;
int MIN_TRAFFIC_DELAY = 6000;

unsigned int ADD_CAR = 1;
unsigned int DO_NOT_ADD_CAR = 0;

unsigned int CHANGE_TRAFFIC_LIGHT = 1;
unsigned int DO_NOT_CHANGE_TRAFFIC_LIGHT = 0;


xQueueHandle Traffic_Flow_Queue = 0;
xQueueHandle Traffic_Light_Delay_Queue = 0;
xQueueHandle Traffic_Display_Queue = 0;


EventGroupHandle_t Change_Light_Flag;
SemaphoreHandle_t Flow_Semaphore;

static void setUpADC() {

}

static void setUpGPIO(){
	/* PC6 -> A, B*/
	/* PC8 -> CLR*/
	/* PC7 -> CLK */

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructureC.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructureC.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructureC.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureC.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructureC.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &GPIO_InitStructureC);
}

static unsigned int get_pot_val() {
	return 1;
}


int main(void) {
	srand(time(NULL));

	setUpGPIO();

	Traffic_Flow_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	Traffic_Light_Delay_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	Traffic_Display_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

	// Flow_Semaphore = xSemaphoreCreateCount( 2, 0 );
	Change_Light_Flag = xEventGroupCreate();

	// Start tasks
	xTaskCreate( Traffic_Flow_Task, "Traffic_Flow", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Creator_Task, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Light_Task, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Display_Task, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	//Start tasks and timers
	vTaskStartScheduler();
//	enumerate_traffic(0b1111111100000000000000);
}

static void Traffic_Flow_Task() {
	uint8_t flow_rate;
	while (1) {
		unsigned int pot_val = get_pot_val();

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

	}

}

static void Traffic_Creator_Task() {
	uint8_t flow_rate;

	int rounds_since_last_added_car = 0;
	unsigned int prev_traffic = 0;
	uint32_t traffic_to_display;

	while (1) {
		// if ( xSemaphoreTask(Flow_Semaphore, (TickType_t) 10) == pdTRUE ) {
			if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 500) ) {
				// need to get notification on changing traffic light

				unsigned int change_light = xEventGroupGetBits(Change_Light_Flag);

				if (flow_rate >= MAX_TRAFFIC) {
					if (rounds_since_last_added_car > 1 || rand() % 2 == 0) {
						traffic_to_display = shift_traffic(prev_traffic, change_light, ADD_CAR);
						rounds_since_last_added_car = 0;
					}
					else {
						traffic_to_display = shift_traffic(prev_traffic, change_light, DO_NOT_ADD_CAR);
						rounds_since_last_added_car++ ;
					}
				}
				else if (flow_rate >= MED_TRAFFIC) {
					if (rounds_since_last_added_car > 4 || rand() % 3 == 0) {
						traffic_to_display = shift_traffic(prev_traffic, change_light, ADD_CAR);
						rounds_since_last_added_car = 0;
					}
					else {
						traffic_to_display = shift_traffic(prev_traffic, change_light, DO_NOT_ADD_CAR);
						rounds_since_last_added_car++ ;
					}
				}
				else {
					if (rounds_since_last_added_car > 8 || rand() % 5 == 0){
						traffic_to_display = shift_traffic(prev_traffic, change_light, ADD_CAR);
						rounds_since_last_added_car = 0;

					}
					else {
						traffic_to_display = shift_traffic(prev_traffic, change_light, DO_NOT_ADD_CAR);
						rounds_since_last_added_car++ ;
					}

				}
				prev_traffic = traffic_to_display;
				xQueueSend(Traffic_Display_Queue, &traffic_to_display, 1000);
			}

			vTaskDelay(1000);
		}
	//}
}
static void Traffic_Light_Task() {
	uint8_t flow_rate;

	int rounds_since_light_change = 0;
	int light_delay = 0;

	while(1)
	{
		// if (xSemaphoreTask(Flow_Semaphore, (TickType_t) 10) == pdTRUE) {
			// Get flow rate from queue
			if (xQueueReceive(Traffic_Flow_Queue, &flow_rate, 500)) {
				if (flow_rate >= MAX_TRAFFIC && rounds_since_light_change > 1) {
					xEventGroupSetBits(Change_Light_Flag, 0);
					xQueueSend(Traffic_Light_Delay_Queue, &MAX_TRAFFIC_DELAY, 1000);
					rounds_since_light_change = 0;
				}
				else if(flow_rate >= MED_TRAFFIC && rounds_since_light_change > 3) {
					xEventGroupSetBits(Change_Light_Flag, 0);
					xQueueSend(Traffic_Light_Delay_Queue, &MED_TRAFFIC_DELAY, 1000);

					rounds_since_light_change = 0;
				}
				else if(rounds_since_light_change > 6) {
					xEventGroupSetBits(Change_Light_Flag, 0);
					xQueueSend(Traffic_Light_Delay_Queue, &MIN_TRAFFIC_DELAY, 1000);

					rounds_since_light_change = 0;
				}

				else {
					rounds_since_light_change++;
				}
			}
		// }
		vTaskDelay(1000);
	}

}
static void Traffic_Display_Task() {
	uint32_t traffic;
	int traffic_delay_on_red;

	while (1) {
		if (xQueueReceive(Traffic_Display_Queue, &traffic, 500)) {
			uint32_t test = traffic & 0x3FFFF;
			enumerate_traffic(traffic & 0x3FFFF);
			// if red light, delay for specified time
			if ((traffic >> 11 & 0x7) == 4) {
				if(xQueueReceive(Traffic_Light_Delay_Queue, &traffic_delay_on_red, 500)){
					sleep(traffic_delay_on_red);
				}
			}
		}
		vTaskDelay(1000);
	}

}

unsigned int shift_traffic(unsigned int prev_traffic, int change_light, int add_car) {

	// shift first 8 bits
	unsigned int first_eight_cars = prev_traffic & 0xFF;

	// either |1 or |0 based on the values from traffic flow task
	unsigned int shifted_first_eight_cars = first_eight_cars << 1 | add_car;

	// shift middle three lights
	unsigned int middle_three_cars = (prev_traffic >> 8 & 0x3);
	unsigned int shifted_middle_three_cars = (middle_three_cars << 1 | (first_eight_cars >> 8 & 0x1));

	// transition the light colour
	unsigned int light_colour = prev_traffic >> 11 & 0x7;

	unsigned int new_light_colour = light_colour;
	if(change_light == 1) {
		get_next_light_colour(light_colour);
	}

	new_light_colour = 1;

	//shift the last eight lights
	unsigned int last_eight_cars = (prev_traffic >> 14) & 0xFF;
	unsigned int shifted_last_eight_cars = ((last_eight_cars << 1) | (prev_traffic >> 10 & 0x1)) & 0xFF;
	unsigned int updated_traffic = (((shifted_last_eight_cars | 0x00) << 14) | (new_light_colour << 11)| (shifted_middle_three_cars << 8)| (shifted_first_eight_cars));

	return updated_traffic;

}

static void enumerate_traffic(unsigned int traffic_to_display) {
	// clear -> 0
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);

	unsigned int number_lights = 22;
	unsigned int current_light = 0;


	while(current_light <= number_lights) {
		unsigned int emulated_light = (traffic_to_display >> (number_lights - current_light)) & 0x1;

		printf("%d\n", emulated_light);

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
	if (light_colour == 1 || light_colour == 2) {
		return light_colour << 1;
	}
	return 1;
}

void sleep(unsigned int mseconds)
{
		int j = 0;
		while(++j < mseconds);

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

