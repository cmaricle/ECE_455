// Traffic 10101010 001 (green yellow) 010 01010101
// 01010100 010 (yellow light) 100 10101011
// 01010100 010 			   100 10101011
// green light 001, yellow 010, red 100
// value in decimal is 
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

static unsigned int get_next_light_colour(unsigned int);
static unsigned int output_traffic(unsigned int traffic_to_display);
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
	return rand() % 4;
}


int main(void) {
	srand(time(NULL));   

	setUpGPIO();

	
	Traffic_Flow_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));
	Traffic_Light_Delay_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));
	Traffic_Display_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));

	Flow_Semaphore = xSemaphoreCreateCount( 2, 0 );

	Change_Light_Flag = xEventGroupCreate();

	// Start tasks
	xTaskCreate( Traffic_Flow_Task, "Traffic_Flow", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Creator_Task, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Light_Task, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Display_Task, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Start tasks and timers
	vTaskStartScheduler();

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

		xSemaphoreGive(Flow_Semaphore);
		xSemaphoreGive(Flow_Semaphore);

	}

}

static void Traffic_Creator_Task() {
	uint8_t flow_rate;

	int rounds_since_last_added_car = 0;
	unsigned int prev_traffic = 0;
	unsigned int traffic_to_display;

	while (1) {

		if ( xSemaphoreTask(Flow_Semaphore, (TickType_t) 10) == pdTRUE ) {
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
				xQueueSend(Traffic_Display_Queue, &traffic_to_display, 1000);
			}

			vTaskDelay(1000);
		}
	}

}
static void Traffic_Light_Task() {
	uint8_t flow_rate;

	int rounds_since_light_change = 0;
	int light_delay = 0;
	while(1)
	{
		if (xSemaphoreTask(Flow_Semaphore, (TickType_t) 10) == pdTRUE) {
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
		}
		vTaskDelay(1000);
	}

}
static void Traffic_Display_Task() {
	unsigned int traffic;
	int traffic_delay_on_red;

	while (1) {
		if (xQueueReceive(Traffic_Display_Queue, &traffic, 500)) {
			output_traffic(traffic);
			// if red light, delay for specified time
			if ((traffic >> 11 & 0x7) == 4) {
				if(xQueueReceive(Traffic_Light_Delay_Queue, &traffic_delay_on_red, 500)){
					sleep(traffic_delay_on_red)
				}
			}
		}
		vTaskDelay(1000);
	}

}

static unsigned int shift_traffic(unsigned int prev_traffic, unsigned int change_light, unsigned int add_car) {
	// shift first 8 bits
	unsigned int first_eight_cars = prev_traffic & 0xFF;

	// either |1 or |0 based on the values from traffic flow task 
	unsigned int shifted_first_eight_cars = first_eight_cars << 1 | 1;

	// shift middle three lights
	unsigned int middle_three_cars = (prev_traffic >> 8 & 0x3);
	unsigned int shifted_middle_three_cars = middle_three_cars << 1 | (first_eight_cars >> 8 & 0x1);

	// transition the light colour
	unsigned int light_colour = prev_traffic >> 11 & 0x7; 

	unsigned int new_light_colour = light_colour;
	if(change_light == 1) {
		get_next_light_colour(light_colour);
	}

	//shift the last eight lights
	unsigned int last_eight_cars = (prev_traffic >> 14) & 0xFF;
	unsigned int shifted_last_eight_cars = (last_eight_cars << 1 & 0xFF) | (middle_three_cars >> 2 & 0x1);

	unsigned int updated_traffic = ((shifted_last_eight_cars << 14)| (new_light_colour << 11)| (shifted_middle_three_cars << 8)| (shifted_first_eight_cars)) & 0x3FFFFF;

	return output_traffic;
	
}

static void output_traffic(unsigned int traffic_to_display) {
	// clear -> 0
	GPIO_SetBits(GPIOC, GPIO_Pin_8);

	int number_lights = 22;
	int current_light = 0;
	while(current_light < number_lights) {
		unsigned int emulated_light = (updated_traffic >> current_light) & 0x1;

		// write emulated_light val to PC6
		GPIO_SetBits(GPIOC, GPIO_Pin_6);
		
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