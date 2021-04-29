// Standard includes test
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "io.h"
#include "alt_types.h"
#include "sys/alt_timestamp.h"
#include "sys/alt_irq.h"

#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include <altera_avalon_timer.h>
#include <altera_avalon_pio_regs.h>

#include "global.h"

#define TASK_STACKSIZE  2048
#define FREQ_ARRAY_SIZE 50
#define MSG_QUEUE_SIZE  30
#define ALT_CLK_TO_MS 100

#define PS2_KEY_DOWN 114
#define PS2_KEY_UP 117
#define PS2_KEY_LEFT 107
#define PS2_KEY_RIGHT 116
#define PS2_KEY_ESC 118


//VGA variable definitions
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

// Definition of Task Priorities
#define VGA_PRIORITY 			2
#define STABILITY_PRIORITY      4
#define LOAD_PRIORITY      		3
#define SWITCH_PRIORITY    		1

#define SWITCH_DELAY 100
#define VGA_DELAY 16 // most vga monitors do not support above 60hz so why refresh faster
#define SHORT_DELAY 10

#define LOAD_STATE_OFF 0
#define LOAD_STATE_ON 1
#define LOAD_STATE_MANAGED 2

//-------------------------------------FreeRTOS Structures
// Queues
QueueHandle_t LoadQueue;
QueueHandle_t frequencyQ;

// Tasks
TaskHandle_t xHandle;

// Timers
TimerHandle_t unstable_timer;
TimerHandle_t reaction_timer;

// Semaphore
xSemaphoreHandle sys_status_flag;
xSemaphoreHandle management_flag;
xSemaphoreHandle LoadQ_sem;
SemaphoreHandle_t shared_resource_sem;

//-------------------------------------GLOBAL VARIABLES
int LoadStates[NUM_LOADS];
int LoadStatesUpdate[NUM_LOADS];
int freq_index = 0;
float freqThreshold = FREQ_THRESHOLD;
int ps2_buffer_index = 0;
int GREEN_LED = 0;
int RED_LED = 0;

long reaction_measures[5] = { 0 };
long max_reaction = 0;
long min_reaction = 0;
float average_reaction = 0;
char measureBuffer[50];
char Threshold_Input_Buffer[3];
double old_time = 0;

float ROCThreshold = ROC_DEFAULT;
float freqValues[FREQ_ARRAY_SIZE] = {0};
float freqROCValues[FREQ_ARRAY_SIZE] = {0};

bool System_Stable = true;
bool loadshedding = false;
bool first_shed = false;
bool disable_shed = false;
bool managementState = false;

//VGA global variables
alt_up_char_buffer_dev *char_buf;
alt_up_pixel_buffer_dma_dev *pixel_buf;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);
void initDevices(void);
void LoadDisconnect(void);
void LoadReconnect(const char*);
void SaveMeasurement(long);
void drawBackground(void);
void drawGraphs(float*, float*, int);
void drawReactionTime(float, long, long, long*);
void drawThresholds(float, float);
void drawUptime(int);

//-------------------------------------Interrupt service routines
// Handles new frequency data from memory
void NewFreqISR(){

    #define SAMPLING_FREQUENCY 16000.0
	if (!first_shed)
		alt_timestamp_start();
	double freq = SAMPLING_FREQUENCY/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	xQueueSendToBackFromISR(frequencyQ, &freq, pdFALSE);
	return;
}
// ISR for maintenance mode button
void PushButtonISR(){
	managementState = !managementState;
	if (managementState){

		disable_shed = true;
		loadshedding = false;
		xTimerStopFromISR(unstable_timer,0);
	}
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//xQueueSendFromISR(LoadQueue,0, 0);
	xSemaphoreGiveFromISR(LoadQ_sem,8);
}

// TODO ROC threshold semaphore
// ISR for keyboard: up and down arrows to control ROC, esc to reset to default
void KeyboardISR(void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;
  static bool prevent_double_trigger = false;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
  if ( status == 0 ) //success
  {
	  if (prevent_double_trigger) {
		  switch (key) {
		  case PS2_KEY_DOWN:
			  if (ROCThreshold > 0)
				  ROCThreshold -= ROC_STEP;
			  break;
		  case PS2_KEY_UP:
			  ROCThreshold += ROC_STEP;
			  break;
		  case PS2_KEY_RIGHT:
			  freqThreshold += FREQ_STEP;
			  break;
		  case PS2_KEY_LEFT:
			  if (freqThreshold > 0)
				  freqThreshold -= FREQ_STEP;
			  break;
		  case PS2_KEY_ESC:
			  freqThreshold = FREQ_THRESHOLD;
			  ROCThreshold = ROC_DEFAULT;
			  break;
		  }
	  }
	}

  prevent_double_trigger = !prevent_double_trigger;
}

// This task continually polls the switches to update 
void SwitchPollingTask(void *pvParameters)
{
	int sw_result;
	int i;
	unsigned int change = 0;
	while (1)
	{
		bool j[NUM_LOADS];
		sw_result=IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		sw_result = sw_result & 0x1F; // mask only bottom switches
		for (i=0; i < NUM_LOADS;i++){
			j[i] = (bool) (sw_result & (1 << i));
		}

		//if load shedding
		xSemaphoreTake(sys_status_flag, portMAX_DELAY);	//TODO FIX SEMAPHORE
		if(loadshedding && !managementState){
			for(i = 0; i<NUM_LOADS; i++){
				//only turn off loads
				if (j[i] == LOAD_STATE_OFF){
					LoadStates[i] = j[i];
				}
			}
		}else{
			for(i = 0; i<NUM_LOADS; i++){
				if (LoadStates[i] == LOAD_STATE_MANAGED){
				}
				LoadStates[i] = (int)(j[i]);
			}
		}

		xSemaphoreGive(sys_status_flag);
		xSemaphoreGive(LoadQ_sem);

		//xQueueSend(LoadQueue,(void*)&change, 0);
		vTaskDelay(SWITCH_DELAY);
	}



}

void VGATask(void *pvParameters)
{
	float vga_Freq[FREQ_ARRAY_SIZE];
	float vga_ROCFreq[FREQ_ARRAY_SIZE];
	long vga_measures[5];
	int startpos;

	while (1)
	{

		taskENTER_CRITICAL();
		startpos = freq_index;
		int i;
		for(i = 0; i < FREQ_ARRAY_SIZE; i++){
			vga_Freq[i] = freqValues[i];
			vga_ROCFreq[i] = freqROCValues[i];
		}
		for(i = 0; i<5; i++){
			vga_measures[i] = reaction_measures[i];
		}
		taskEXIT_CRITICAL();


		drawBackground();
		drawGraphs(vga_Freq, vga_ROCFreq, startpos);
		drawReactionTime(average_reaction, min_reaction, max_reaction, vga_measures);
		drawThresholds(freqThreshold, ROCThreshold);
		drawUptime(xTaskGetTickCount()/1000);

		vTaskDelay(VGA_DELAY);
	}
}

void drawGraphs(float* freq, float* ROCfreq, int i){

	//float freq[50];
	//float ROCfreq[50];
	int j;

	typedef struct{
		unsigned int x1;
		unsigned int x2;
		unsigned int y1;
		unsigned int y2;
	}Line;

	Line line_freq;
	Line line_roc;


	//clear old graph to draw new graph
	alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
	alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

	for(j=0;j<49;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
		if (((int)(freq[(i+j)%50]) > MIN_FREQ) && ((int)(freq[(i+j+1)%50]) > MIN_FREQ)){
			//Calculate coordinates of the two data points to draw a line in between
			//Frequency plot
			line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
			line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%50] - MIN_FREQ));

			line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
			line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%50] - MIN_FREQ));

			//Frequency RoC plot
			line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
			line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * ROCfreq[(i+j)%50]);

			line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
			line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * ROCfreq[(i+j+1)%50]);

			//Draw
			alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
			alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
		}
	}
	vTaskDelay(SHORT_DELAY);

}

void drawBackground(){

	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

}

void drawReactionTime(float average, long min, long max, long *measures){

	int i;
	float minimum = (float)min/1000;
	float maximum = (float)max/1000;
	float avg = (float)average/1000;
	float temp;

	// print past 5 timing measurements
	alt_up_char_buffer_string(char_buf, "TIMING MEASUREMENTS (MS)", 4, 40);
	for(i = 0; i < 5; i++){
		temp = (float)measures[i];
		sprintf(measureBuffer, "%d:  %f", i+1, temp/1000);
		alt_up_char_buffer_string(char_buf, measureBuffer, 4, 42+i*2);
	}

	sprintf(measureBuffer, "Average: %f", avg);
	alt_up_char_buffer_string(char_buf, measureBuffer, 4, 42+10);
	sprintf(measureBuffer, "Minimum: %f", minimum);
	alt_up_char_buffer_string(char_buf, measureBuffer, 4, 42+12);
	sprintf(measureBuffer, "Maximum: %f", maximum);
	alt_up_char_buffer_string(char_buf, measureBuffer, 4, 42+14);

}

void drawThresholds(float freq, float roc){
	sprintf(measureBuffer, "Frequency Threshold:  %.2f Hz", freq);
	alt_up_char_buffer_string(char_buf, measureBuffer, 44, 40);
	sprintf(measureBuffer, "ROC Threshold:  %.2f Hz/s", roc);
	alt_up_char_buffer_string(char_buf, measureBuffer, 44, 42);

}

void drawUptime(int seconds){

	int days;
	int hours;
	int minutes;
	int daysConversion = 60*60*24;
	int hoursConversion = 60*60;
	int minutesConversion = 60;

	days = seconds/daysConversion;
	seconds = seconds - (days*daysConversion);
	hours = seconds/hoursConversion;
	seconds = seconds - (hours*hoursConversion);
	minutes = seconds/minutesConversion;
	seconds = seconds-(minutes*minutesConversion);

	sprintf(measureBuffer, "System Uptime: %d:%d:%d:%d", days, hours, minutes, seconds);
	alt_up_char_buffer_string(char_buf, measureBuffer, 44, 44);

}

void StabilityMonitorTask(void *pvParameters)
{
	double freq;
	//index used to check freqValues and freqROCValues for unstability
	int index;
	unsigned int change = 0;
	while (1)
	{
	    if(xQueueReceive(frequencyQ, &freq, portMAX_DELAY)==pdTRUE)
	    {
			//add frequencies to array
			freqValues[freq_index] = freq;
			//calculation for frequency rate of change
			if(freq_index == 0){
				freqROCValues[0] = (freqValues[0]-freqValues[49])*2.0*freqValues[0]*freqValues[49]/(freqValues[0]+freqValues[49]);
			}else{
				freqROCValues[freq_index] = (freqValues[freq_index]-freqValues[freq_index-1])*2.0*freqValues[freq_index]*freqValues[freq_index-1]/(freqValues[freq_index]+freqValues[freq_index-1]);
			}

			freq_index = (freq_index + 1)%50;

			if (freq_index == 0){
				index = FREQ_ARRAY_SIZE-1;
			}else{
				index = freq_index -1;
			}
			//printf("Frequency = %f, ROC = %f\n", freqValues[index], freqROCValues[index]);
			if(!managementState){

				if(freqValues[index] < freqThreshold || fabs(freqROCValues[index]) >= ROCThreshold)
				{
					//-------------------------------------------------------------------------------SYSTEM UNSTABLE!
					if (System_Stable){
						System_Stable = false; // TODO SEMAPHORE :)
						xTimerStart(unstable_timer,0);
					}

					if(!loadshedding && RED_LED != 0)
					{
						// time initial load shedding


						//old_time = (double)(alt_timestamp());
						loadshedding = true;
						first_shed = true;

						//shed first low priority load
						LoadDisconnect();
						//xQueueSend(LoadQueue,(void*)&change, 0);
						xSemaphoreGive(LoadQ_sem);

						//printf("first_shed %d, loadshedding %d, \n",uxQueueMessagesWaiting(LoadQueue));
						//xTimerStart(unstable_timer, 0);
					}
				}else{
					//-------------------------------------------------------------------------------SYSTEM STABLE!!
					if (!System_Stable) {
						System_Stable = true; // TODO SEMAPHORE
						xTimerStart(unstable_timer,0);
					}
				}
	    	}
	    }
	    vTaskDelay(SHORT_DELAY);
	}
}

void LoadDisconnect(){
	int i;
	xSemaphoreTake(sys_status_flag, portMAX_DELAY);
	for(i=0; i<NUM_LOADS; i++ )
	{
		if(LoadStates[i] == LOAD_STATE_ON)
		{
			LoadStates[i] = LOAD_STATE_MANAGED;
			xSemaphoreGive(sys_status_flag);
			return;
		}
	}
	xSemaphoreGive(sys_status_flag);
	return;
}

void LoadReconnect(const char *ReconnectType){
	int i;
	xSemaphoreTake(sys_status_flag, portMAX_DELAY);
	for(i=(NUM_LOADS-1); i>=0; i--)
	{
		if(LoadStates[i] == LOAD_STATE_MANAGED)
		{
			LoadStates[i] = LOAD_STATE_ON;
			// if arg was single reconnect, stop after one has been enabled
			if (!strcmp(ReconnectType,"Single")){
				xSemaphoreGive(sys_status_flag);
				return;
			}
		}

	}
	xSemaphoreGive(sys_status_flag);
	// if arg was single reconnect, all loads are connected
	if (!strcmp(ReconnectType, "Single")) {
		loadshedding = false; // TODO semaphore;
		xTimerStop(unstable_timer, 0);
	}
}

// This task manages the loads turning them on and off based on the data in loadQ. If the system becomes
// unstable LoadManagementTask ignores inputs from SwitchPollingTask and starts load shedding
// The system can be put into management mode through a push button ISR
void LoadControlTask(void *pvParameters)
{
	unsigned int change;
	int i = 0;
	long time = 0;
	while (1)
	{
		if (first_shed){
			printf("LOAD CTRL TASK: BEFORE QUEUE %d\n",alt_timestamp());
		}

		//if (xQueueReceive(LoadQueue, &change, portMAX_DELAY)==pdTRUE){
		if (xSemaphoreTake(LoadQ_sem, 200)){
			GREEN_LED = 0;
			RED_LED = 0;
			if (first_shed){
				printf("LOAD CTRL TASK: AFTER QUEUE\n");
				//printf("sys_stable = %d, loadshedding = %d, first_shed = %d, disable_shed = %d, managementstate = %d\n",System_Stable,loadshedding,first_shed,disable_shed,managementState);
			}
			// ensure management mode turns off all loads with flag
			if (disable_shed){
				LoadReconnect("All");
				disable_shed = false;
			}

			for (i = 0;i<NUM_LOADS;i++){
				switch(LoadStates[i]){
				case(LOAD_STATE_ON):
					RED_LED += (1 << i);
					break;
				case(LOAD_STATE_MANAGED):
					GREEN_LED += (1 << i);
					break;
				default:
					break;
				}

			}
		}

		if (GREEN_LED == 0){
			loadshedding = false;
			first_shed = false;
			xTimerStop(unstable_timer, 0);
		}

		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, RED_LED);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, GREEN_LED);

		// evaluate timing constraints
		if (first_shed){
			first_shed = false;
			time = (long)(alt_timestamp());
			//printf("time between unstable and load shed = %fms\n",time);
			//printf("old %f new %f diff %f\n",old_time,time,time-old_time);
			SaveMeasurement(time);

			// Start 500ms timer
			//xTimerStart(unstable_timer, 0);
		}
		vTaskDelay(SHORT_DELAY);
	}
}

// helper function to save timer measurements for display on vga
void SaveMeasurement(long newMeasure) {
	int i;
	static bool init = false;
	average_reaction = 0;

	for (i = 4; i > 0; i--) {
		reaction_measures[i] = reaction_measures[i-1];
		average_reaction += reaction_measures[i];
	}
	reaction_measures[0] = newMeasure;
	average_reaction += reaction_measures[0];
	average_reaction = average_reaction / NUM_LOADS;

	if (init) {
		if (newMeasure > max_reaction)
			max_reaction = newMeasure;

		if (newMeasure < min_reaction)
			min_reaction = newMeasure;
	}
	else {
		max_reaction = newMeasure;
		min_reaction = newMeasure;
		init = true;
	}

	//printf("------------------SAVEMEASUREMENT VALUES--------------\n");
	//for (i = 4; i >= 0; i--) {
	//	printf("measure[%d] = %ldms\n", i, reaction_measures[i]);
	//}
	//printf("max reaction = %ld, min reaction = %ld average = %f\n", max_reaction, min_reaction, average_reaction);
	//printf("------------------END SAVEMEASUREMENT VALUES--------------\n");

}

// 500ms timer to evaluate stability, load reconnecting/disconnecting is a function of system state
void vTimerCallback(TimerHandle_t timer){
	unsigned int *change = 0;

	// toggle another load
	if (!managementState){
		if (!System_Stable){
			LoadDisconnect();
		}
		else{
			LoadReconnect("Single");
		}
		xQueueSend(LoadQueue,(void*)&change, 0);
	}
	// timer will auto restart until stopped by StabilityMonitorTask when necessary
}

int main(int argc, char* argv[], char* envp[])
{
	initOSDataStructs();
	initCreateTasks();
	initDevices();
	vTaskStartScheduler();
	for (;;);
	return 0;
}

void initDevices(){
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	alt_up_ps2_clear_fifo (ps2_device) ;
	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, KeyboardISR);
	// register the PS/2 interrupt
	IOWR_8DIRECT(PS2_BASE,4,1);

	//initialize the pixel buffer
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("Cannot find pixel buffer device\n");
	}
	//clear the display
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	//initialize character buffer
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	//clear the character buffer
	alt_up_char_buffer_clear(char_buf);

}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	// QUEUES
	LoadQueue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(float));
	frequencyQ = xQueueCreate(MSG_QUEUE_SIZE, sizeof(double));

	// ISRS
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x1); // mask interrupt for button 1
	alt_irq_register(PUSH_BUTTON_IRQ, NULL, PushButtonISR);
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, NewFreqISR);

	// TIMER
	unstable_timer = xTimerCreate("Timer", (pdMS_TO_TICKS(STABILITY_DELAY_MS)),pdTRUE,(void *)0,vTimerCallback);
	//alt_timestamp_start();
	// SEMAPHORE
	sys_status_flag = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(LoadQ_sem);

	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(VGATask, "VGATask", TASK_STACKSIZE, NULL, VGA_PRIORITY, NULL);
	xTaskCreate(StabilityMonitorTask, "StabilityMonitorTask", TASK_STACKSIZE, NULL, STABILITY_PRIORITY, NULL);
	xTaskCreate(LoadControlTask, "LoadControlTask", TASK_STACKSIZE, NULL, LOAD_PRIORITY, NULL);
	xTaskCreate(SwitchPollingTask, "SwitchPollingTask", TASK_STACKSIZE, NULL, SWITCH_PRIORITY, NULL);
	return 0;
}
