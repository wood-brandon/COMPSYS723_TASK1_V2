// Standard includes
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
#include "alt_types.h"
#include "sys/alt_irq.h"
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "sys/alt_irq.h"

#include <altera_avalon_pio_regs.h>

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048
//freqarray size could be moved to global.h file???
#define FREQ_ARRAY_SIZE 50
#define PS2_KEY_DOWN 114
#define PS2_KEY_UP 117
#define PS2_KEY_ESC 118
#define ROC_STEP 0.5
#define ROC_DEFAULT 5

// Definition of Task Priorities
#define VGA_PRIORITY 			1
#define STABILITY_PRIORITY      6
#define LOAD_PRIORITY      		4
#define MANAGE_PRIORITY    		5
#define SWITCH_PRIORITY    		1
#define DEBUG_PRIORITY       	0

// Definition of Message Queue
#define   	MSG_QUEUE_SIZE  30
#define   	NUM_LOADS 5
QueueHandle_t LoadQueue;
QueueHandle_t frequencyQ;

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
xSemaphoreHandle sys_status_flag;
xSemaphoreHandle management_flag;
SemaphoreHandle_t shared_resource_sem;

// globals variables
int LoadStates[NUM_LOADS];
int LoadStatesUpdate[NUM_LOADS];
int freq_index = 0;
int freqThreshold = 50;
int ps2_buffer_index = 0;

float ROCThreshold = ROC_DEFAULT;
float freqValues[FREQ_ARRAY_SIZE] = {0};
float freqROCValues[FREQ_ARRAY_SIZE] = {0};

bool loadshedding = false;
bool managementState = false;
bool MaintenanceState = false;

char Threshold_Input_Buffer[3];

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);
void initDevices(void);
void LoadDisconnect(void);

// ----------Interrupt service routines
void NewFreqISR(){

    #define SAMPLING_FREQUENCY 16000.0
	double freq = SAMPLING_FREQUENCY/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR(frequencyQ, &freq, pdFALSE);
	//printf("Exiting ISR\n");
	return;
}
void PushButtonISR(){
	printf("button isr");
	MaintenanceState = !MaintenanceState;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);


}

void KeyboardISR(void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
  if ( status == 0 ) //success
  {
	printf("key  = %d",key);
    // print out the result
	switch(key){
	case PS2_KEY_DOWN:
		ROCThreshold -= ROC_STEP;
		break;
	case PS2_KEY_UP:
		ROCThreshold += ROC_STEP;
		break;
	case PS2_KEY_ESC:
		ROCThreshold = ROC_DEFAULT;
		break;
	}
  }
}
void SwitchPollingTask(void *pvParameters)
{
	int sw_result;
	int i;
	unsigned int change = 0;

	while (1)
	{
		bool j[NUM_LOADS];
		// Read load values from switch
		sw_result=IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		printf("number is %d\n", sw_result);
		// Filter and update LoadStatesUpdate
		for (i=0; i < NUM_LOADS;i++){
			j[i] = (bool) (sw_result & (1 << i));

		}

		//if load shedding
		xSemaphoreTake(sys_status_flag, portMAX_DELAY);
		if(loadshedding){
			for(i = 0; i<NUM_LOADS; i++){
				//only turn off loads
				LoadStates[i] = LoadStates[i] && j[i];
			}
		}else{
			for(i = 0; i<NUM_LOADS; i++){
				//otherwise do what you want
				LoadStates[i] = j[i];
			}
		}
		xSemaphoreGive(sys_status_flag);



		xQueueSend(LoadQueue,(void*)&change, 0);
		vTaskDelay(100);
	}



}

void VGATask(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(1000);

	}
}

// The following task fills up a message queue with incrementing data.  The data
// is not actually used by the application.  If the queue is full the task is
// suspended for 1 second.
void StabilityMonitorTask(void *pvParameters)
{

	double freq;
	//index used to check freqValues and freqROCValues for unstability
	int index;
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

			if(!managementState){

				if(freqValues[index] < freqThreshold || fabs(freqROCValues[index]) > ROCThreshold)
				{
					if(!loadshedding)
					{
						printf("unstable\n");

						//set loadshedding state to true
						loadshedding = true;
						int i;
						//copy the current state of the loads to update array and immediately shed a load
						for(i = 0; i < NUM_LOADS; i++)
						{
							xSemaphoreTake(sys_status_flag, portMAX_DELAY);
							LoadStatesUpdate[i] = LoadStates[i];

							xSemaphoreGive(sys_status_flag);
						}

						//shed first low priority load
						LoadDisconnect();


					}

				}
				else{
					printf("Stable\n");
				}


	    	}

	    }
	    //vTaskDelay(100);
	}
}

void LoadDisconnect(){
	int i;

	for(i=0; i<NUM_LOADS; i++ )
	{
		if(LoadStates[i] == 1)
		{
			xSemaphoreTake(sys_status_flag, portMAX_DELAY);
			LoadStates[i] = 0;
			xSemaphoreGive(sys_status_flag);
			return;
		}
	}
}

// This task manages the loads turning them on and off based on the data in loadQ. If the system becomes
// unstable LoadManagementTask ignores inputs from SwitchPollingTask and starts load shedding
// The system can be put into management mode through a push button ISR
void LoadControlTask(void *pvParameters)
{
	unsigned int *change;


	while (1)
	{
		xQueueReceive(LoadQueue, &change, portMAX_DELAY);

		if(loadshedding){



		}else{

		}

	}
}

void ManageLoadTask(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(1000);

	}
}

void DebugTask(void *pvParameters)
{
//	FILE* fp;
//	fp = fopen(CHARACTER_LCD_NAME, "w"); //open the character LCD as a file stream for write

	while (1)
	{
		vTaskDelay(3000);

//		fprintf(fp, "%c%sHello\n", 0x1b, "[2J"); //esc character (0x1b) followed by "[2J]" clears the screen
//		usleep(1000000);
//		fprintf(fp, "World!\n");
//		usleep(1000000);
//		fprintf(fp, "%c%sFrom NIOS II\n", 0x1b, "[2J");
//		usleep(1000000);
	}
}

void vTimerCallback(TimerHandle_t timer){
	printf("BASED DEPARTMENT??? \n");
}

int main(int argc, char* argv[], char* envp[])
{

	initOSDataStructs();
	initCreateTasks();
	initDevices();
	printf("started!!");
	vTaskStartScheduler();
	for (;;);
	return 0;
}
void initDevices(){
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	alt_up_ps2_clear_fifo (ps2_device) ;
	alt_irq_register(PS2_IRQ, ps2_device, KeyboardISR);
	  // register the PS/2 interrupt
	IOWR_8DIRECT(PS2_BASE,4,1);

}


// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	// QUEUES
	LoadQueue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(float));
	frequencyQ = xQueueCreate(MSG_QUEUE_SIZE, sizeof(double));

	// ISR
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x1); // mask interrupt for button 1
	alt_irq_register(PUSH_BUTTON_IRQ, NULL, PushButtonISR);
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, NewFreqISR);

	// TIMER
	TimerHandle_t unstable_timer;
	unstable_timer = xTimerCreate("Timer", (pdMS_TO_TICKS(5000)),pdTRUE,(void *)0,vTimerCallback);
	int debug = 0;
	debug = xTimerStart(unstable_timer,0);

	// SEMAPHORE
	shared_resource_sem = xSemaphoreCreateCounting( 9999, 1 );
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	//xTaskCreate(VGATask, "VGATask", TASK_STACKSIZE, NULL, VGA_PRIORITY, NULL);
	xTaskCreate(StabilityMonitorTask, "StabilityMonitorTask", TASK_STACKSIZE, NULL, STABILITY_PRIORITY, NULL);
	//xTaskCreate(LoadControlTask, "LoadControlTask", TASK_STACKSIZE, NULL, LOAD_PRIORITY, NULL);
	//xTaskCreate(ManageLoadTask, "ManageLoadTask", TASK_STACKSIZE, NULL, MANAGE_PRIORITY, NULL);
	xTaskCreate(SwitchPollingTask, "SwitchPollingTask", TASK_STACKSIZE, NULL, SWITCH_PRIORITY, NULL);
	//xTaskCreate(DebugTask, "DebugTask", TASK_STACKSIZE, NULL, DEBUG_PRIORITY, NULL);
	return 0;
}