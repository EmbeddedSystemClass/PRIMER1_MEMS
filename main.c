/*
    FreeRTOS V8.0.0 - Copyright (C) 2014 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!

*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks
 * (which just exist to test the kernel port and provide an example of how to use
 * each FreeRTOS API function).
 *
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.  The use
 * of a gatekeeper in this manner permits both tasks and interrupts to write to
 * the LCD without worrying about mutual exclusion.  This is demonstrated by the
 * check hook (see below) which sends messages to the display even though it
 * executes from an interrupt context.
 *
 * "Check" hook -  This only executes fully every five seconds from the tick
 * hook.  Its main function is to check that all the standard demo tasks are
 * still operational.  Should any unexpected behaviour be discovered within a
 * demo task then the tick hook will write an error to the LCD (via the LCD task).
 * If all the demo tasks are executing with their expected behaviour then the
 * check task writes PASS to the LCD (again via the LCD task), as described above.
 *
 * LED tasks - These just demonstrate how multiple instances of a single task
 * definition can be created.  Each LED task simply toggles an LED.  The task
 * parameter is used to pass the number of the LED to be toggled into the task.
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the OLED display by the 'OLED' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Library includes for STM32F429I boards. */
#include "stm32f4xx.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_tim.h"
#include "stm32_eval_legacy.h"
#include "stm32f4xx_conf.h"
#include "partest.h"

/*My custom libraries includes */

#include "custom_libs/inc/usart_RTOS.h"
#include "custom_libs/inc/i2c_RTOS.h"
/*
 * Configure the hardware.
 */
static void prvSetupHardware( void );

extern void vSetupHighFrequencyTimer( void );

static void prvSetupMyLCD( void );

/*

static char USART1GetChar();

static void USART1PutChar(char put_char);
static void USART1PutString(char *put_string, uint8_t string_length);
*/
static void ADC_config();

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY ( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY ( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY   ( tskIDLE_PRIORITY + 1)

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY1000         ( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* The rate at LCD is refreshed. */
#define mainLCD_DELAY               ( ( portTickType ) 300 / portTICK_RATE_MS )
/* The rate at which the message is sent to the USART*/
#define mainUSART_DELAY             ( ( portTickType ) 1000 / portTICK_RATE_MS )





typedef struct
{
    int32_t temp;
    uint16_t tickcount;

} adc_temp;




/*Task Handlers*/
    TaskHandle_t HandleTask1;
    TaskHandle_t HandleTask2;
    TaskHandle_t HandleTask3;
    TaskHandle_t HandleTask4;

/*Mutexs*/
    /* USART1 mutex */
    //extern SemaphoreHandle_t xMutex_USART;

/*Bynary Semaphores*/
    /* Button Pressed Binary Semaphore*/
    SemaphoreHandle_t xButtonPressed_SEMBIN;


/*Queues*/

    /*USART Rx*/
    //extern QueueHandle_t xQueue_USART_Rx;

    /*USART Tx*/
    //extern QueueHandle_t xQueue_USART_Tx;

    /* Button clicks*/
    QueueHandle_t xQueue_Button;

    /* Temperature Queue */
    QueueHandle_t xQueue_Temp;




static void prvTaskClicks () {

    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    uint8_t duplo_clique=0, tick_reset=0;
    TickType_t last_tick=0, last_last_tick=0, actual_tick;

    for(;;)
    {
        xQueueReceive( xQueue_Button, &last_last_tick, ( TickType_t ) portMAX_DELAY );
        last_tick = last_last_tick;
        while(xQueueReceive( xQueue_Button, &last_tick, (300-(last_tick-last_last_tick)) / portTICK_RATE_MS ))
        {
            if(last_tick - last_last_tick<30){
                //replicas
                continue;
            }
            else{
                //duplo clique
                GPIO_WriteBit(GPIOG, GPIO_Pin_14, (1-GPIO_ReadOutputDataBit(GPIOG, GPIO_Pin_14)));
                duplo_clique=1;
                break;
            }
        }

        if(duplo_clique==0)
            //single clique
            GPIO_WriteBit(GPIOG, GPIO_Pin_13, (1-GPIO_ReadOutputDataBit(GPIOG, GPIO_Pin_13)));
        else
            duplo_clique =0;

    }
}


static void prvTaski2cprint()  {

    TickType_t xLastExecutionTime = xTaskGetTickCount();

    USART1Init(115200, 50);
    init_I2C1();

    TickType_t timestamp;

    char buffer[10];

    for(;;)
    {
        vTaskDelayUntil(&xLastExecutionTime, 20 / portTICK_RATE_MS);

        timestamp = xTaskGetTickCount();
        sprintf(buffer,"%d", timestamp);
        USART1PutString(buffer, strlen(buffer));

        I2C_printf(0xD0," amb=%d  %d %d %d %d %d %d %d %d \r\n", 0x01, 0x02, 0x03, 0x04,0x05,0x06,0x07, 0x08, 0x09);

    }
}

static void prvTaskLed1Hz(){

    TickType_t xLastExecutionTime = xTaskGetTickCount();


    init_I2C1();

    for(;;){
        vTaskDelayUntil( &xLastExecutionTime, 500/ portTICK_RATE_MS);

        GPIO_WriteBit(GPIOG, GPIO_Pin_13, (1-GPIO_ReadOutputDataBit(GPIOG, GPIO_Pin_13)));



    }
}

static void prvTaskI2CRead(){

    TickType_t xLastExecutionTime = xTaskGetTickCount();


    init_I2C1();

    for(;;){
        vTaskDelayUntil( &xLastExecutionTime, 500/ portTICK_RATE_MS);



    }

}

static void prvTaskLCD() {


    TickType_t xLastExecutionTime = xTaskGetTickCount();

    char buffer[30];

    for(;;){
        vTaskDelayUntil( &xLastExecutionTime, 500/ portTICK_RATE_MS);

        sprintf(buffer, "Pedro");
        LCD_DisplayStringLine( Line1 , buffer);


    }


}



int main( void )
{

    prvSetupHardware();


    /* Task blink*/
    xTaskCreate(prvTaskLed1Hz, "led1Hz",configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, &HandleTask2);


    /* LCD */
    xTaskCreate(prvTaskLCD, "lcd",configMINIMAL_STACK_SIZE*2, NULL, mainLCD_TASK_PRIORITY, &HandleTask1);


    /* Task to output all I2C data at 50 Hz*/
    xTaskCreate(prvTaski2cprint, "i2c_print",configMINIMAL_STACK_SIZE*2, NULL, mainLCD_TASK_PRIORITY, &HandleTask4);


	/* Start the scheduler. */
	vTaskStartScheduler();

	for( ;; );
}

void ButtonInit(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    EXTI_InitTypeDef    EXTI_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect EXTI Line0 to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure EXTI Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line0 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    xQueue_Button = xQueueCreate(10, sizeof(TickType_t));



}


void EXTI0_IRQHandler(void)
{
    static BaseType_t pxHigherPriorityTaskWoken;

    TickType_t tick_count;

    tick_count = xTaskGetTickCountFromISR();

    //Check if EXTI_Line0 is asserted
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        //xSemaphoreGiveFromISR( xButtonPressed_SEMBIN, &pxHigherPriorityTaskWoken );
        //we need to clear line pending bit manually
        //GPIO_WriteBit(GPIOG, GPIO_Pin_14, (1-GPIO_ReadOutputDataBit(GPIOG, GPIO_Pin_14)));

        xQueueSendToBackFromISR( xQueue_Button, &tick_count, &pxHigherPriorityTaskWoken);

        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    if( pxHigherPriorityTaskWoken == pdTRUE )
        taskYIELD(); /* forces a context switch before exit the ISR */

}



static void prvSetupHardware( void )
{
    /* Setup STM32 system (clock, PLL and Flash configuration) */
    SystemInit(); //-> sets the clock to 72 Mhz (already changed from 180Mhz )

    ButtonInit();
    /* Ensure all priority bits are assigned as preemption priority bits. */
    //NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    prvSetupMyLCD();
    /* Initialise the IO used for the LED outputs. */
    vParTestInitialise();

    //ADC_config();
}


void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/


static void prvSetupMyLCD( void )
{

    LCD_Init();
    LCD_LayerInit();
    LTDC_LayerCmd(LTDC_Layer1, ENABLE);
    LTDC_LayerCmd(LTDC_Layer2, DISABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_SetFont(&Font8x12);
    LCD_Clear(White);
    LCD_SetTextColor(Black);

}
