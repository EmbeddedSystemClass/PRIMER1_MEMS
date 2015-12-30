


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

#include "usart_RTOS.h"

/*Queues*/
    /*USART Rx*/
    QueueHandle_t xQueue_USART_Rx;

    /*USART Tx*/
    QueueHandle_t xQueue_USART_Tx;

/*Mutexes*/
    /*USART (Write) Mutex*/
    SemaphoreHandle_t xMutex_USART;



void USART1Init(uint32_t baudrate, uint16_t buffer_depth )
{
    switch(baudrate)
    {
        case 2400:
            prvSetupUSART1(baudrate);
            break;
        case 4800:
            prvSetupUSART1(baudrate);
            break;
        case 9600:
            prvSetupUSART1(baudrate);
            break;
        case 14400:
            prvSetupUSART1(baudrate);
            break;
        case 19200:
            prvSetupUSART1(baudrate);
            break;
        case 28800:
            prvSetupUSART1(baudrate);
            break;
        case 38400:
            prvSetupUSART1(baudrate);
            break;
        case 57600:
            prvSetupUSART1(baudrate);
            break;
        case 76800:
            prvSetupUSART1(baudrate);
            break;
        case 115200:
            prvSetupUSART1(baudrate);
            break;
        case 230400:
            prvSetupUSART1(baudrate);
            break;
        default:
            prvSetupUSART1(115200);
    }

    xQueue_USART_Tx = xQueueCreate(buffer_depth, sizeof(char));
    xQueue_USART_Rx = xQueueCreate(buffer_depth, sizeof(char));

    xMutex_USART = xSemaphoreCreateMutex();

    if(xQueue_USART_Rx==0 && xQueue_USART_Tx==0)
        prvSendMessageUSART1("USART1 queue error.\r\n");

}

char USART1GetChar()
{
    char aux_char;

    if(xQueueReceive( xQueue_USART_Rx, &aux_char, ( TickType_t ) portMAX_DELAY ))
    {
        return aux_char;
    }
}

void USART1PutChar(char put_char)
{
    xQueueSendToBack( xQueue_USART_Tx, ( void * ) &put_char,( TickType_t ) portMAX_DELAY  );

    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void USART1PutString(char *put_string, uint8_t string_length)
{

    uint8_t tamanho =0;

    xSemaphoreTake( xMutex_USART, ( TickType_t ) portMAX_DELAY  );

    while(tamanho<string_length){
        xQueueSendToBack( xQueue_USART_Tx, ( void * ) &put_string[tamanho],( TickType_t ) portMAX_DELAY  );
        tamanho++;
    }

    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    xSemaphoreGive( xMutex_USART);
}

void USART1Close()
{


}



void prvSetupUSART1( uint32_t baudrate )
{

    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


    GPIO_InitStruct.GPIO_Mode   =    GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType  =    GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   =    GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed  =    GPIO_Speed_100MHz;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    //                         TX           RX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;

    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = baudrate;

    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx| USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;

    /* Configure the USART1 */
    USART_Init(USART1, &USART_InitStruct);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Config USART1 Interrupt*/
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable de USART1 */
    USART_Cmd(USART1, ENABLE);

}


void prvSendMessageUSART1(char *message){

uint16_t cont_aux=0;

    while(cont_aux != strlen(message))
    {
        USART_SendData(USART1, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        cont_aux++;
    }
}

<<<<<<< HEAD


=======
>>>>>>> 3c634e9d7419a9588772ee9a81cabc54f53872fc
/* UART1-Interrupt */

void USART1_IRQHandler(void) {

    static BaseType_t pxHigherPriorityTaskWoken;

    static char usart_aux;
    static char usart_aux_tx;

    if( USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        usart_aux = USART_ReceiveData(USART1); //read the received byte clears the interruption flag
        xQueueSendToBackFromISR( xQueue_USART_Rx, &usart_aux, &pxHigherPriorityTaskWoken);
    }

    if(USART_GetITStatus(USART1, USART_IT_TXE))
    {
<<<<<<< HEAD
        USART_ClearITPendingBit(USART1, USART_IT_TXE);

        if(xQueueReceiveFromISR( xQueue_USART_Tx, &usart_aux_tx ,&pxHigherPriorityTaskWoken)==pdTRUE)
        {
            USART_SendData(USART1, usart_aux_tx);
            if(xQueueIsQueueEmptyFromISR( xQueue_USART_Tx )==pdTRUE)
                USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
=======
        if(xQueueReceiveFromISR( xQueue_USART_Tx, &usart_aux_tx ,&pxHigherPriorityTaskWoken))
        {
            USART_SendData(USART1, usart_aux_tx);
>>>>>>> 3c634e9d7419a9588772ee9a81cabc54f53872fc
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }

<<<<<<< HEAD
=======
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
>>>>>>> 3c634e9d7419a9588772ee9a81cabc54f53872fc
    }

    if( pxHigherPriorityTaskWoken == pdTRUE )
        taskYIELD(); /* forces a context switch before exit the ISR */

}