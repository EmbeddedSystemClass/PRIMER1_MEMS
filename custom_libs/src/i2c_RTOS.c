#define SLAVE_ADDRESS 0xD0 // slave address
//#define DEBUG
/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "queue.h"

//  Library includes for STM32F429I boards.
#include "stm32f4xx.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_tim.h"
#include "stm32_eval_legacy.h"
#include "stm32f4xx_conf.h"
#include "partest.h"


#include "i2c_RTOS.h"

/* Queues */
    /* Data Instructions */
    QueueHandle_t xQueue_I2C_INSTR;

    /* Rx Receiver*/
    QueueHandle_t xQueue_I2C_RX;


typedef struct
{
    /*1 - Read, 2 - Write */
    uint8_t op_code;
    /* Adress of the slave */
    uint8_t address;
    /*Register to read from */
    uint8_t reg;
    /*Data to write on slave*/
    uint8_t data; // just when op_code == 2

} I2CData;



void init_I2C1(void){

    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    I2C_InitTypeDef I2C_InitStruct;



    // enable APB1 peripheral clock for I2C1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    // enable clock for SCL and SDA pins
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Connect I2C1 pins to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

    // configure I2C1
    I2C_InitStruct.I2C_ClockSpeed = 100000;         // 100kHz
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;         // I2C mode
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;          // own address, not relevant in master mode
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;       // disable acknowledge when reading (can be changed later on)
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
    I2C_Init(I2C1, &I2C_InitStruct);                // init I2C1


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Configure and enable I2Cx event interrupt -------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);


    /* Configure and enable I2C1 error interrupt -------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);


    // enable I2C1
    I2C_Cmd(I2C1, ENABLE);

    xQueue_I2C_INSTR = xQueueCreate(20, sizeof( I2CData));
    xQueue_I2C_RX = xQueueCreate(20, sizeof(char));

}



char I2C_Read(uint8_t address, uint8_t reg){

    char rx_aux;

    I2CData i2c_data;

    /*OP read*/
    i2c_data.op_code = 1;
    i2c_data.address = address;
    i2c_data.reg = reg;

    xQueueSendToBack(xQueue_I2C_INSTR, &i2c_data, portMAX_DELAY);

    I2C_GenerateSTART(I2C1,ENABLE);

    xQueueReceive( xQueue_I2C_RX, &rx_aux, ( TickType_t ) portMAX_DELAY );

    return rx_aux;

}

void I2C_Write(uint8_t address, uint8_t reg, uint8_t data){

    I2CData i2c_data;

    /* OP Write */
    i2c_data.op_code = 2;
    i2c_data.address = address;
    i2c_data.reg = reg;
    i2c_data.data = data;

    xQueueSendToBack(xQueue_I2C_INSTR, &i2c_data, portMAX_DELAY);

    I2C_GenerateSTART(I2C1, ENABLE);

}



char I2C_printf( uint8_t address, char *string, ...){
/*
    Needs USARTInit() ...
*/
    char next_char,rx;
    va_list list;
    uint32_t num_chars;

    TickType_t timestamp;

    va_start(list, string);

    while(*string!='\0'){
        if(*string == '%')
        {
            next_char = *(++string);
            switch(next_char)
            {
                case 'i':
                case 'd':
                    rx = I2C_Read(address,(uint8_t) va_arg(list ,int));

                    /* My version of sprintf for integers ...*/
                    num_chars = 1;
                    while(num_chars<rx) // see how big is var rx without change it
                        num_chars *=10;

                    while(num_chars/=10 ){
                        USART1PutChar(    (uint8_t)rx/num_chars          +48); //output the most signficant digit
                        rx %=num_chars; // update the new value of rx
                    }
                break;
            }

        }
        else{

            USART1PutChar(*string);
        }
        string++;

    }

}

/* I2C Interrup Handlers*/

void I2C1_EV_IRQHandler(void)
{
    static BaseType_t pxHigherPriorityTaskWoken;

    char rx_aux;

    static char state = 0;

    I2CData i2c_data;


    I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);

    xQueuePeekFromISR( xQueue_I2C_INSTR, &i2c_data);

    /* Debug*/
    #ifdef DEBUG
        USART1PutString("1",1);
    #endif

    switch (I2C_GetLastEvent(I2C1))
    {

        case I2C_EVENT_MASTER_MODE_SELECT:

            /* READ OP(operation) code*/
            if(i2c_data.op_code == 1 && state == 0){ /* OP:read  before set the slave address*/

                   I2C_Send7bitAddress(I2C1, i2c_data.address, I2C_Direction_Transmitter);

            } else if(i2c_data.op_code==1 && state == 1){ /* OP:read  after set the slave address*/

                I2C_Send7bitAddress(I2C1, i2c_data.address, I2C_Direction_Receiver);
            }
            /* End READ OP(operation) code*/

            /* Start of Write OP */
            else if( i2c_data.op_code = 2 && state == 0){

                I2C_Send7bitAddress(I2C1, i2c_data.address, I2C_Direction_Transmitter);

            } else if ( i2c_data.op_code == 2 && state == 1){

                I2C_Send7bitAddress(I2C1, i2c_data.address, I2C_Direction_Transmitter);

            }
            /* End of write OP*/

            /* Debug*/
            #ifdef DEBUG
                USART1PutString("6",1);
            #endif

            break;


        case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:

            /* READ OP(operation) code*/
            if(i2c_data.op_code == 1 && state == 1){ /* if the OP:read after set the slave address*/
                /* INT to generate received event */
                I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);

                /* not acknowledge we only need to read one byte */
                I2C_AcknowledgeConfig(I2C1, DISABLE);
            }
            /* End READ OP(operation) code*/

            #ifdef DEBUG
                USART1PutString("7",1);
            #endif

            break;


        case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:

            /* READ OP(operation) code*/
            if(i2c_data.op_code == 1 && state == 0){ /* if the "global" instruction is read*/
                /* Set the registe of slave from we want to read*/
                I2C_SendData(I2C1, i2c_data.reg);
            }
            /* End READ OP(operation) code*/

            /*  Start od write operation*/
            else if (i2c_data.op_code == 2 && state == 0 ) {

                I2C_SendData(I2C1, i2c_data.reg);

            }
            else if (i2c_data.op_code == 2 && state == 1){

                I2C_SendData(I2C1, i2c_data.data);
            }
            /* end of write operation*/


            #ifdef DEBUG
                USART1PutString("3",1);
            #endif

            break;

        case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
            /* Nothing to do */

            #ifdef DEBUG
                USART1PutString("4", 1);
            #endif

            break;

        case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
            /* READ OP(operation) code */
            if(i2c_data.op_code == 1 && state == 0){
                //I2C_GenerateSTOP(I2C1, ENABLE);
                I2C_GenerateSTART(I2C1, ENABLE);
                state ++;

            }

            else if(i2c_data.op_code == 2 && state == 0 )
            {
                I2C_GenerateSTART(I2C1, ENABLE);
                state ++;
            }

            else if(i2c_data.op_code == 2 && state == 1)
            {
                I2C_GenerateSTOP(I2C1, ENABLE);
            }


            /*Debug*/
            #ifdef DEBUG
                USART1PutString("5", 1);
            #endif

            break;

        case I2C_EVENT_MASTER_BYTE_RECEIVED:
            /* READ OP(operation) code*/

            if(i2c_data.op_code == 1 && state == 1){
                /*Disable the interrupt */
                I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE );

                /* Read the byte */
                rx_aux= I2C_ReceiveData(I2C1);

                /*End of comunication */
                I2C_GenerateSTOP(I2C1, ENABLE);

                /*Send the readed byte to Queue*/
                xQueueSendToBackFromISR( xQueue_I2C_RX , &rx_aux, &pxHigherPriorityTaskWoken);

                /*Reset the static static var*/
                state = 0;

                /*The work is done - read (to clean) the queue*/
                xQueueReceiveFromISR(xQueue_I2C_INSTR, &i2c_data, &pxHigherPriorityTaskWoken );

                /* Lets check if there's more work to do...*/
                if(!xQueueIsQueueEmptyFromISR( xQueue_I2C_INSTR )){
                    I2C_GenerateSTART(I2C1, ENABLE);
                }


            }

            /* End READ OP(operation) code*/

            #ifdef DEBUG
                USART1PutString("8",1);
            #endif
            break;


    default:
        break;
    }


    if( pxHigherPriorityTaskWoken == pdTRUE )
        taskYIELD(); /* forces a context switch before exit the ISR */

}



void I2C1_ER_IRQHandler(void)
{
    USART1PutString("isr_err", 8);


    /* Check on I2C1 AF flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_AF))
    {

    }

}

