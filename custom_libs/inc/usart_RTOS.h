


// /*USART Rx*/
extern QueueHandle_t xQueue_USART_Rx;

// /*USART Tx*/
extern QueueHandle_t xQueue_USART_Tx;


extern SemaphoreHandle_t xMutex_USART;

void USART1Init(uint32_t baudrate, uint16_t buffer_depth );

char USART1GetChar();

void USART1PutChar(char put_char);
void USART1PutString(char *put_string, uint8_t string_length);

void USART1Close();
