
void USART1Init(uint32_t baudrate, uint16_t buffer_depth );

char USART1GetChar();

void USART1PutChar(char put_char);

void USART1PutString(char *put_string, uint8_t string_length);

void USART1Close();
