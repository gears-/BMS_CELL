#include "communication.h"
#include "stm32l0xx_hal.h"
#include "usart.h"

#define READ	0;
#define WRITE	1;
#define IGNORE_KEY ((uint16_t) (0xF00))

#define RX_CRC(raw) (raw & 0x0000FF);
#define RX_COMMAND(raw) ((raw >> 8) & 0x000007);
#define RX_MODE(raw) ((raw >> 0xB) & 0x000001);
#define RX_DATA(raw) ((raw >> 0xC) & 0x000FFF);

//uint32_t RECEIVE, SEND;
char  buffer[3];
uint32_t Tx_Data;

void Send_Updated_Packet(uint16_t data)
{
	Tx_Data = ((data << 0xC) & 0x00FFF000) + IGNORE_KEY;
	Tx_Data = Tx_Data + Calculate_CRC(Tx_Data);
	buffer[2] = Tx_Data & 0x0000FF;
	buffer[1] = (Tx_Data >> 0x8) & 0xFF;
	buffer[0] = (Tx_Data >> 0x10) & 0xFF;
	HAL_UART_Transmit(&huart2, buffer , 3, 1000);

}
