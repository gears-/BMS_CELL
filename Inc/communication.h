#ifndef __communication_H
#define __communication_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"


extern void _Error_Handler(char *, int);
extern void Send_Updated_Packet(uint16_t data);



#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
