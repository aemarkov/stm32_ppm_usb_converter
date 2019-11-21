#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdint.h>

#define PPM_NUM_CHANNELS   10

void usart_send_byte(uint8_t byte);
void usart_write(uint8_t* buffer, uint32_t size);

#endif
