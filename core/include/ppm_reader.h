#ifndef PPM_READER_H
#define PPM_READER_H

#include <stdint.h>

typedef void(*impulse_received)(uint16_t duration);
void ppm_reader_init(impulse_received callback);

#endif
