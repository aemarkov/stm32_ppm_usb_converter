#ifndef PPM_DECODER_H
#define PPM_DECODER_H

#include <stdint.h>

#define PPM_SYNC_THRESHOLD  3000


typedef void(*channel_received)(uint8_t channel, uint16_t duration);
typedef void(*command_received)(void);

// Initialize the ppm_decoder
// channel_callback  Callback to process value of the channel. Will be called after receiving every channel
// command_callback  Callback to notify that all channels are received
void ppm_decoder_init(channel_received channel_callback, command_received command_callback);

// Process captured impulse
void ppm_decoder_decode(uint16_t duration);

#endif
