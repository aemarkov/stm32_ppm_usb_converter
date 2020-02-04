#include "ppm_decoder.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef enum ppm_decoder_state
{
	PPM_STATE_START, 					// Initial unknown state
	PPM_STATE_RECEIVING,			// Receiving state
} ppm_decoder_state_t;


channel_received m_channel_callback;
command_received m_command_callback;

uint8_t ppm_channel_index;
ppm_decoder_state_t ppm_current_state;

void ppm_decoder_init(channel_received channel_callback, command_received command_callback)
{
	m_channel_callback = channel_callback;
	m_command_callback = command_callback;
	
	ppm_channel_index = 0;
	ppm_current_state = PPM_STATE_START;
}

void ppm_decoder_decode(uint16_t duration)
{
	bool is_sync = duration > PPM_SYNC_THRESHOLD;
	
	switch(ppm_current_state)
	{
		case PPM_STATE_START:
			// Initial unknown state
			if(is_sync) {
				ppm_channel_index = 0;
				ppm_current_state = PPM_STATE_RECEIVING;
			}
			break;
		case PPM_STATE_RECEIVING:
			if(is_sync) {
				if(ppm_channel_index == PPM_NUM_CHANNELS) {
					// All channels received
					// All channels received, sync strobe received
					m_command_callback();
					ppm_channel_index = 0;
					ppm_current_state = PPM_STATE_RECEIVING;
				}
				else {
					// Not enough channels received
					// Looks like synchronization is failed
					ppm_current_state = PPM_STATE_START;
				}
			}
			else {
				if(ppm_channel_index < PPM_NUM_CHANNELS) {
					// One more channel received
					m_channel_callback(ppm_channel_index++, duration);
				} else {
					// Too many channels received
					// Looks like synchronization is failed
					ppm_current_state = PPM_STATE_START;
				}
			}
			break;
	};
}
