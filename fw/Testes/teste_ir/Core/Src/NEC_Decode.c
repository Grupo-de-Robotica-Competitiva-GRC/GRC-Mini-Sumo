/*
 * NEC_Decode.c
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#include "NEC_Decode.h"

#if defined (NEC_DEBUG)
// TODO The printf :)
#define NEC_DEBUG(...)			printf(__VA_ARGS__)
#else
#define NEC_DEBUG(...)
#endif

void NEC_TIM_IC_CaptureCallback(NEC* handle) {

    if (handle->state == NEC_INIT) {

    	handle->NEC_InCapt_Stop();
        if (handle->rawTimerData[1] < handle->timingAgcBoundary) {
			NEC_DEBUG("Received Repeat command\n");
            handle->state = NEC_OK;
            handle->NEC_RepeatCallback();
        } else {
        	NEC_DEBUG("Received AGC command\n");
            handle->state = NEC_AGC_OK;
            handle->NEC_InCapt_Start((uint32_t*) handle->rawTimerData, 32);
        }

    } else if (handle->state == NEC_AGC_OK) {

    	handle->NEC_InCapt_Stop();
    	NEC_DEBUG("Received the Data\n");
        for (int pos = 0; pos < 32; pos++) {
            int time = handle->rawTimerData[pos];
            if (time > handle->timingBitBoundary) {
                handle->decoded[pos / 8] |= 1 << (pos % 8);
            } else {
                handle->decoded[pos / 8] &= ~(1 << (pos % 8));
            }
        }

        uint8_t valid = 1;

        uint8_t naddr = ~handle->decoded[0];
        uint8_t ncmd = ~handle->decoded[2];

        // check if the Address is match with the complementaryn Address
        // only if is NOT Extended
        if (handle->type == NEC_NOT_EXTENDED && handle->decoded[1] != naddr)
            valid = 0;
        // Check if Command matches the complementary
        if (handle->decoded[3] != ncmd)
            valid = 0;

        handle->state = NEC_OK;

        if (valid){
        	handle->Command = handle->decoded[2];
        	if (handle->type == NEC_NOT_EXTENDED)
        		handle->Address = handle->decoded[0];
        	else
        		handle->Address = handle->decoded[0] | (handle->decoded[1] << 8);
            NEC_DEBUG("NEC packet is valid! We received the following data:\n");
            NEC_DEBUG("> Address: 0x%04X\n"
            		  "> Command: %d\n",
            		handle->Address, handle->Command);
        	handle->NEC_DecodedCallback(handle->Address, handle->Command);
        }
		else
			NEC_DEBUG("The received packet is corrupted or incorrectly configured\n");
            handle->NEC_ErrorCallback();
    }
}

NEC_STATE NEC_Init(NEC* handle){
	if (handle->NEC_DecodedCallback == NULL ||
		handle->NEC_ErrorCallback == NULL ||
		handle->NEC_InCapt_Start == NULL ||
		handle->NEC_InCapt_Stop == NULL ||
		handle->NEC_RepeatCallback == NULL){

		NEC_DEBUG("Failed to Start, missing functions for &08X handle", (uint32_t)handle);
		return NEC_FAIL;
		}
	memset(handle->rawTimerData, 0, sizeof(handle->rawTimerData));
	NEC_DEBUG("NEC decoded start for %08X Handle", (uint32_t)handle);

	return NEC_OK;
}

void NEC_DeInit(NEC* handle){
	handle->NEC_InCapt_Stop();
}

void NEC_Read(NEC* handle) {
    handle->state = NEC_INIT;
    handle->NEC_InCapt_Start((uint32_t*) handle->rawTimerData, 2);
    NEC_DEBUG("Started NEC Reading\n");
}
