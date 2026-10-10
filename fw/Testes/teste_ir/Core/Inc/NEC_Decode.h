/*
 * NEC_Decode.h
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#ifndef INC_NEC_DECODE_H_
#define INC_NEC_DECODE_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>



typedef enum {
    NEC_NOT_EXTENDED, NEC_EXTENDED
} NEC_TYPE;

typedef enum {
	NEC_OK, NEC_INIT, NEC_AGC_OK, NEC_AGC_FAIL, NEC_FAIL
} NEC_STATE;

typedef struct {
    uint32_t rawTimerData[32];
    uint8_t decoded[4];
    uint16_t Address;
    uint8_t Command;

    NEC_STATE state;

    // The timer of value to indicate if the value is 1 or 0
    // if the value of Input Capture is > timingBitBoundary = '1'
    // else if Input Capture is < timingBitBoundary = '0'
    uint16_t timingBitBoundary;
    // The timing of the Pulse of Agc, if the value is less
    // than the timingAgcBoundary, we have a Repeat Code
    uint16_t timingAgcBoundary;
    NEC_TYPE type;

    void (*NEC_InCapt_Start)(uint32_t *Arr, uint8_t len);
    void (*NEC_InCapt_Stop)();

    void (*NEC_DecodedCallback)(uint16_t Addr, uint8_t Cmd);
    void (*NEC_ErrorCallback)();
    void (*NEC_RepeatCallback)();
} NEC;

// Init NEC
NEC_STATE NEC_Init(NEC* handle);

// deinit de NEC
void NEC_DeInit(NEC* handle);

// Callback of the timer
void NEC_TIM_IC_CaptureCallback(NEC* handle);

// Start Read one time. Need to be called every Callback
void NEC_Read(NEC* handle);

#endif /* INC_NEC_DECODE_H_ */
