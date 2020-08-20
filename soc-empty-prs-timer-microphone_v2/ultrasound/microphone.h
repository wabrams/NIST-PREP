#include "ultrasound_define.h"
#define PP_BUFFER_SIZE               256

#define PDM_INTERRUPTS
#define RECORD_TX

#ifndef _MICROPHONE_H
#define _MICROPHONE_H
#include "bg_types.h"

#include "app.h"
#include "em_cmu.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_ldma.h"
#include "dmadrv.h"

#include "em_timer.h"
#include "em_rtcc.h"

#include "gpiointerrupt.h"

// PDM global variables
extern int16_t left[BUFFER_SIZE];
extern int16_t right[BUFFER_SIZE];
extern int32_t corr[BUFFER_SIZE];
extern uint8_t pdm_template[512];
extern int N_pdm_template;

extern uint32_t pingBuffer[PP_BUFFER_SIZE];
extern uint32_t pongBuffer[PP_BUFFER_SIZE];
extern bool prevBufferPing;
extern unsigned int ldma_channelPDM;
extern bool recording;

// functions
extern void initPDM(void);
extern void startLDMA_PDM(void);
extern void initTimer0(void);
extern void TIMER0_IRQHandler(void);

#endif
