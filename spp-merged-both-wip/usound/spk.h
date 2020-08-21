#include "usound.h"

#define DUTY_CYCLE_STEPS  0.10
#define TARGET_DUTY_CYCLE 0.50


#ifndef ULTRASOUND_SPEAKER_H_
#define ULTRASOUND_SPEAKER_H_

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

void populateBuffers(int k);
void initSpeaker(void);
void startSpeaker(void);
void startDMADRV_TMR(void);

#endif /* ULTRASOUND_SPEAKER_H_ */
