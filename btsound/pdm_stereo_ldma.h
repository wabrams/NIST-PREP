// standard:
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
// emlib/
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_pdm.h"


#ifndef PDM_STEREO_LDMA_H
#define PDM_STEREO_LDMA_H

// DMA channel used for the example
#define LDMA_CHANNEL        0
#define LDMA_CH_MASK        (1 << LDMA_CHANNEL)
#define BUFFER_SIZE         1024
#define PP_BUFFER_SIZE      128 //TODO: 64 or 128?

void initPdm(void);
void initLdma(void);
void stopLdma(void);
void pdmPass(void);
void pdmReset(void);
int pdmDone(void);
void pdmPrintLeft(void);
void pdmPrintRight(void);

#endif /* PDM_STEREO_LDMA_H */
