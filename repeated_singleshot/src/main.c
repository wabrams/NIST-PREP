#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_pdm.h"
#include "em_device.h"
#include "em_timer.h"
#include "em_chip.h"
#include "em_rtcc.h"

#include "retargetserialconfig.h"
#include "retargetserial.h"
#include "stdio.h"

#define LDMA_CHANNEL        				     0
#define LDMA_CH_MASK    (1 << LDMA_CHANNEL)

#define BUFFER_SIZE 		              1024
#define PP_BUFFER_SIZE                 128

#define SINGLE_SHOT 1

int16_t left[BUFFER_SIZE];
int16_t right[BUFFER_SIZE];
LDMA_Descriptor_t descLink[2];
uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];
bool prevBufferPing;

uint32_t square_cnts = 6 * 4;
uint32_t dur_freq = 400;

uint32_t start, stop, durr;
uint32_t measured_ms = 0;


void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PDM, true);
    CMU_ClockSelectSet(cmuClock_PDM, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_TIMER0, true);
    CMU_ClockSelectSet(cmuClock_TIMER0, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_TIMER1, true);
    CMU_ClockSelectSet(cmuClock_TIMER1, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_RTCC, true);
    CMU_ClockSelectSet(cmuClock_RTCC, cmuSelect_LFRCO);
}

void initRTCC(void)
{
  RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
    rtccInit.debugRun = false;
    rtccInit.enable = true;
    rtccInit.presc = rtccCntPresc_1;
    rtccInit.prescMode = rtccCntTickPresc;
  RTCC_Init(&rtccInit);
}

void initGPIO(void)
{
  // Config GPIO and pin routing
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);     // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);     // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);        // PDM_DATA
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);     // TIMER0
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);     // SPEAKER / TIMER1

  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT ) | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT );
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);

  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  GPIO->TIMERROUTE[1].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN;
  GPIO->TIMERROUTE[1].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[1].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
}

void initTIMER(void)
{
  // Initialize TIMER0
  TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
    timer0Init.prescale = timerPrescale32;
    timer0Init.enable   = false;
    timer0Init.debugRun = false;
#if SINGLE_SHOT
    timer0Init.fallAction = timerInputActionStop;
#endif
  TIMER_Init(TIMER0, &timer0Init);

  TIMER_InitCC_TypeDef timer0CC0Init = TIMER_INITCC_DEFAULT;
    timer0CC0Init.mode = timerCCModeCompare;
    timer0CC0Init.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER0, 0, &timer0CC0Init);

  uint32_t max_freq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timer0Init.prescale + 1);
  int topValue = max_freq / (2*dur_freq);
  TIMER_TopSet(TIMER0, topValue);

  // Initialize TIMER1
  TIMER_Init_TypeDef timer1Init = TIMER_INIT_DEFAULT;
    timer1Init.prescale = timer0Init.prescale;
    timer1Init.enable   = false;
    timer1Init.debugRun = false;
    timer1Init.riseAction = timerInputActionReloadStart;
    timer1Init.fallAction = timerInputActionStop;
  TIMER_Init(TIMER1, &timer1Init);

  TIMER_InitCC_TypeDef timer1CC1Init = TIMER_INITCC_DEFAULT;
    timer1CC1Init.mode = timerCCModeCompare;
    timer1CC1Init.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER1, 1, &timer1CC1Init);

  TIMER_InitCC_TypeDef timer1CC0Init = TIMER_INITCC_DEFAULT;
      timer1CC0Init.mode = timerCCModeCapture;
  TIMER_InitCC(TIMER1, 0, &timer1CC0Init);

  TIMER_TopSet(TIMER1, square_cnts / 2);
}

void initPDM(void)
{
  // Config PDM
  PDM -> CFG0 = PDM_CFG0_STEREOMODECH01_CH01ENABLE
              | PDM_CFG0_CH0CLKPOL_NORMAL
              | PDM_CFG0_CH1CLKPOL_INVERT
              | PDM_CFG0_FIFODVL_FOUR
              | PDM_CFG0_DATAFORMAT_DOUBLE16
              | PDM_CFG0_NUMCH_TWO
              | PDM_CFG0_FORDER_FIFTH;
  PDM -> CFG1 = (5 << _PDM_CFG1_PRESC_SHIFT);
  // Enable module
  PDM -> EN = PDM_EN_EN;
  // Start filter
  while (PDM -> SYNCBUSY);
  PDM->CMD = PDM_CMD_START;
  // Config DSR/Gain
  while (PDM -> SYNCBUSY);
  // Changed: From 3 to 8
  PDM -> CTRL = (8 << _PDM_CTRL_GAIN_SHIFT) | (32 << _PDM_CTRL_DSR_SHIFT);
}

void initLDMA(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // LDMA transfers trigger on PDM RX Data Valid
  LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_PDM_RXDATAV);

  // Link descriptors for ping-pong transfer
  descLink[0] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pingBuffer, PP_BUFFER_SIZE, +1);
  descLink[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pongBuffer, PP_BUFFER_SIZE, -1);
  // Next transfer writes to pingBuffer
  prevBufferPing = false;

  LDMA_Init(&init);
  LDMA_StartTransfer(LDMA_CHANNEL, (void*)&periTransferTx, (void*)&descLink);
}

void LDMA_IRQHandler(void)
{
  uint32_t pending = LDMA_IntGet();
  LDMA_IntClear(pending);

  if(pending & LDMA_IF_ERROR)
  {
    while(1); //TODO: assert here
  }

  prevBufferPing = !prevBufferPing;
}

static void initialize()
{
  // Chip errata
  CHIP_Init();
  // Initialize LDMA and PDM
  initCMU();
  initRTCC();
  initGPIO();
  initTIMER();
  initPDM();
  initLDMA();
}

static void listen(void)
{
  int offset = 0;

  while (offset < BUFFER_SIZE)
  {
    EMU_EnterEM1();
    if(prevBufferPing)
    {
      for(int i = 0; i < PP_BUFFER_SIZE; i++)
      {
        left[i + offset] = pingBuffer[i] & 0x0000FFFF;
        right[i + offset] = (pingBuffer[i] >> 16) & 0x0000FFFF;
      }
    }
    else
    {
      for(int i = 0; i < PP_BUFFER_SIZE; i++)
      {
        left[offset + i] = pongBuffer[i] & 0x0000FFFF;
        right[offset + i] = (pongBuffer[i] >> 16) & 0x0000FFFF;
      }
    }
    offset += PP_BUFFER_SIZE;
  }
}

static void printData(void)
{
  printf("left\r\n");
  for (int i = 0; i < BUFFER_SIZE; i++)
    printf("%d ", left[i]);
  printf("\r\n");
  RETARGET_SerialFlush();

  printf("right\r\n");
  for (int i = 0; i < BUFFER_SIZE; i++)
    printf("%d ", right[i]);
  printf("\r\n");
  RETARGET_SerialFlush();
}

int main(void)
{
  initialize();

  char c = 'c';

  do
  {
	  if (c == 'r')
	  {
//	    start = RTCC_CounterGet();

	    TIMER_Enable(TIMER0, true);
	    listen();
      printData();

//	    stop = RTCC_CounterGet();
//	    durr = stop - start;
//	    measured_ms = durr * 0.0305;
	  }
	  c = RETARGET_ReadChar();
  }
  while (c != 'q');
}

