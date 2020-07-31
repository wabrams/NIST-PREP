#include <stdio.h>

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

#define CONSTANT_TONE 0

#define FREQ_ONE 20000
#define FREQ_TWO 30000
#define FREQ_AVG ((FREQ_ONE + FREQ_TWO) / 2)
#define FREQ_GAP (FREQ_TWO - FREQ_ONE)
#define FREQ_ONE_TO_TWO 1 //direction to change frequency in (false: two to one, true: one to two)

#define DUTY_CYCLE_STEPS  0.05
#define TARGET_DUTY_CYCLE 0.55

#define LDMA_CHANNEL                     0
#define LDMA_CH_MASK    (1 << LDMA_CHANNEL)

#define BUFFER_SIZE                   1024
#define PP_BUFFER_SIZE                 128

int16_t left[BUFFER_SIZE];
int16_t right[BUFFER_SIZE];
LDMA_Descriptor_t descLink[2];
uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];
bool prevBufferPing;

float timeChirp = 2.0; //in mS
uint32_t chirpN;
uint32_t topOne, topTwo;
float topValue, topInc;
uint32_t timerFreq;

bool chirpEn = false;

typedef enum squarechirp_mode_e
{
  chirpInc,
  chirpOn,
  chirpDec,
  chirpOff
} squarechirp_mode_t;

static volatile squarechirp_mode_t chirp_mode = chirpInc;
static volatile float dutyCycle = 0;

void TIMER0_IRQHandler(void)
{
  uint32_t flags = TIMER0 -> IF_CLR = TIMER0 -> IF;
  // Acknowledge the interrupt
  if (flags & TIMER_IF_CC1)
  {
#if CONSTANT_TONE

#else
    switch (chirp_mode)
    {
      case chirpInc:
        dutyCycle += DUTY_CYCLE_STEPS;
        if (dutyCycle >= TARGET_DUTY_CYCLE)
        {
          chirp_mode = chirpOn;
          dutyCycle = TARGET_DUTY_CYCLE;
        }
        TIMER_CompareBufSet(TIMER0, 1, (uint32_t)(topValue * dutyCycle));
        break;
      case chirpOn:
        #if FREQ_ONE_TO_TWO
          topValue -= topInc;
          if (topValue <= topTwo)
          {
            topValue = topTwo;
            chirp_mode = chirpDec;
          }
        #else
          topValue += topInc;
          if (topValue >= topOne)
          {
            topValue = topOne;
            chirp_mode = chirpDec;
          }
        #endif
        TIMER_CompareBufSet(TIMER0, 1, (uint32_t)(topValue * dutyCycle));
        TIMER_TopBufSet(TIMER0, topValue);
        break;
      case chirpDec:
        dutyCycle -= DUTY_CYCLE_STEPS;
        if (dutyCycle <= 0)
        {
          chirp_mode = chirpOff;
          dutyCycle = 0;
        }
        TIMER_CompareBufSet(TIMER0, 1, (uint32_t)(topValue * dutyCycle));
        break;
      case chirpOff:
        #if FREQ_ONE_TO_TWO
          topValue = topOne;
        #else
          topValue = topTwo;
        #endif
        TIMER_TopSet(TIMER0, topValue);
        TIMER_Enable(TIMER0, false);
        chirp_mode = chirpInc;
        chirpEn = false;
        break;
    }
#endif
  }
}


void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PDM, true);
    CMU_ClockSelectSet(cmuClock_PDM, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_TIMER0, true);
    CMU_ClockSelectSet(cmuClock_TIMER0, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_RTCC, true);
    CMU_ClockSelectSet(cmuClock_RTCC, cmuSelect_LFRCO);
}

void initGPIO(void)
{
  //gpio
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);     // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);     // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);        // PDM_DATA
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
  //pdm
  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT ) | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT );
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
  //timer
  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[0].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
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

void initTIMER(void)
{
  TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
    timer0Init.prescale = timerPrescale1;
    timer0Init.enable = false;
    timer0Init.debugRun = false;
//    timer0Init.riseAction = timerInputActionReloadStart;
  TIMER_Init(TIMER0, &timer0Init);

//  TIMER_InitCC_TypeDef timer0CC0Init = TIMER_INITCC_DEFAULT;
//    timer0CC0Init.mode = timerCCModeCapture;
//  TIMER_InitCC(TIMER0, 0, &timer0CC0Init);

  TIMER_InitCC_TypeDef timer0CC1Init = TIMER_INITCC_DEFAULT;
    timer0CC1Init.mode = timerCCModePWM;
  TIMER_InitCC(TIMER0, 1, &timer0CC1Init);

  // Start with 10% duty cycle
#if CONSTANT_TONE
  dutyCycle = TARGET_DUTY_CYCLE;
#else
  dutyCycle = DUTY_CYCLE_STEPS;
#endif
  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timer0Init.prescale + 1);
  topOne = timerFreq / FREQ_ONE;
  topTwo = timerFreq / FREQ_TWO;
  chirpN = timeChirp * FREQ_AVG / 1000.0;
  topInc = ((float)FREQ_GAP / (float)(FREQ_ONE * FREQ_TWO)) * timerFreq / chirpN;
  #if FREQ_ONE_TO_TWO
    topValue = topOne;
  #else
    topValue = topTwo;
  #endif
  TIMER_TopSet(TIMER0, topValue);
  TIMER_CompareSet(TIMER0, 1, (uint32_t)(topValue * dutyCycle));

  TIMER0 -> IEN = TIMER_IEN_CC1;
  NVIC_EnableIRQ(TIMER0_IRQn);
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

  if (chirpEn)
  {
    TIMER_Enable(TIMER0, true);
  }
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
  bool lastPing = prevBufferPing;

  while (offset < BUFFER_SIZE)
  {
    while (lastPing == prevBufferPing)
    {
      EMU_EnterEM1();
    }
    lastPing = prevBufferPing;

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

  char c = 'i';

  do
  {
    switch (c) //TODO: need a listen + transmit mode
    {
      case 'a': // all (chirp + listen, transmit)
        chirpEn = true;
        listen();
        printData();
        break;
      case 'c': // chirp
        chirpEn = true;
        break;
      case 'i':
        break;
      case 'l': // listen
        listen();
        break;
      case 'r': // record (chirp + listen)
        chirpEn = true;
        listen();
        break;
      case 's': // sample numbers
        printf("samp\r\n%d\r\n", BUFFER_SIZE);
        RETARGET_SerialFlush();
        break;
      case 't': // transmit
        printData();
        break;
    }
    c = RETARGET_ReadChar();
  }
  while (c != 'q');
}
