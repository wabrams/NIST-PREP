#include "microphone.h"
#include "simple_dsp.h"
#include "em_pdm.h"

int16_t left[BUFFER_SIZE];
int16_t right[BUFFER_SIZE];
int32_t corr[BUFFER_SIZE];
uint8_t pdm_template[512];
int N_pdm_template;

uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];
bool prevBufferPing;
unsigned int ldma_channelPDM;
bool recording = false;


void initPDM(void)
{
  // configgure clock
  CMU_ClockEnable(cmuClock_PDM, true);
  // configure PRS
#ifdef RECORD_TX
  PRS_ConnectConsumer(TX_OBS_PRS_CHANNEL, prsTypeAsync, prsConsumerTIMER1_CC0);
#else
  PRS_ConnectConsumer(RX_OBS_PRS_CHANNEL, prsTypeAsync, prsConsumerTIMER1_CC0);
#endif
  //configure gpio for pdm
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);     // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);     // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);        // PDM_DATA
  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT)
      | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT)
      | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT)
      | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
  // Config PDM registers
  PDM->CFG0 = PDM_CFG0_STEREOMODECH01_CH01ENABLE | PDM_CFG0_CH0CLKPOL_NORMAL
      | PDM_CFG0_CH1CLKPOL_INVERT | PDM_CFG0_FIFODVL_FOUR | PDM_CFG0_DATAFORMAT_DOUBLE16
      | PDM_CFG0_NUMCH_TWO | PDM_CFG0_FORDER_FIFTH;
  PDM->CFG1 = (5 << _PDM_CFG1_PRESC_SHIFT);
  // Enable module
#ifdef PDM_INTERRUPTS
  PDM->IEN |= PDM_IEN_UF | PDM_IEN_OF;
  NVIC_EnableIRQ(PDM_IRQn);
#endif
  PDM->EN = PDM_EN_EN;
  while (PDM->SYNCBUSY);
  // Start filter
  PDM->CMD = PDM_CMD_START;
  while (PDM->SYNCBUSY);
  // Config DSR/Gain
  // Changed: From 3 to 8
  PDM->CTRL = (8 << _PDM_CTRL_GAIN_SHIFT) | (32 << _PDM_CTRL_DSR_SHIFT);
}

void PDM_IRQHandler(void)
{
  uint32_t flags = PDM->IF_CLR = PDM->IF;

  static int count = 0;

  if (recording)
  {
    if (flags & PDM_IF_OF)
    {
      printLog("PDM OF\r\n");
      count++;
      if (count > 200)
      {
        printLog("Hit 200 OF's...disable for now");
        PDM->IEN &= ~(PDM_IEN_UF | PDM_IEN_OF); //disable interrupts
        count = 0;
      }
    }
    if (flags & PDM_IF_UF)
    {
      printLog("PDM UF\r\n");
    }
  }
}

bool pdm_dma_cb(unsigned int channel, unsigned int sequenceNo, void *userParam)
{
  // printf("pdm_ldma_cb\r\n");
  static int offset = 0;
  static int index = 0;

  int start;

  if (offset == 0)
  {
    memset(left, 0, BUFFER_SIZE << 1);
    memset(right, 0, BUFFER_SIZE << 1);

  }
  prevBufferPing = !prevBufferPing;
  start = RTCC_CounterGet();
  if (recording)
  {
    if (offset < BUFFER_SIZE)
    {

      if (prevBufferPing)
      {
        for (int i = 0; i < PP_BUFFER_SIZE; i++)
        {
          left[i + offset] = pingBuffer[i] & 0x0000FFFF;
          right[i + offset] = (pingBuffer[i] >> 16) & 0x0000FFFF;
        }
      }
      else
      {
        for (int i = 0; i < PP_BUFFER_SIZE; i++)
        {
          left[offset + i] = pongBuffer[i] & 0x0000FFFF;
          right[offset + i] = (pongBuffer[i] >> 16) & 0x0000FFFF;
        }
      }

      offset += PP_BUFFER_SIZE;
    }
    // printLog("dma_cb: channel %d, sequenceNo %d, offset %d\r\n", channel, sequenceNo, offset);
    if (offset == BUFFER_SIZE)
    {
#ifdef PDM_INTERRUPTS
      PDM->IEN &= ~(PDM_IEN_UF | PDM_IEN_OF); //disable interrupts
#endif
      offset = 0;
      index = 0;
      DMADRV_StopTransfer(channel);
      recording = false;
      printLog("done recording\r\n\r\n");
      float g = goertzel(left, PP_BUFFER_SIZE);
      printLog("goertzel: %f\r\n", g);
    }
  }
  return 0;
}

void startLDMA_PDM(void)
{
  static LDMA_Descriptor_t descLink[2];
  // static LDMA_TransferCfg_t periTransferTx;
  // PDM FIFO:
  static bool need_setup = true;
  LDMA_TransferCfg_t periTransferTx =
  LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_PDM_RXDATAV);
  if (need_setup)
  {

    // Link descriptors for ping-pong transfer
    descLink[0] = (LDMA_Descriptor_t )
        LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pingBuffer, PP_BUFFER_SIZE, +1);
    descLink[1] = (LDMA_Descriptor_t )
        LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pongBuffer, PP_BUFFER_SIZE, -1);
    need_setup = false;
  }
  // Next transfer writes to pingBuffer
  prevBufferPing = false;
//	LDMA_StartTransfer(LDMA_PDM_CHANNEL, (void*) &periTransferTx,
//			(void*) &descLink);

  DMADRV_LdmaStartTransfer(ldma_channelPDM, (void*) &periTransferTx, (void*) &descLink, pdm_dma_cb,
  NULL);

}

void initTimer0(void)
{
  // Enable clock for TIMER0 module
  CMU_ClockEnable(cmuClock_TIMER0, true);

  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.enable = false;
    timerInit.clkSel = timerClkSelHFPerClk;
  TIMER_Init(TIMER0, &timerInit);

  // Configure the TIMER0 module for Capture mode and to trigger on every other edge
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
//  timerCCInit.eventCtrl = timerEventEveryEdge; // timerEventEvery2ndEdge;
//  timerCCInit.edge = timerEdgeRising;  // timerEdgeBoth; // Trigger an input capture on every edge
  timerCCInit.eventCtrl = timerEventEvery2ndEdge;
  timerCCInit.edge = timerEdgeBoth; // Trigger an input capture on every edge
  timerCCInit.mode = timerCCModeCapture;
  timerCCInit.prsSel = TX_OBS_PRS_CHANNEL;
  timerCCInit.prsInput = true;
  timerCCInit.prsInputType = timerPrsInputAsyncPulse;
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  // Enable TIMER0 interrupts
  TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
  NVIC_EnableIRQ(TIMER0_IRQn);

  // Enable the TIMER
  TIMER_Enable(TIMER0, true);

}

void TIMER0_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER0);
  TIMER_IntClear(TIMER0, flags);
//  static uint32_t prev = 0;
//  uint32_t curr = RTCC_CounterGet();

//  uint32_t firstEdge;
//  uint32_t secondEdge;
//  // Read the last two captured edges
//  // Note: interrupt occurs after the second capture
//  firstEdge = TIMER_CaptureGet(TIMER0, 0);
//  secondEdge = TIMER_CaptureGet(TIMER0, 0);
//  printLog("rtcc:%lu\r\n", curr - prev);
//  prev = curr;
  recording = true;
  startLDMA_PDM();

#ifdef PDM_INTERRUPTS
  PDM->IEN |= PDM_IEN_UF | PDM_IEN_OF; //enable interrupts
#endif
}
