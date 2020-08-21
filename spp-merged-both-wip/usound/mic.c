#include "mic.h"
//#include "simple_dsp.h"
#include "em_pdm.h"
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

// extern float pulse_width;
#define PDM_INTERRUPTS
#define BUFFER_SIZE 1024
#define PP_BUFFER_SIZE 256

// PDM stuff
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
// end PDM stuff
#define RECORD_TX

void initPDM(void)
{
  // setup ldma for pdm
  uint32_t e = DMADRV_AllocateChannel(&ldma_channelPDM, NULL);

  printLog("\t[DMA*]> CH:%1d for MIC, EC:%lu\r\n", ldma_channelPDM, e);
  // configgure clock
  CMU_ClockEnable(cmuClock_PDM, true);

  //configure gpio for pdm
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);     // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);     // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);        // PDM_DATA
  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT)  | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
}

void pdm_start(void)
{
  // Config PDM registers
  PDM->CFG0 = PDM_CFG0_STEREOMODECH01_CH01ENABLE | PDM_CFG0_CH0CLKPOL_NORMAL
      | PDM_CFG0_CH1CLKPOL_INVERT | PDM_CFG0_FIFODVL_FOUR | PDM_CFG0_DATAFORMAT_DOUBLE16
      | PDM_CFG0_NUMCH_TWO | PDM_CFG0_FORDER_FIFTH;
  PDM->CFG1 = (5 << _PDM_CFG1_PRESC_SHIFT);
#ifdef PDM_INTERRUPTS
  PDM->IEN |= PDM_IEN_UF | PDM_IEN_OF;
  NVIC_EnableIRQ(PDM_IRQn);
#endif
  // Enable module
  PDM->EN = PDM_EN_EN;
  // Start filter
  while (PDM -> SYNCBUSY);
  PDM->CMD = PDM_CMD_START;
  // Config DSR/Gain
  while (PDM -> SYNCBUSY);
  // Changed: From 3 to 8
  PDM->CTRL = (8 << _PDM_CTRL_GAIN_SHIFT) | (32 << _PDM_CTRL_DSR_SHIFT);
}

void pdm_pause()
{
  DMADRV_StopTransfer(ldma_channelPDM);
  recording = false;
  // stop pdm filter
  while (PDM -> SYNCBUSY);
  PDM -> CMD = PDM_CMD_STOP;
  // clear pdm filter
  while (PDM -> SYNCBUSY);
  PDM -> CMD = PDM_CMD_CLEAR;
  // flush pdfm fifo
  while (PDM -> SYNCBUSY);
  PDM -> CMD = PDM_CMD_FIFOFL;
  // disable
  while (PDM -> SYNCBUSY);
  PDM -> EN = PDM_EN_EN_DISABLE;
}

void PDM_IRQHandler(void)
{
  uint32_t flags = PDM -> IF_CLR = PDM -> IF;
  static int count = 0;

  if (recording)
  {
    if (flags & PDM_IF_OF)
    {
      // printLog("PDM OF\r\n");
      count++;
      // flush ovflw from efr32bg22 ref manual
      while (PDM->SYNCBUSY != 0)
        ;
      PDM->CMD = PDM_CMD_FIFOFL;
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
  static int offset = 0;

  prevBufferPing = !prevBufferPing;
  // start = RTCC_CounterGet();
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
    // times[index++] = RTCC_CounterGet() - start;
    if (offset == BUFFER_SIZE)
    {
#ifdef PDM_INTERRUPTS
      PDM->IEN &= ~(PDM_IEN_UF | PDM_IEN_OF); //disable interrupts
#endif
      printLog("[DMA%1d]> MIC: sequenceNo %d, offset %d\r\n", channel, sequenceNo, offset);
      offset = 0;
      // index = 0;
      DMADRV_StopTransfer(channel);
      recording = false;
#ifdef GOERTZEL
      uint32_t t0 = RTCC_CounterGet();
      float P_left = goertzel(left, k_goertzel);
      float P_right = goertzel(right, k_goertzel);

      uint32_t curr = RTCC_CounterGet();
      printf("%ld, %ld, %ld, %d ------------:g[%ld]: %e, %e\r\n", curr, curr-prev_rtcc, curr-t0, sequenceNo,
          k_goertzel, P_left, P_right);
      prev_rtcc = curr;

      calc_chirp_v2(k_goertzel, pulse_width, pdm_template, &N_pdm_template);
      filter_biquad(left, 0);
      memset(corr, 0, BUFFER_SIZE<<2);
      calc_cross(left, BUFFER_SIZE, pdm_template, N_pdm_template, corr);
      int left_t = shape(corr);
      filter_biquad(right, 0);
      memset(corr, 0, BUFFER_SIZE<<2);
      calc_cross(right, BUFFER_SIZE, pdm_template, N_pdm_template, corr);
      int right_t = shape(corr);
      curr = RTCC_CounterGet();
      printLog("%ld, %ld ---- peak times: %d, %d\r\n", curr, curr-prev_rtcc, left_t, right_t);

      prev_rtcc = curr;
#endif
    }
  }
  return 0;
}

void startDMADRV_PDM(void)
{
  static LDMA_Descriptor_t descLink[2];
  // static LDMA_TransferCfg_t periTransferTx;
  static bool need_setup = true;

  LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_PDM_RXDATAV);
  if (need_setup)
  {
    // Link descriptors for ping-pong transfer
    descLink[0] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pingBuffer, PP_BUFFER_SIZE, +1);
    descLink[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pongBuffer, PP_BUFFER_SIZE, -1);
    need_setup = false;
  }
  // Next transfer writes to pingBuffer
  prevBufferPing = false;

  DMADRV_LdmaStartTransfer(ldma_channelPDM, (void*) &periTransferTx, (void*) &descLink, pdm_dma_cb, NULL);
}
