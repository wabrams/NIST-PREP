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
#include "em_chip.h"
#include "retargetserialconfig.h"
#include "stdio.h"
#include "retargetserial.h"

#define LDMA_CHANNEL        				 0
#define LDMA_CH_MASK  		(1 << LDMA_CHANNEL)

#define BUFFER_SIZE 		              1024
#define PP_BUFFER_SIZE                 128

int16_t left[BUFFER_SIZE];
int16_t right[BUFFER_SIZE];

LDMA_Descriptor_t descLink[2];

uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];
bool prevBufferPing;

void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PDM, true);
  CMU_ClockSelectSet(cmuClock_PDM, cmuSelect_HFRCODPLL); // 19 MHz
}

void initGPIO(void)
{
  // Config GPIO and pin routing
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);    // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);    // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);       // PDM_DATA

  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT ) | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT );
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
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
  // Changed: From 5 to 20
  PDM -> CFG1 = (20 << _PDM_CFG1_PRESC_SHIFT);
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

  // LDMA transfers trigger on PDM Rx Data Valid
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
  initGPIO();
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
	    listen();
      printData();
	  }
	  c = RETARGET_ReadChar();
  }
  while (c != 'q');
}

