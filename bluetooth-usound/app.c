/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "app.h"
#include "em_cmu.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_ldma.h"
#include "dmadrv.h"

#include "em_timer.h"

#include "gpiointerrupt.h"

#define RX_OBS_PRS_CHANNEL   0
#define TX_OBS_PRS_CHANNEL   1

#define LR_BUFFER_SIZE     512
#define PP_BUFFER_SIZE     128

int16_t left [LR_BUFFER_SIZE];
int16_t right[LR_BUFFER_SIZE];
uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];
bool prevBufferPing = false;
int offset = 0;

volatile bool on_off = false;

//void startLDMA(void);
void startDMADRV(void);

static inline void printLR()
{
  printf("left\r\n");
  for (int i = 0; i < LR_BUFFER_SIZE; i++)
  {
    printf("%d ", left[i]);
  }
  printf("\r\n");
  printf("right\r\n");
  for (int i = 0; i < LR_BUFFER_SIZE; i++)
  {
    printf("%d ", right[i]);
  }
  printf("\r\n");
}

void button0Callback(uint8_t pin)
{
  printf("Button callback\r\n");
  if (offset == LR_BUFFER_SIZE)
  {
    printLR();
    offset = 0;
  }
  startDMADRV();
  TIMER_Enable(TIMER1, true);
}

void initGpio(void)
{
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
  // Button stuff
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPullFilter, 1);
  GPIOINT_Init();
  printLog("initgpio setup callback\r\n");
  GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, button0Callback);
  GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, false,
  true, true);
  // PDM stuff
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);     // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);     // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput,    0);     // PDM_DATA
  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT)  | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
}

void initCmu(void)
{
  // Enable clock to GPIO and TIMER1
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
  CMU_ClockEnable(cmuClock_PDM, true);
}

#define PWM_FREQ 1000
// Buffer size
#define BUFFER_SIZE 11
static const uint16_t dutyCyclePercentages[BUFFER_SIZE] =
// { 5, 8, 10, 10, 8, 5, 2, 0, 0, 2 };
// {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
// {100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100};
    { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };

// Buffer of duty cycle values for DMA transfer to CCVB
// Buffer is populated after TIMER is initialized and Top value is set
static uint16_t buffer[BUFFER_SIZE];

void populateBuffer(void)
{
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    buffer[i] = (uint16_t) (TIMER_TopGet(TIMER1) * dutyCyclePercentages[i] / 100);
}

void initTimer(void)
{
  // Initialize and start timer with no prescaling
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER1 Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  // Use PWM mode, which sets output on overflow and clears on compare events
  timerInit.enable = false;
  timerCCInit.mode = timerCCModePWM;

  // Configure but do not start the timer
  TIMER_Init(TIMER1, &timerInit);

  // Route TIMER1 CC0 output to PA6
  GPIO->TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[1].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
      | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  /*
   (gpioPortA << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
   | (6 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
   */
  TIMER_InitCC(TIMER1, 0, &timerCCInit);

  // Set top value to overflow at the desired PWM_FREQ frequency
  TIMER_TopSet(TIMER1, CMU_ClockFreqGet(cmuClock_TIMER1) / PWM_FREQ);

  // Start the timer
  // TIMER_Enable(TIMER1, true);

  // Trigger DMA on compare event to set CCVB to update duty cycle on next period
  TIMER_IntEnable(TIMER1, TIMER_IEN_CC0);
}

void initPDM(void)
{
  // Config PDM
  PDM -> CFG0 = PDM_CFG0_STEREOMODECH01_CH01ENABLE | PDM_CFG0_CH0CLKPOL_NORMAL
            | PDM_CFG0_CH1CLKPOL_INVERT | PDM_CFG0_FIFODVL_FOUR
            | PDM_CFG0_DATAFORMAT_DOUBLE16 | PDM_CFG0_NUMCH_TWO
            | PDM_CFG0_FORDER_FIFTH;
  PDM -> CFG1 = (5 << _PDM_CFG1_PRESC_SHIFT);
  // Enable module
  PDM -> EN = PDM_EN_EN;
  // Start filter
  while (PDM -> SYNCBUSY);
  PDM -> CMD = PDM_CMD_START;
  // Config DSR/Gain
  while (PDM -> SYNCBUSY);
  // Changed: From 3 to 8
  PDM -> CTRL = (8 << _PDM_CTRL_GAIN_SHIFT) | (32 << _PDM_CTRL_DSR_SHIFT);
}

unsigned int channel, channelPDM;
bool dma_cb(unsigned int channel, unsigned int sequenceNo, void *userParam)
{
  printLog("dma_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
  TIMER_Enable(TIMER1, false);

  return 0;
}
void dma_pdm_cb(unsigned int channel, unsigned int sequenceNo, void * userParam)
{
  prevBufferPing = !prevBufferPing;
  if (offset < LR_BUFFER_SIZE)
  { //i had to move this inside the if, otherwise it floods the serial terminal
    printLog("pdm_cb: channel %d, sequenceNo %d\r\n", channelPDM, sequenceNo);
    if (prevBufferPing)
    {
      for (int i = 0; i < PP_BUFFER_SIZE; i++) {
        left[i + offset] = pingBuffer[i] & 0x0000FFFF;
        right[i + offset] = (pingBuffer[i] >> 16) & 0x0000FFFF;
      }
    }
    else
    {
      for (int i = 0; i < PP_BUFFER_SIZE; i++) {
        left[offset + i] = pongBuffer[i] & 0x0000FFFF;
        right[offset + i] = (pongBuffer[i] >> 16) & 0x0000FFFF;
      }
    }
    offset += PP_BUFFER_SIZE;
  }
}
void startDMADRV(void)
{
  DMADRV_MemoryPeripheral(channel, dmadrvPeripheralSignal_TIMER1_CC0, &TIMER1->CC[0].OCB,
      &buffer, true, BUFFER_SIZE, dmadrvDataSize2, dma_cb, NULL);
}
void startDMADRV_PDM(void)
{
  DMADRV_PeripheralMemoryPingPong(channelPDM, dmadrvPeripheralSignal_PDM_RXDATAV, pingBuffer, pongBuffer,
      &(PDM->RXDATA), true, PP_BUFFER_SIZE, dmadrvDataSize4, dma_pdm_cb, NULL);
}
//void startLDMA(void)
//{
//  // Start the transfer
//  uint32_t channelNum = 0;
//  // Transfer configuration and trigger selection
//  LDMA_TransferCfg_t transferConfig =
//      LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER1_CC0);
//  // Channel descriptor configuration
//  static LDMA_Descriptor_t descriptor =
//      LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&buffer, &TIMER1->CC[0].OCB, BUFFER_SIZE);
//  descriptor.xfer.size = ldmaCtrlSizeHalf;
//  descriptor.xfer.doneIfs = 0;
//
//  // LDMA_StartTransfer(channelNum, &transferConfig, &descriptor);
//  DMADRV_LdmaStartTransfer(channel, &transferConfig, &descriptor, dma_cb,
//  NULL);
//
//}

void enableDebugGpios(GPIO_Port_TypeDef rx_obs_port, uint8_t rx_obs_pin,
    GPIO_Port_TypeDef tx_obs_port, uint8_t tx_obs_pin)
{
  // Turn on the PRS and GPIO clocks to access their registers.
  // Configure pins as output.
  GPIO_PinModeSet(rx_obs_port, rx_obs_pin, gpioModePushPull, 0);
  GPIO_PinModeSet(tx_obs_port, tx_obs_pin, gpioModePushPull, 0);

#ifdef _SILICON_LABS_32B_SERIES_2

  // Configure PRS Channel 0 to output RAC_RX.
  PRS_SourceAsyncSignalSet(0, PRS_ASYNC_CH_CTRL_SOURCESEL_RACL,
  _PRS_ASYNC_CH_CTRL_SIGSEL_RACLRX);
  PRS_PinOutput(RX_OBS_PRS_CHANNEL, prsTypeAsync, rx_obs_port, rx_obs_pin);

  /* Configure PRS Channel 0 to output RAC_RX.*/
  PRS_SourceAsyncSignalSet(1, PRS_ASYNC_CH_CTRL_SOURCESEL_RACL,
  _PRS_ASYNC_CH_CTRL_SIGSEL_RACLTX);
  PRS_PinOutput(TX_OBS_PRS_CHANNEL, prsTypeAsync, tx_obs_port, tx_obs_pin);
#elif defined(_SILICON_LABS_32B_SERIES_1)
  if((tx_obs_port != gpioPortF)||(rx_obs_port != gpioPortF))
  {
    return;
  }
  if(rx_obs_port == gpioPortF)
  {
    ch0_loc = rx_obs_pin;
  }

  /*set the radio RX active to channel 0*/
  PRS_SourceAsyncSignalSet(RX_OBS_PRS_CHANNEL,
      PRS_RAC_RX,
      ((PRS_RAC_RX & _PRS_CH_CTRL_SIGSEL_MASK)>> _PRS_CH_CTRL_SIGSEL_SHIFT));
  /* set the location for channel0 and enable it*/
  PRS_GpioOutputLocation(RX_OBS_PRS_CHANNEL,/*ch0_loc*/rx_obs_pin);

  /*set radio Tx active to channel 1.
   * channel 1. Locations can be 0 - 7*/
  PRS_SourceAsyncSignalSet(TX_OBS_PRS_CHANNEL,
      (PRS_RAC_RX & _PRS_CH_CTRL_SOURCESEL_MASK),
      ((PRS_RAC_RX & _PRS_CH_CTRL_SIGSEL_MASK)>> _PRS_CH_CTRL_SIGSEL_SHIFT));

  /*for series 1 PRS channel 1 can only be output on port F so the pin number minus 1 equates to the location*/
  PRS_GpioOutputLocation(TX_OBS_PRS_CHANNEL,tx_obs_pin-1);
#endif
}
/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();
  initGpio();
  initCmu();
  initTimer();
  initPDM();

  // Initialize DMA only after buffer is populated
  populateBuffer();
  DMADRV_Init();

  /* Initialize stack */
  gecko_init(pconfig);

  while (1)
  {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending())
    {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header))
    {
    /* This boot event is generated when the system boots up after reset.
     * Do not call any stack commands before receiving the boot event.
     * Here the system is set to start advertising immediately after boot procedure. */
    case gecko_evt_system_boot_id:

      bootMessage(&(evt->data.evt_system_boot));
      printLog("boot event - starting advertising\r\n");
      //enableDebugGpios(gpioPortB, 0, gpioPortB, 0);
      /* Set advertising parameters. 100ms advertisement interval.
       * The first parameter is advertising set handle
       * The next two parameters are minimum and maximum advertising interval, both in
       * units of (milliseconds * 1.6).
       * The last two parameters are duration and maxevents left as default. */

      uint32_t e1 = DMADRV_AllocateChannel(&channel, NULL);
      printLog("DMADRV channel %d, retcode: %lu\r\n", channel, e1);
      uint32_t e2 = DMADRV_AllocateChannel(&channelPDM, NULL);
      printLog("DMADRV channel %d, retcode: %lu\r\n", channelPDM, e2);

      startDMADRV();
      startDMADRV_PDM();
      gecko_cmd_le_gap_set_advertise_timing(0, 10 * 160, 10 * 160, 0, 0);

      /* Start general advertising and enable connections. */
      gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
          le_gap_connectable_scannable);
      break;

    case gecko_evt_le_connection_opened_id:

      printLog("connection opened\r\n");
      GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 1);

      break;

    case gecko_evt_le_connection_closed_id:

      printLog("connection closed, reason: 0x%2.2x\r\n",
          evt->data.evt_le_connection_closed.reason);
      GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 0);

      /* Check if need to boot to OTA DFU mode */
      if (boot_to_dfu)
      {
        /* Enter to OTA DFU mode */
        gecko_cmd_system_reset(2);
      }
      else
      {
        /* Restart advertising after client has disconnected */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
            le_gap_connectable_scannable);
      }
      break;

      /* Events related to OTA upgrading
       ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
    case gecko_evt_gatt_server_user_write_request_id:

      if (evt->data.evt_gatt_server_user_write_request.characteristic
          == gattdb_ota_control)
      {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control, bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(
            evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;

      /* Add additional event handlers as your application requires */

    default:
      break;
    }
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor,
      bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++)
  {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}
