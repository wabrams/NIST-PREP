/*
 * usound.c
 *
 *  Created on: Aug 5, 2020
 *      Author: Will
 */
#include <stdio.h>
#include <string.h>

#include "em_chip.h"
#include "em_emu.h"
#include "em_ldma.h"
#include "em_pdm.h"
#include "em_device.h"
#include "em_timer.h"

#include "retargetserial.h"
#include "simple_dsp.h"
#include "usound.h"


int freq_start = 20000;                                 /**< Starting Frequency for Square Chirp, in Hz **/
int freq_stop = 30000;                                  /**< Stopping Frequency for Square Chirp, in Hz **/

int16_t left[BUFFER_SIZE];                              /**< Left Microphone's PDM Array  **/
int16_t right[BUFFER_SIZE];                             /**< Right Microphone's PDM Array **/
uint32_t offset;                                        /**< Variable: Keeps track of where to write in the left and right arrays **/

int32_t corr_left[BUFFER_SIZE];                         /**<  **/
int32_t corr_right[BUFFER_SIZE];                        /**<  **/
uint8_t pdm_template[512];                              /**<  **/
int N_pdm_template;                                     /**<  **/

uint32_t pingBuffer[PP_BUFFER_SIZE];                    /**< Ping-Buffer for PDM, written to by LDMA **/
uint32_t pongBuffer[PP_BUFFER_SIZE];                    /**< Pong-Buffer for PDM, written to by LDMA **/
bool prevBufferPing;                                    /**< Ping-Pong state variable **/

uint16_t list_pwm[TIMER_BUFFER_SIZE];                   /**< List of COMP values for the TIMER, for PWM **/
uint16_t list_top[TIMER_BUFFER_SIZE];                   /**< List of TOP values for the TIMER, for FREQ **/

float pulse_width = 5e-3;                               /**< Input: Square Chirp Duration, in seconds **/
float dutyCycle = 0;                                    /**<  **/
int numWaves;                                           /**< Calculated: Number of waves in the Square Chirp **/
float top_step;                                         /**< Calculated: Top Value increment between waves   **/
uint32_t top_start;                                     /**< Calculated: Top Value for TIMER, for freq_start **/
uint32_t top_stop;                                      /**< Calculated: Top Value for TIMER, for freq_stop  **/
uint32_t top_value;                                     /**<  **/
uint32_t timerFreq;                                     /**< Constant: Calculating using TIMER0's maximum frequency and prescale value **/
bool play_chirp = false;                                /**<  **/

void setupChirp()
{
  dutyCycle = TARGET_DUTY_CYCLE;
  top_start = timerFreq / freq_start; // period of start frequency
  top_stop = timerFreq / freq_stop;  // period of stop frequency
  // update the variables and lists
  memset(list_pwm, 0, TIMER_BUFFER_SIZE * sizeof(uint16_t));
  memset(list_top, 0, TIMER_BUFFER_SIZE * sizeof(uint16_t));
  calculate_periods_list(freq_start, freq_stop, pulse_width, list_top, &numWaves);
  printf("top_start: %lu, top_stop %lu, numWaves: %d\r\n", top_start, top_stop, numWaves);
  for (int i = 0; i < numWaves; i++)
  {
    list_pwm[i] = list_top[i] * dutyCycle;
  }
  if (numWaves > 20)
  {
    float prev = 0;
    for (int i = 0; i < 10; i++)
    {
      prev += 0.05;
      list_pwm[i] = prev * list_top[i];
      list_pwm[numWaves - 1 - i] = prev * list_top[numWaves-1-i];
    }
  }
  else if (numWaves > 10)
  {
    float prev = 0;
    for (int i = 0; i < 4; i++)
    {
      prev += 0.1;
      list_pwm[i] = prev * list_top[i];
      list_pwm[numWaves - 1 - i] = prev * list_top[numWaves-1-i];
    }
  }
  // set first values (to be repeated)
//  TIMER_TopBufSet(TIMER0, top_start);
//  TIMER_CompareBufSet(TIMER0, 1, (uint32_t) (top_start * DUTY_CYCLE_STEPS));
}

void initTIMER(void)
{
  TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
  timer0Init.prescale = TIMER0_PRESCALE;
  timer0Init.enable = false;
  timer0Init.debugRun = false;
  timer0Init.fallAction = timerInputActionReloadStart;
  TIMER_Init(TIMER0, &timer0Init);

  TIMER_InitCC_TypeDef timer0CC0Init = TIMER_INITCC_DEFAULT;
    timer0CC0Init.edge = timerEdgeFalling;
    timer0CC0Init.mode = timerCCModeCapture;
    timer0CC0Init.prsSel = GPIO_PRS_CHANNEL;
    timer0CC0Init.prsInput = true;
    timer0CC0Init.prsInputType = timerPrsInputAsyncLevel;
  TIMER_InitCC(TIMER0, 0, &timer0CC0Init);

  TIMER_InitCC_TypeDef timer0CC1Init = TIMER_INITCC_DEFAULT;
  timer0CC1Init.mode = timerCCModePWM;
  TIMER_InitCC(TIMER0, 1, &timer0CC1Init);

  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (TIMER0_PRESCALE + 1);
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

void initLDMA(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;
  LDMA_Init(&init);
}

void startLDMA_PDM(void)
{
  static LDMA_Descriptor_t descLink[2];

  // PDM FIFO:
  LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_PDM_RXDATAV);
  // Link descriptors for ping-pong transfer
  descLink[0] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pingBuffer, PP_BUFFER_SIZE, +1);
  descLink[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pongBuffer, PP_BUFFER_SIZE, -1);
  // Next transfer writes to pingBuffer
  prevBufferPing = false;
  LDMA_StartTransfer(LDMA_PDM_CHANNEL, (void*) &periTransferTx, (void*) &descLink);
}

void startLDMA_TIMER(void)
{
  static LDMA_Descriptor_t timerTOPBLink;
  static LDMA_Descriptor_t timerCOMPLink;

  // TIMER COMP:
  LDMA_TransferCfg_t transferCOMPConfig = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER0_CC1);
  timerCOMPLink = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&list_pwm, &(TIMER0 -> CC[1].OCB), numWaves + 1);
  timerCOMPLink.xfer.size = ldmaCtrlSizeHalf;
  timerCOMPLink.xfer.doneIfs = true;
  LDMA_StartTransfer(LDMA_TIMER_COMP_CHANNEL, &transferCOMPConfig, &timerCOMPLink);
  // TIMER TOPB:
  LDMA_TransferCfg_t transferTOPBConfig = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER0_UFOF);
  timerTOPBLink = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&list_top, &(TIMER0 -> TOPB), numWaves + 1);
  timerTOPBLink.xfer.size = ldmaCtrlSizeHalf;
  timerTOPBLink.xfer.doneIfs = true;
  LDMA_StartTransfer(LDMA_TIMER_TOPB_CHANNEL, &transferTOPBConfig, &timerTOPBLink);
}

static inline void listenPass()
{
  if (play_chirp)
  {
    TIMER_Enable(TIMER0, true);
    play_chirp=false;
  }
  prevBufferPing = !prevBufferPing;
  if (offset < BUFFER_SIZE)
  {
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

void listen(bool snd)
{
  offset = 0;
  play_chirp = snd;

  while (offset < BUFFER_SIZE)
  {
    EMU_EnterEM1();
  }
}

void LDMA_IRQHandler(void)
{
  uint32_t pending = LDMA -> IF_CLR = LDMA -> IF;
  EFM_ASSERT(!(pending & LDMA_IF_ERROR));

  if (pending & LDMA_CHDONE_CHDONE0)
  {
    listenPass();
  }
  if (pending & LDMA_CHDONE_CHDONE1)
  {
    LDMA_StopTransfer(1);
  }
  if (pending & LDMA_CHDONE_CHDONE2)
  {
    LDMA_StopTransfer(2);
    TIMER_Enable(TIMER0, false);
    startLDMA_TIMER();
  }
}

void init_usound()
{
  // Chip errata
  CHIP_Init();
  // Initialize Peripherals
  initTIMER();
  initPDM();
  initLDMA();
  startLDMA_PDM();
  startLDMA_TIMER();
  setupChirp();
}

void binary_dump(uint8_t * buffer, int length)
{
  uint8_t * temp = buffer;
  for (int i = 0; i < length; i++) {
    RETARGET_WriteChar(*temp++);
  }
}

void printData(void)
{
  printf("left: %d\r\n", BUFFER_SIZE);
  binary_dump((uint8_t*)left, BUFFER_SIZE << 1);
  printf("\r\n");
  RETARGET_SerialFlush();

  printf("right: %d\r\n", BUFFER_SIZE);
  binary_dump((uint8_t*)right, BUFFER_SIZE << 1);
  printf("\r\n");
  RETARGET_SerialFlush();
}

void printCorr(void)
{
  printf("leftcorr: %d\r\n", BUFFER_SIZE);
  binary_dump((uint8_t*) corr_left, BUFFER_SIZE << 2);
  printf("\r\n");
  RETARGET_SerialFlush();

  printf("rightcorr: %d\r\n", BUFFER_SIZE);
  binary_dump((uint8_t*) corr_right, BUFFER_SIZE << 2);
  printf("\r\n");
  RETARGET_SerialFlush();
}

void read_msg(uint8_t * buf, int length)
{
  uint8_t *temp = buf;
  int c;
  while (length > 0)
  {
    while ((c = RETARGET_ReadChar()) < 0){;}
    *temp++ = (uint8_t) c;
    length--;
  }
}

void read_ser(char c)
{
  switch (c)
  {
    case 'a': // all (chirp + listen, transmit)
      listen(true);
      // play_chirp = true;
      printData();
      break;
    case 'b': // all (chirp + listen, transmit)
      listen(true);
      printData();
      calc_chirp(freq_start, freq_stop, pulse_width, pdm_template, &N_pdm_template);
      calc_cross(left, BUFFER_SIZE, pdm_template, N_pdm_template, corr_left);
      calc_cross(right, BUFFER_SIZE, pdm_template, N_pdm_template, corr_right);
      printCorr();
      break;
    case 'c': // chirp
      play_chirp = true;
      // TIMER_Enable(TIMER0, true);
      break;
    case 'i':
      break;
    case 'l': // listen
      listen(false);
      break;
    case 'r': // record (chirp + listen)
      listen(true);
      // play_chirp = true;
      break;
    case 's': // sample numbers
      printf("samp\r\n%d\r\n", BUFFER_SIZE);
      RETARGET_SerialFlush();
      break;
    case 't': // transmit
      printData();
      break;
    case 'p': // get list of periods
      printf("top_values: %d\r\n", TIMER_BUFFER_SIZE * sizeof(uint16_t));
      binary_dump((uint8_t*) list_top,
          TIMER_BUFFER_SIZE * sizeof(uint16_t));
      printf("\r\n");
      break;
    case 'w': // get list of pwm
      printf("pwm_values: %d\r\n", TIMER_BUFFER_SIZE * sizeof(uint16_t));
      binary_dump((uint8_t*) list_pwm,
          TIMER_BUFFER_SIZE * sizeof(uint16_t));
      printf("\r\n");
      break;
    case '0': // pulse_duration
      read_msg((uint8_t *) &pulse_width, 4);
      printf("chirp duration[ms]: %d\r\n", (int) (pulse_width * 1000));
      setupChirp();
      printf("ack\r\n");
      RETARGET_SerialFlush();
      break;
    case '1': // start frequency
      read_msg((uint8_t*) &freq_start, 2);
      printf("start: %d\r\n", freq_start);
      setupChirp();
      printf("ack\r\n");
      RETARGET_SerialFlush();
      break;
    case '2': // stop frequency
      read_msg((uint8_t *) &freq_stop, 2);
      printf("stop: %d\r\n", freq_stop);
      setupChirp();
      printf("ack\r\n");
      RETARGET_SerialFlush();
      break;
  }
}
