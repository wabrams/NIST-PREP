#include <stdio.h>
#include <string.h>

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

#include "retargetserial.h"

#define DUTY_CYCLE_STEPS  0.10
#define TARGET_DUTY_CYCLE 0.50

#define LDMA_PDM_CHANNEL             0
#define LDMA_TIMER_COMP_CHANNEL      1
#define LDMA_TIMER_TOPB_CHANNEL      2

#define BUFFER_SIZE                 2048
#define PP_BUFFER_SIZE               256
#define TIMER_BUFFER_SIZE			       256

int freq_start = 20000;
int freq_stop = 30000;

int16_t left[BUFFER_SIZE];
int16_t right[BUFFER_SIZE];

uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];
bool prevBufferPing;

uint16_t list_pwm[TIMER_BUFFER_SIZE];
uint16_t list_top[TIMER_BUFFER_SIZE];

float pulse_width = 5e-3; //in sec
float dutyCycle = 0;
int numWaves;
uint32_t top_start, top_stop, top_value;
uint32_t offset;
float top_step;
uint32_t timerFreq;
bool play_chirp = false;

void calculateChirpLists()
{
  //EXTRA: clear lists, not totally necessary
  memset(list_pwm, 0, TIMER_BUFFER_SIZE*sizeof(uint16_t));
  memset(list_top, 0, TIMER_BUFFER_SIZE*sizeof(uint16_t));
  // populate list using globals
  for (int i = 0; i < numWaves; i++)
  {
    list_top[i] = top_start + (int)(i * top_step);
    list_pwm[i] = list_top[i] * dutyCycle; //TODO: increment or decrement duty cycle based on i
  }
}

void calculateChirp()
{
  dutyCycle = TARGET_DUTY_CYCLE; //TODO: reintroduce the tapered ends with duty cycle
  top_start = timerFreq / freq_start; // period of start frequency
  top_stop  = timerFreq / freq_stop;  // period of stop frequency
  numWaves = (int)(pulse_width * (freq_stop + freq_start) / 2.0) % TIMER_BUFFER_SIZE; //note the modulo to keep us in bounds of the array
  top_step = ((float) (freq_start - freq_stop)) / ((float) (freq_start * freq_stop)) * timerFreq / (numWaves - 1);
}

void setupChirp(int prescale)
{
  // update the variables and lists
	timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (prescale + 1);
	calculateChirp();
	calculateChirpLists();
  // set first values (to be repeated)
	TIMER_TopBufSet(TIMER0, top_start);
  TIMER_CompareBufSet(TIMER0, 1, (uint32_t) (top_start * DUTY_CYCLE_STEPS));
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
	GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT)  | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT);
	GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
	GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
	//timer
	GPIO->TIMERROUTE[0].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
	GPIO->TIMERROUTE[0].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)	| (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
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

	setupChirp(timerPrescale1);
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

void initLDMA_PDM(void)
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

void initLDMA_TIMER(void)
{
  static LDMA_Descriptor_t timerTOPBLink;
  static LDMA_Descriptor_t timerCOMPLink;
  // TIMER COMP:
    LDMA_TransferCfg_t transferCOMPConfig = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER0_CC1);
    timerCOMPLink = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&list_pwm, &(TIMER0 -> CC[1].OCB), numWaves);
    timerCOMPLink.xfer.size = ldmaCtrlSizeHalf;
    timerCOMPLink.xfer.doneIfs = true;
    LDMA_StartTransfer(LDMA_TIMER_COMP_CHANNEL, &transferCOMPConfig, &timerCOMPLink);
  // TIMER TOPB:
    LDMA_TransferCfg_t transferTOPBConfig = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER0_UFOF);
    timerTOPBLink = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&list_top, &(TIMER0 -> TOPB), numWaves);
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
	  initLDMA_TIMER();
	}
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
	initLDMA_PDM();
	initLDMA_TIMER();
}

static void listen(bool snd)
{
	offset = 0;
	play_chirp = snd;

	while (offset < BUFFER_SIZE)
	{
    EMU_EnterEM1();
	}
}

void binary_dump(uint8_t * buffer, int length)
{
	uint8_t * temp = buffer;
	for (int i = 0; i < length; i++) {
		RETARGET_WriteChar(*temp++);
	}
}

static void printData(void)
{
	printf("left: %d\r\n", BUFFER_SIZE);
	binary_dump((uint8_t*)left, BUFFER_SIZE<<1);
	printf("\r\n");
	RETARGET_SerialFlush();

	printf("right: %d\r\n", BUFFER_SIZE);
	binary_dump((uint8_t*)right, BUFFER_SIZE<<1);
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

int main(void)
{
	RETARGET_SerialCrLf(0);
  printf("Hello World wait\r\n");
  RETARGET_SerialFlush();

  initialize();

	char c = 'i';
	do
	{
		switch (c)
		{
      case 'a': // all (chirp + listen, transmit)
        listen(true);
        // play_chirp = true;
        printData();
        break;
      case 'c': // chirp
        play_chirp = true;
        // setupChirp(freq_start,  freq_stop, timerPrescale1);
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
        printf("top_values: %d\r\n", TIMER_BUFFER_SIZE*sizeof(int));
        binary_dump((uint8_t*)list_pwm, TIMER_BUFFER_SIZE*sizeof(uint32_t));
        printf("\r\n");
        break;
      case '0': // pulse_duration
        read_msg((uint8_t *)&pulse_width, 4);
        printf("chirp duration[ms]: %d\r\n", (int)(pulse_width*1000));
        setupChirp(timerPrescale1);
        printf("ack\r\n");
        RETARGET_SerialFlush();
        break;
      case '1': // start frequency
        read_msg((uint8_t*)&freq_start, 2);
        printf("start: %d\r\n", freq_start);
        setupChirp(timerPrescale1);
        printf("ack\r\n");
        RETARGET_SerialFlush();
        break;
      case '2': // stop frequency
        read_msg((uint8_t *)&freq_stop, 2);
        printf("stop: %d\r\n", freq_stop);
        setupChirp(timerPrescale1);
        printf("ack\r\n");
        RETARGET_SerialFlush();
        break;
		}
		c = RETARGET_ReadChar();
	} while (c != 'q');
}
