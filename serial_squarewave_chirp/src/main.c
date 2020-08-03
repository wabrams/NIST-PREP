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

int freq_start = 20000;
int freq_stop = 30000;

#define DUTY_CYCLE_STEPS  0.1
#define TARGET_DUTY_CYCLE 0.50

#define LDMA_CHANNEL                     0
#define LDMA_CH_MASK    (1 << LDMA_CHANNEL)

#define BUFFER_SIZE               (1 << 11)
#define PP_BUFFER_SIZE                 256
#define PERIOD_BUFFER_SIZE			       256

bool prevBufferPing;
int16_t left[BUFFER_SIZE];
int16_t right[BUFFER_SIZE];
LDMA_Descriptor_t descLink[2];
uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];

float pulse_width = 1e-3; //in sec
int N_waves;
uint32_t top_start, top_stop, top_value;
float top_step;
uint32_t timerFreq;
bool play_chirp = false;
uint32_t list_periods[PERIOD_BUFFER_SIZE];

typedef enum squarechirp_mode_e
{
	chirpInc, chirpOn, chirpDec//, chirpOff
} squarechirp_mode_t;

static volatile squarechirp_mode_t chirp_mode = chirpInc;
static volatile float dutyCycle = 0;

void TIMER0_IRQHandler(void)
{
	uint32_t flags = TIMER0->IF_CLR = TIMER0->IF;
	static int counter = 0; // Keep track of square wave number
	static int ramp_length = 0;
	// Acknowledge the interrupt
	if (flags & TIMER_IF_CC1)
	{
		/*
		top_value = top_start + (int32_t) (counter * top_step);
		TIMER_CompareBufSet(TIMER0, 1, (uint32_t) (top_value * dutyCycle));
		TIMER_TopBufSet(TIMER0, top_value);
		*/
		switch (chirp_mode)
		{
      case chirpInc:
        dutyCycle += DUTY_CYCLE_STEPS;
        ramp_length++;
        if (dutyCycle >= TARGET_DUTY_CYCLE) {
          chirp_mode = chirpOn;
          dutyCycle = TARGET_DUTY_CYCLE;
          ramp_length--;
        }
        break;
      case chirpOn:
        // I think below should be -2... or 0... -3 seems to be corrrect according to my oscilloscope
        if (counter == (N_waves - 3 - ramp_length)) {
          chirp_mode = chirpDec;
          //printf("counter: %d\r\n", counter);
        }
        break;
      case chirpDec:
        dutyCycle -= DUTY_CYCLE_STEPS;
        if (dutyCycle <= 0) {
          // chirp_mode = chirpOff;
          dutyCycle = 0;
          TIMER_Enable(TIMER0, false);
          printf("Chirp off, counter: %d, ramp:%d\r\n", counter, ramp_length);
          counter = -1;  // gets incremented to 0 below
          ramp_length = 0; // reset
          dutyCycle = DUTY_CYCLE_STEPS; // setup for when timer is restarted
          // dutyCycle = 0;
          chirp_mode = chirpInc;
        }
        break;
		}
		// collect all top_values used
		list_periods[counter] = TIMER_TopGet(TIMER0);
		counter++;

		top_value = top_start + (int32_t) (counter * top_step);
		TIMER_CompareBufSet(TIMER0, 1, (uint32_t) (top_value * dutyCycle));
		TIMER_TopBufSet(TIMER0, top_value);

	}
}

void initCMU(void) {
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

void setupChirp(int freq_start, int freq_stop, int prescale)
{
	// Start with 10% duty cycle
	dutyCycle = DUTY_CYCLE_STEPS;

	timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (prescale + 1);
	top_start = timerFreq / freq_start;  // period of start frequency
	top_stop = timerFreq / freq_stop;  // period of stop frequency
	N_waves = pulse_width * (freq_stop+freq_start)/2;
	top_step = ((float) (freq_start - freq_stop)) / ((float) (freq_start * freq_stop)) * timerFreq / (N_waves - 1);

	printf("N_waves: %d. top_start: %lu, top_stop: %lu, top_step x1000: %d\r\n", N_waves, top_start, top_stop, (int) (top_step * 1000));
	top_value = top_start;

	// TIMER_TopSet(TIMER0, top_value);
	// TIMER_CompareSet(TIMER0, 1, (uint32_t) (top_value * dutyCycle));
	// Seems like when the timer is enabled, it generate an immediate interrupt
	//    So, need to load first value into the buffer.
	TIMER_TopBufSet(TIMER0, top_value);
	TIMER_CompareBufSet(TIMER0, 1, (uint32_t) (top_value * dutyCycle));
    // clear out previous list of periods
	printf("Check first stop: %lu\r\n", TIMER_TopGet(TIMER0));

	memset(list_periods, 255, PERIOD_BUFFER_SIZE*sizeof(uint32_t));
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

	setupChirp(freq_start,  freq_stop, timerPrescale1);

	TIMER0->IEN = TIMER_IEN_CC1;
	NVIC_EnableIRQ(TIMER0_IRQn);
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
	// LDMA transfers trigger on PDM RX Data Valid
	LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_PDM_RXDATAV);
	// Link descriptors for ping-pong transfer
	descLink[0] =	(LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pingBuffer, PP_BUFFER_SIZE, +1);
	descLink[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(PDM->RXDATA), pongBuffer, PP_BUFFER_SIZE, -1);
	// Next transfer writes to pingBuffer
	prevBufferPing = false;
	LDMA_Init(&init);
	LDMA_StartTransfer(LDMA_CHANNEL, (void*) &periTransferTx, (void*) &descLink);
}

void LDMA_IRQHandler(void)
{
	uint32_t pending = LDMA_IntGet();
	LDMA_IntClear(pending);
    // static bool wait = true;
	if (pending & LDMA_IF_ERROR)
	{
		while (1); //TODO: assert here
	}
	if (play_chirp)
	{
		TIMER_Enable(TIMER0, true);
		play_chirp=false;
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

static void listen(bool snd)
{
	int offset = 0;
	bool lastPing = prevBufferPing;
	play_chirp=snd;
    // memset(left, 0xFFFF, BUFFER_SIZE<<1);
    // memset(right, 0xFFFF, BUFFER_SIZE<<1);
	while (offset < BUFFER_SIZE)
	{
		while (lastPing == prevBufferPing)
		{
			EMU_EnterEM1();
		}
		lastPing = prevBufferPing;

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

void binary_dump(uint8_t * buffer, int length)
{
	uint8_t* temp = buffer;
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
        printf("top_values: %d\r\n", PERIOD_BUFFER_SIZE*sizeof(int));
        binary_dump((uint8_t*)list_periods, PERIOD_BUFFER_SIZE*sizeof(uint32_t));
        printf("\r\n");
        break;
      case '0': // pulse_duration
        read_msg((uint8_t *)&pulse_width, 4);
        printf("chirp duration[ms]: %d\r\n", (int)(pulse_width*1000));
        setupChirp(freq_start, freq_stop, timerPrescale1);
        printf("ack\r\n");
        RETARGET_SerialFlush();
        break;
      case '1': // start frequency
        read_msg((uint8_t*)&freq_start, 2);
        printf("start: %d\r\n", freq_start);
        setupChirp(freq_start, freq_stop, timerPrescale1);
        printf("ack\r\n");
        RETARGET_SerialFlush();
        break;
      case '2': // stop frequency
        read_msg((uint8_t *)&freq_stop, 2);
        printf("stop: %d\r\n", freq_stop);
        setupChirp(freq_start, freq_stop, timerPrescale1);
        printf("ack\r\n");
        RETARGET_SerialFlush();
        break;
		}
		c = RETARGET_ReadChar();
	} while (c != 'q');
}
