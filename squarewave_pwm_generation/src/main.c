#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"

#define BSP_BUTTON0_PIN                      (1U)
#define BSP_BUTTON0_PORT                     (gpioPortB)
#include "gpiointerrupt.h"

#define PWM_FREQ          20000
#define DUTY_CYCLE_STEPS  0.05
#define TARGET_DUTY_CYCLE	0.55

static uint32_t topValue = 0;
volatile int pulse_width= PWM_FREQ / 400;  // this sets the pulse width to 2.5mS (1/400)

typedef enum squarechirp_mode_e
{
  chirpInc,
  chirpOn,
  chirpDec,
  chirpOff
} squarechirp_mode_t;

static volatile squarechirp_mode_t chirp_mode = chirpOff;
static volatile uint32_t chirp_cnt = 0;
static volatile float dutyCycle = 0;

void button0Callback(uint8_t pin)
{
  chirp_mode = chirpInc;
	GPIO_PinOutToggle(gpioPortD, 2);
}

void TIMER0_IRQHandler(void)
{
  uint32_t flags = TIMER0 -> IF_CLR = TIMER0 -> IF;
  // Acknowledge the interrupt
  if (flags & TIMER_IF_CC1)
  {
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
        chirp_cnt++;
        if (chirp_cnt >= pulse_width)
            chirp_mode = chirpDec;
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
        chirp_cnt = 0;

        //TODO: disable timer here
        break;
    }
  }
}

void initGPIO(void)
{
  //gpio
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
  //timer
  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN;
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[0].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
  //button
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPullFilter, 1);
  GPIOINT_Init();
  GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, button0Callback);
  GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, false, true, true);
}

void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
}

void initTIMER(void)
{

  TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
    timer0Init.prescale = timerPrescale64;
    timer0Init.enable = false;
    timer0Init.debugRun = false;
    timer0Init.riseAction = timerInputActionReloadStart;
  TIMER_Init(TIMER0, &timer0Init);

  TIMER_InitCC_TypeDef timer0CC0Init = TIMER_INITCC_DEFAULT;
    timer0CC0Init.mode = timerCCModeCapture;
  TIMER_InitCC(TIMER0, 0, &timer0CC0Init);

  TIMER_InitCC_TypeDef timer0CC1Init = TIMER_INITCC_DEFAULT;
    timer0CC1Init.mode = timerCCModePWM;
  TIMER_InitCC(TIMER0, 1, &timer0CC1Init);

  // Start with 10% duty cycle
  dutyCycle = DUTY_CYCLE_STEPS;

  uint32_t timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timer0Init.prescale + 1);
  topValue = (timerFreq / PWM_FREQ);
  TIMER_TopSet(TIMER0, topValue);
  TIMER_CompareSet(TIMER0, 1, (uint32_t)(topValue * dutyCycle));

  TIMER_IntEnable(TIMER0, TIMER_IEN_CC1);
  NVIC_EnableIRQ(TIMER0_IRQn);

  TIMER_Enable(TIMER0, true);
}

int main(void)
{
  CHIP_Init();
  initCMU();
  initGPIO();
  initTIMER();

  while (1)
  {
    EMU_EnterEM1(); // Enter EM1 (won't exit)
  }
}

