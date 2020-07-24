#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"

/*
 * TIMER0 is the heartbeat
 * TIMER0 outputs onto CC0
 *
 * TIMER1 is the squarewave burst
 * TIMER1 outputs onto CC1, listens from CC0
 */

uint32_t square_freq = 2000;
uint32_t chirp_freq = 50;
uint32_t max_freq;

void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
}

void initGPIO(void)
{
  // Note: [<X>].CC<Y> : X is TIMER#, Y is CH#
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);

  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  GPIO->TIMERROUTE[1].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN;
  GPIO->TIMERROUTE[1].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[1].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
}
// timer0 uses ch1, timer1 uses ch0
void initTIMER(void)
{
  // Initialize TIMER0
  TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
	  timer0Init.prescale = timerPrescale1;
	  timer0Init.enable   = false;
	  timer0Init.debugRun = false;
	  //TODO: we want to turn the timer off on falling edges...
//	  timer0Init.fallAction = timerInputActionStop;
  TIMER_Init(TIMER0, &timer0Init);

  TIMER_InitCC_TypeDef timer0CC0Init = TIMER_INITCC_DEFAULT;
	  timer0CC0Init.mode = timerCCModeCompare;
	  timer0CC0Init.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER0, 0, &timer0CC0Init);

  max_freq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timer0Init.prescale + 1);
  int topValue = max_freq / (2*chirp_freq);
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

  max_freq = CMU_ClockFreqGet(cmuClock_TIMER1) / (timer1Init.prescale + 1);
  topValue = max_freq / (2*square_freq);
  TIMER_TopSet(TIMER1, topValue);
}

int main(void)
{
  CHIP_Init();
  // Initialize Hardware
  initCMU();
  initGPIO();
  initTIMER();
  // Start Timers
  TIMER_Enable(TIMER0, true);
  TIMER_Enable(TIMER1, true);
  // Infinite Loop
  while (1)
  {
	  EMU_EnterEM1();
  }
}
