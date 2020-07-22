#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"

#define OUT_FREQ 1000				/**< Output Frequency (Hz) **/
#define REFERENCE_INVERT 1			/**< Compiler Macro, based on speaker configuration **/

void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
}

void initGPIO(void)
{
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
  // [<X>].CC<Y> : X is TIMER#, Y is CH#
  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  #if REFERENCE_INVERT
    GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
	GPIO -> TIMERROUTE[0].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
	GPIO -> TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN;
  #else
	GPIO -> TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  #endif
}

void initTIMER(void)
{
  // Initialize TIMER0
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	  timerInit.prescale = timerPrescale1;
	  timerInit.enable = false;
	  timerInit.debugRun = false;
  TIMER_Init(TIMER0, &timerInit);
  // Initialize TIMER0 Compare / Capture
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	  timerCCInit.mode = timerCCModeCompare;
	  timerCCInit.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER0, 0, &timerCCInit);
  // Initialize TIMER0 Compare / Capture Invert
  #if REFERENCE_INVERT
  	timerCCInit.outInvert = true;
    TIMER_InitCC(TIMER0, 1, &timerCCInit);
  #endif
  // Initialize Top Value, note each compare event is 1/2 the signal period
  uint32_t timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
  int topValue = timerFreq / (2*OUT_FREQ) - 1;
  TIMER_TopSet (TIMER0, topValue);
}

int main(void)
{
  CHIP_Init();
  // Initialize Hardware
  initCMU();
  initGPIO();
  initTIMER();
  // Start Timer
  TIMER_Enable(TIMER0, true);
  // Infinite Loop
  while (1)
  {
	  EMU_EnterEM1();
  }
}
