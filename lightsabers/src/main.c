#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"

#define TIMER_PRESCALE  timerPrescale2
#define TIMER_CHANNEL 		 		 1
#define TONE_FREQ       		  3500

uint32_t max_frequency;
uint32_t pwm_duty;

uint32_t top_value(uint32_t frequency)
{
  if (frequency == 0) return 0;

  return (100*max_frequency/frequency);
}

uint32_t sine_wave_generator()
{
  const uint8_t lookup_table[10] = {5,8,10,10,8,5,2,0,0,2};
  static uint8_t count = 0;

  // Lookup the value
  uint32_t result = lookup_table[count];

  // Adjust count for next time and correct for overflow
  count++; if (count >= 10) count = 0;

  return result * top_value(TONE_FREQ) / 100;
}

void initCMU()
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Calculate the max frequency supported by this program
	uint32_t timer_freq = CMU_ClockFreqGet(cmuClock_TIMER3);
	max_frequency = (timer_freq / (1 << TIMER_PRESCALE)) / 1000;
}

void initGPIO()
{
	// setup speaker (GPIO PD3 = EXP15)
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
	// setup debug (GPIO PD2 = LED0 = EXP11)
	GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 0);
}

void initTIMER()
{
    TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
		timer0Init.prescale = TIMER_PRESCALE;
    TIMER_Init(TIMER0, &timer0Init);
    TIMER0 -> TOP = top_value(TONE_FREQ);
    TIMER0 -> IEN = TIMER_IEN_OF;
    NVIC_EnableIRQ(TIMER0_IRQn);

    TIMER_Init_TypeDef timer1Init = TIMER_INIT_DEFAULT;
    	timer1Init.prescale = TIMER_PRESCALE;
    	timer1Init.enable = false;
    TIMER_InitCC_TypeDef timer1CCInit = TIMER_INITCC_DEFAULT;
    	timer1CCInit.mode = timerCCModeCompare;
    	timer1CCInit.cmoa = timerOutputActionToggle;
    TIMER_Init(TIMER1, &timer1Init);
    GPIO -> TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN;
    GPIO -> TIMERROUTE[1].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
  		  						     | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
    TIMER_InitCC(TIMER1, 0, &timer1CCInit);

    uint32_t freq = 1000;
    uint32_t timerFreq = CMU_ClockFreqGet(cmuClock_TIMER1)/(timer1Init.prescale + 1);
    int topValue = timerFreq / (2*freq) - 1;
    TIMER_TopSet(TIMER1, topValue);

    TIMER_Enable(TIMER0, true);
    TIMER_Enable(TIMER1, true);
}

int main(void)
{
	CHIP_Init();

	initCMU();
	initGPIO();
	initTIMER();

	__enable_irq();

	while (1)
	{
		EMU_EnterEM1();
	}
}

void TIMER0_IRQHandler(void)
{
	__disable_irq();

    // Clear the interrupt
	TIMER0 -> IF_CLR = TIMER0 -> IF;

    // This is for debug viewing on a scope
    GPIO_PinOutToggle(gpioPortB, 0);

    // Get the next sine value
    pwm_duty = sine_wave_generator();

    // Set the new duty cycle for this sample based on the generator
//    TIMER_CompareBufSet(TIMER1, 0, pwm_duty);

	__enable_irq();
}
