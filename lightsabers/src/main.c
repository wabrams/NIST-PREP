#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"

#define TIMER_PRESCALE  timerPrescale2
#define TONE_FREQ       		  3500

#define PWM_TIMER 				TIMER1
#define PWM_CHANNEL 		 		 1

uint32_t max_frequency;
uint32_t i = 0;

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
//	CMU_ClockEnable(cmuClock_TIMER1, true);

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
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
		timerInit.prescale = TIMER_PRESCALE;
    TIMER_Init(TIMER0, &timerInit);
    TIMER0 -> TOP = top_value(TONE_FREQ);
    TIMER0 -> IEN = TIMER_IEN_OF;

    NVIC_EnableIRQ(TIMER0_IRQn);
}

int main(void)
{
	CHIP_Init();

	initCMU();
	initGPIO();
	initTIMER();

	i = 0;
	while (1)
	{
		i += 1;
//		EMU_EnterEM1();
	}
}

void TIMER0_IRQHandler(void)
{
    // Clear the interrupt
    TIMER_IntClear(TIMER0, TIMER_IF_OF);

    // This is for debug viewing on a scope
    GPIO_PinOutToggle(gpioPortB, 0);

    // Get the next sine value
    uint32_t pwm_duty = sine_wave_generator();

    // Set the new duty cycle for this sample based on the generator
    TIMER_CompareBufSet(PWM_TIMER, PWM_CHANNEL, pwm_duty);
}
