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

static volatile float dutyCycle = 0;
volatile bool on_off = true;

void button0Callback(uint8_t pin)
{
	on_off = !on_off;
	TIMER_Enable(TIMER0, on_off);
}

/**************************************************************************//**
 * @brief
 *    Interrupt handler for TIMER0 that changes the duty cycle
 *
 * @note
 *    This handler doesn't actually dynamically change the duty cycle. Instead,
 *    it acts as a template for doing so. Simply change the dutyCycle
 *    global variable here to dynamically change the duty cycle.
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER0);
  static int count=0;
  TIMER_IntClear(TIMER0, flags);
  if ((dutyCycle<TARGET_DUTY_CYCLE)&&(on_off)) {
	  dutyCycle += DUTY_CYCLE_STEPS;
  }
  if ((dutyCycle>0)&&(!on_off)) {
	  dutyCycle -= DUTY_CYCLE_STEPS;
  }
  if (on_off) {
	  count++;
	  if (count==pulse_width) {
		  on_off = false;
		  count=0;
	  }
  }
  // Update CCVB to alter duty cycle starting next period
  TIMER_CompareBufSet(TIMER0, 0, (uint32_t)(topValue * dutyCycle));
}

void initGPIO(void)
{
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
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

void initTimer(void)
{
  uint32_t timerFreq = 0;
  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER0 Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  // Use PWM mode, which sets output on overflow and clears on compare events
  timerInit.prescale = timerPrescale64;
  timerInit.enable = false;
  timerCCInit.mode = timerCCModePWM;

  // Configure but do not start the timer
  TIMER_Init(TIMER0, &timerInit);

  // Route Timer0 CC0 output to PA6
  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
  								  | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  // Configure CC Channel 0
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  // Start with 10% duty cycle
  dutyCycle = DUTY_CYCLE_STEPS;

  // set PWM period
  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
  topValue = (timerFreq / PWM_FREQ);
  // Set top value to overflow at the desired PWM_FREQ frequency
  TIMER_TopSet(TIMER0, topValue);

  // Set compare value for initial duty cycle
  TIMER_CompareSet(TIMER0, 0, (uint32_t)(topValue * dutyCycle));

  // Start the timer
  TIMER_Enable(TIMER0, true);

  // Enable TIMER0 compare event interrupts to update the duty cycle
  TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
  NVIC_EnableIRQ(TIMER0_IRQn);
}

int main(void)
{
  CHIP_Init();
  initCMU();
  initGPIO();
  initTimer();

  while (1)
  {
    EMU_EnterEM1(); // Enter EM1 (won't exit)
  }
}

