#include <stdio.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_rtcc.h"
#include "em_prs.h"

#include "retargetserial.h"
#include "simple_dsp.h"
#include "usound.h"

void initCMU(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  CMU_ClockEnable(cmuClock_PDM, true);
  CMU_ClockSelectSet(cmuClock_PDM, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockSelectSet(cmuClock_TIMER0, cmuSelect_HFRCODPLL);
  CMU_ClockEnable(cmuClock_RTCC, true);
  CMU_ClockSelectSet(cmuClock_RTCC, cmuSelect_LFRCO);
}

void initPRS(void)
{
  // Select GPIO as source and button 0 GPIO pin as signal for PRS channel 0
  PRS_SourceAsyncSignalSet(GPIO_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO, BSP_GPIO_PB0_PIN);
  // Do not apply any logic on the PRS Channel
  PRS_Combine (GPIO_PRS_CHANNEL, GPIO_PRS_CHANNEL, prsLogic_A);
  // Select PRS channel for Timer 0
  PRS_ConnectConsumer(GPIO_PRS_CHANNEL, prsTypeAsync, prsConsumerTIMER0_CC0);
}

void initGPIO(void)
{
  //gpio
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);     // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);     // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput,    0);     // PDM_DATA
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
  //button
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPullFilter, 1);
  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, false, false);
  //pdm
  GPIO_SlewrateSet(gpioPortC, 7, 7);
  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE  = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT)  | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT) | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);
  //timer
  GPIO->TIMERROUTE[0].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[0].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
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

int main(void)
{
  //retarget to serial port
	RETARGET_SerialCrLf(0);
  printf("Hello World wait\r\n");
  RETARGET_SerialFlush();

  //initialize heavies
  initCMU();
  initPRS();
  initRTCC();
  initGPIO();
  //initialize usound
  init_usound();

  //waiting for serial
	char c = 'i';
	do
	{
	  read_ser(c);
		c = RETARGET_ReadChar();
	} while (c != 'q');
}
