/*
 * speaker.h
 *
 *  Created on: Aug 9, 2020
 *      Author: nams
 */

#include "ultrasound_define.h"

#define DUTY_CYCLE_STEPS  0.10
#define TARGET_DUTY_CYCLE 0.50


#ifndef ULTRASOUND_SPEAKER_H_
#define ULTRASOUND_SPEAKER_H_

#include "bg_types.h"

#include "app.h"
#include "em_cmu.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_ldma.h"
#include "dmadrv.h"

#include "simple_dsp.h"

#include "em_timer.h"
#include "em_rtcc.h"

#include "gpiointerrupt.h"


extern float pulse_width; /**< Input: Square Chirp Duration, in seconds **/
extern float dutyCycle; /**<  **/
extern int numWaves; /**< Calculated: Number of waves in the Square Chirp **/
//extern float top_step;
//extern int freq_start; /**< Starting Frequency for Square Chirp, in Hz **/
//extern int freq_stop; /**< Stopping Frequency for Square Chirp, in Hz **//**< Calculated: Top Value increment between waves   **/
//extern uint32_t top_start; /**< Calculated: Top Value for TIMER, for freq_start **/
//extern uint32_t top_stop; /**< Calculated: Top Value for TIMER, for freq_stop  **/
//extern uint32_t top_value; /**<  **/
extern uint32_t timerFreq; /**< Constant: Calculating using TIMER_PDM's maximum frequency and prescale value **/
extern bool usound_spk; /**<  **/
extern bool usound_lst;

extern uint32_t tick_pdm_start;
extern uint32_t start, stop, durr, measured_ms;

extern uint16_t list_pwm[TIMER_BUFFER_SIZE]; /**< List of COMP values for the TIMER, for PWM **/
extern uint16_t list_top[TIMER_BUFFER_SIZE]; /**< List of TOP values for the TIMER, for FREQ **/

extern unsigned int ldma_channelTMR_TOPV, ldma_channelTMR_COMP;



void populateBuffers(int k);
void init_speaker(void);
void initTIMER(void);
void prs_gpio(GPIO_Port_TypeDef rx_obs_port, uint8_t rx_obs_pin,
		GPIO_Port_TypeDef tx_obs_port, uint8_t tx_obs_pin);
void startDMADRV_TMR(void);
void init_prs(void) ;
void play_speaker(void);

#endif /* ULTRASOUND_SPEAKER_H_ */
