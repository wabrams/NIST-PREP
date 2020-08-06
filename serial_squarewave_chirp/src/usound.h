#ifndef _USOUND_H_
#define _USOUND_H_

#define DUTY_CYCLE_STEPS  0.10
#define TARGET_DUTY_CYCLE 0.50
#define TIMER_PDM           TIMER0
#define TIMER_PDM_CLK       cmuClock_TIMER0
#define TIMER_PDM_PRESCALE  timerPrescale1
#define TIMER_PDM_LDMA_COMP ldmaPeripheralSignal_TIMER0_CC1
#define TIMER_PDM_LDMA_TOPV ldmaPeripheralSignal_TIMER0_UFOF

#define GPIO_PRS_CHANNEL    1

#define LDMA_PDM_CHANNEL             0
#define LDMA_TIMER_COMP_CHANNEL      1
#define LDMA_TIMER_TOPB_CHANNEL      2

#define BUFFER_SIZE                 2048
#define PP_BUFFER_SIZE               128

void setupChirp();
void initTIMER(void);
void initPDM(void);
void initLDMA(void);
void initUSound(void);
void startLDMA_PDM(void);
void startLDMA_TIMER(void);
void listen();
void init_usound();
void LDMA_IRQHandler(void);

void binary_dump(uint8_t * buffer, int length);
void printData(void);
void printCorr(void);
void read_msg(uint8_t * buf, int length);
void read_ser(char c);

#endif /* _USOUND_H_ */
