#ifndef SPP_UTILS_H
#define SPP_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bg_types.h"
#include "app.h"


/*
 * Optional UART RX buffer overflow checking. This can be added into retargetserial.c
 * by creating a global variable (type int) there with name 'rxOverFlow' and incrementing
 * that variable each time the RX FIFO (the large FIFO in RAM) overflows
 *
 * If you are using the default retargetserial.c implementation without overflow tracking
 * then make sure RX_OVERFLOW_TRACKING is NOT defined
 * */
//#define RX_OVERFLOW_TRACKING
#ifdef RX_OVERFLOW_TRACKING
extern volatile int rxOverFlow;
#endif

extern uint8_t sharedCount;

void spp_main(void);

/* Prototypes of SPP main functions (client / server role) */
void spp_client_main(void);
void spp_server_main(void);

#ifdef __cplusplus
}
#endif

#endif // SPP_UTILS_H
