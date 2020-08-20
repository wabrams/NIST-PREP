/***********************************************************************************************//**
 * \file   spp_utils.c
 * \brief  Generic utilities used by the SPP server and client
 *
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

#include "spp_utils.h"
#include <stdio.h>
#include "gatt_db.h"

/* needed for board specific GPIO mappings :*/
#include "hal-config-board.h"

uint8_t sharedCount = 0;

/**
 *  SPP startup code, called from main.c. Start SPP either in server mode
 *  (implemented in spp_server_main.c) or client mode (spp_client_main.c)
 *
 *  Mode is selected based on pushbutton PB0/PB1 status at startup.
 *  - Default behavior (both buttons released) -> start in SERVER mode
 *  - if either PB0 or PB1 pressed during reset -> start in CLIENT mode
 *
 *  This function never returns. It jumps into SPP main loop in either
 *  server or client role and runs there until device is rebooted.
 * */
void spp_main(void)
{
		printLog("* SPP merged mode *\r\n");
		spp_client_main();
}



