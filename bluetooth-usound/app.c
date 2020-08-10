/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "app.h"
#include "em_cmu.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_ldma.h"
#include "dmadrv.h"

#include "em_timer.h"
#include "em_rtcc.h"

#include "gpiointerrupt.h"

#include "ultrasound/microphone.h"
#include "ultrasound/speaker.h"

static uint8 _conn_handle = 0xFF;

uint8 adv_data[9] = {0x02, 0x01, 0x06, 0x05, 'h', 'e', 'l', 'l', 'o'};
uint8 scan_resp_data[9] = {0x02, 0x01, 0x06, 0x05, 'w', 'o', 'r', 'l', 'd'};

void bin(uint8_t n)
{
  uint8_t i;
  for (i = 1 << 7; i > 0; i = i >> 1)
    (n & i) ? printLog("1") : printLog("0");
}

void print_mac(bd_addr addr)
{
  for (int i = 0; i < 5; i++)
  {
    printLog("%02x:", addr.addr[5 - i]);
  }
  printLog("%02x", addr.addr[0]);
}

void print_data(uint8array * data)
{
  printLog("0x");
  for (int i = 0; i < data->len; i++)
  {
//    printLog("%c", data->data[i]);
    printLog("%02x", data->data[i]);
  }
}

#define ADV_HANDLE 0
void startAdvertising(uint32 interval)
{
  uint16_t ret = 0;
  gecko_cmd_le_gap_set_advertise_timing(ADV_HANDLE, interval, interval, 0, 0);
  // request event when scan request made
  gecko_cmd_le_gap_set_advertise_report_scan_request(ADV_HANDLE, 1);
  ret = gecko_cmd_le_gap_bt5_set_adv_data(ADV_HANDLE, 0, 9, adv_data)->result;
  printLog("set adv msg: %d\r\n", ret);
  ret = gecko_cmd_le_gap_bt5_set_adv_data(ADV_HANDLE, 1, 9, scan_resp_data)->result;
  printLog("set scan response msg: %d\r\n", ret);
  /* Start general advertising and enable connections. */

  /*
   gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
   le_gap_undirected_connectable);
   */
  gecko_cmd_le_gap_start_advertising(ADV_HANDLE, le_gap_user_data, le_gap_scannable_non_connectable);
}

void startObserving(uint16_t interval, uint16_t window)
{
  gecko_cmd_le_gap_end_procedure();
  /*  Start observing
   * Use  extended to get channel info as well
   */
  gecko_cmd_le_gap_set_discovery_extended_scan_response(true);
  /* scan on 1M PHY at 200 ms scan window/interval*/
  gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, interval, window);
  /* passive scanning = 0, no scan response sent, active scanning = 1, request more info */
  gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m, 1);
  /* discover all devices on 1M PHY*/
  gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
}
static int process_scan(struct gecko_msg_le_gap_extended_scan_response_evt_t *pResp)
{
  if (pResp->address.addr[5] == 0x58)
  {
    if (pResp->packet_type & 0x4) //request
    {
// //USEFUL: for printing out BT Packet Info
//      uint8_t channel = pResp->channel;
//      int8_t rssi = pResp->rssi;
//      print_mac(pResp->address);
//      printLog(", type: ");
//      bin(pResp->packet_type);
//      printLog(", ch:%d, rssi:%d, ", channel, rssi);
//      printLog("data len: %2d, data:", pResp->data.len);
//      print_data(&(pResp->data));
//      printLog("\r\n");

      printLog("MICROPHONE\r\n");
      // start microphone
      recording = true;
      startLDMA_PDM();
    }
  }
  /*  TODO: Work in progress end */
  return 0;
}

void init_dma_channels(void)
{
  uint32_t e0 = DMADRV_AllocateChannel(&ldma_channelTMR_TOPV, NULL);
  printLog("DMADRV channel %d, retcode: %lu\r\n", ldma_channelTMR_TOPV, e0);
  uint32_t e1 = DMADRV_AllocateChannel(&ldma_channelTMR_COMP, NULL);
  printLog("DMADRV channel %d, retcode: %lu\r\n", ldma_channelTMR_COMP, e1);
  uint32_t e2 = DMADRV_AllocateChannel(&ldma_channelPDM, NULL);
  printLog("DMADRV channel %d, retcode: %lu\r\n", ldma_channelPDM, e2);
}

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif
  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();
  init_speaker();
  initTIMER();
  // init_prs();
  initPDM();
  initTimer0();
  DMADRV_Init();

  /* Initialize stack */
  gecko_init(pconfig);

  while (1)
  {
    /* Event pointer for handling events */
    struct gecko_cmd_packet * evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending())
    {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt -> header))
    {
      case gecko_evt_system_boot_id:
        gecko_cmd_gatt_set_max_mtu(247);
        bootMessage(&(evt->data.evt_system_boot));
        printLog("boot event - starting advertising\r\n");

        init_dma_channels();
        startDMADRV_TMR();
        prs_gpio(gpioPortB, 0, gpioPortC, 0);

        startAdvertising(160);
        startObserving(3200, 320);
        break;

      case gecko_evt_le_gap_scan_request_id:
      {
        /*
         *  Who made a scan request?   We should now send a chirp
         */
        bd_addr scanner_address = evt->data.evt_le_gap_scan_request.address;
        uint8 address_type = evt->data.evt_le_gap_scan_request.address_type;
        uint8 bonding = evt->data.evt_le_gap_scan_request.bonding;
        /*  Work in progress start */
        //  Need some way to figure out whether this request warrants a speaker chirp
        //  For now just check the mac address... Need to figure out a better way.
        if (scanner_address.addr[5] == 0x58)
        {
          printLog("*************************scan_request: %d, %d, ", address_type, bonding);
          print_mac(scanner_address);
          printLog("\r\n");
          // TODO: play speaker here
          printLog("SPEAKER\r\n");
          TIMER_Enable(TIMER1, true);
        }
        /*  Work in progress end */

      }
        break;
      case gecko_evt_le_gap_extended_scan_response_id:
      {
        /*
         *  Heard an advertiser... what were the parameters of the connection
         */
        // printLog("scan_response_id %d\r\n", evt->data.evt_le_gap_extended_scan_response.data.len);
        process_scan(&(evt->data.evt_le_gap_extended_scan_response));
        break;
      }
      case gecko_evt_le_connection_opened_id:
        _conn_handle = evt->data.evt_le_connection_opened.connection;
        printLog("Connected\r\n");

        /* Request connection parameter update.
         * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
         * supervision timeout 2 seconds
         * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
        gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);

        printLog("connection opened\r\n");
        GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 1);

        break;

      case gecko_evt_le_connection_closed_id:

        printLog("connection closed, reason: 0x%2.2x\r\n",
            evt->data.evt_le_connection_closed.reason);
        GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 0);

        /* Check if need to boot to OTA DFU mode */
        if (boot_to_dfu)
        {
          /* Enter to OTA DFU mode */
          gecko_cmd_system_reset(2);
        }
        else
        {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
              le_gap_connectable_scannable);
        }
        break;

        /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

        /* Check if the user-type OTA Control Characteristic was written.
         * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control)
        {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
              evt->data.evt_gatt_server_user_write_request.connection,
              gattdb_ota_control, bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

        /* Add additional event handlers as your application requires */

      default:
        break;
    }
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++)
  {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}
