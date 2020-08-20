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
#include "print_util/print_usb.h"

static uint8 _conn_handle = 0xFF;

static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet

enum
{
  HANDLE_ADV_SOUND, HANDLE_ADV_DATA
};

uint8 adv_data[9] = { 0x02, 0x01, 0x06, 0x05, 'h', 'e', 'l', 'l', 'o' };
uint8 scan_resp_data[9] = { 0x02, 0x01, 0x06, 0x05, 'w', 'o', 'r', 'l', 'd' };

void send_ota_buf(uint8_t *buf, int len)
{
  uint16 result;
  uint8_t *temp = buf;
  printLog("start send_ota_buf\r\n");
  while (len > _max_packet_size)
  {
    do
    {
      result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle,
          gattdb_gatt_spp_data, _max_packet_size, temp)->result;
    }
    while (result == bg_err_out_of_memory);
    len -= _max_packet_size;
    temp += _max_packet_size;
  }
  if (len > 0)
  {
    do
    {
      result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle,
          gattdb_gatt_spp_data, len, temp)->result;
    }
    while (result == bg_err_out_of_memory);
  }
}

void parse_rw(char c)
{
  switch (c)
  {
    case 'w':
      send_ota_buf((uint8_t *) list_pwm, TIMER_BUFFER_SIZE << 1);
      break;
//	case 't':
//		send_ota_buf((uint8_t *)list_top, TIMER_BUFFER_SIZE<<1);
//		break;
    case 'l':
      send_ota_buf((uint8_t *) left, BUFFER_SIZE << 1);
      break;
    case 'r':
      send_ota_buf((uint8_t *) right, BUFFER_SIZE << 1);
      break;
    case 'c':
      send_ota_buf((uint8_t *) corr, BUFFER_SIZE << 1);
      break;
    case 'g':
    {
      // read k from spp_data
      int32_t *k;
      float p;
      k = (int32_t *) gecko_cmd_gatt_server_read_attribute_value(
      gattdb_gatt_spp_data, 0)->value.data;
      uint8_t len = gecko_cmd_gatt_server_read_attribute_value(
      gattdb_gatt_spp_data, 0)->value.len;
      printLog("got goertzel k: %ld, %d\r\n", k[0], len);
      for (int offset = -10; offset < 11; offset++)
      {
        p = goertzel(left, k[0] + offset);
        // send_ota_buf((uint8_t *) &p, 4);
        printLog("%ld, ", k[0] + offset);
        printLog("%e, ", p);
        p = goertzel(right, k[0] + offset);
        // send_ota_buf((uint8_t *) &p, 4);
        printLog("%e\r\n", p);
      }
      break;
    }
    case 'k':
    {
      // read k from spp_data
      int32_t *k;
      k = (int32_t *) gecko_cmd_gatt_server_read_attribute_value(
      gattdb_gatt_spp_data, 0)->value.data;
      uint8_t len = gecko_cmd_gatt_server_read_attribute_value(
      gattdb_gatt_spp_data, 0)->value.len;
      printLog("got ota k: %ld, %d\r\n", k[0], len);
      populateBuffers(k[0]);
      break;
    }
    case '0':
    {
      // read pulsewidth from spp_data
      int8_t *pw;
      pw = (int8_t *) gecko_cmd_gatt_server_read_attribute_value(
      gattdb_gatt_spp_data, 0)->value.data;
      uint8_t len = gecko_cmd_gatt_server_read_attribute_value(
      gattdb_gatt_spp_data, 0)->value.len;
      printLog("got ota pw: %d, %d\r\n", pw[0], len);
      pulse_width = pw[0] * 1e-3;
      populateBuffers(-1);
      break;
    }
    case 'm':  // record microphone
      TIMER0_IRQHandler();
      break;
    case 's':  // play speaker
      play_speaker();
      break;
    case 'a':  // record and play self
      TIMER0_IRQHandler();
      play_speaker();
      break;
    default:
      break;
  }
}

void startAdvertising(uint32 interval)
{
  uint16_t ret = 0;
  // set to only 1 advertising channel to reduce timing problems...
  gecko_cmd_le_gap_set_advertise_channel_map(HANDLE_ADV_SOUND, 1);
  gecko_cmd_le_gap_set_advertise_timing(HANDLE_ADV_SOUND, interval, interval, 0, 0);
  // request event when scan request made
  gecko_cmd_le_gap_set_advertise_report_scan_request(HANDLE_ADV_SOUND, 1);
  ret = gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_ADV_SOUND, 0, 9, adv_data)->result;
  printLog("set adv msg: %d\r\n", ret);
  ret = gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_ADV_SOUND, 1, 9, scan_resp_data)->result;
  printLog("set scan response msg: %d\r\n", ret);

  gecko_cmd_le_gap_start_advertising(HANDLE_ADV_SOUND, le_gap_user_data,
      le_gap_scannable_non_connectable);

  gecko_cmd_le_gap_set_advertise_channel_map(HANDLE_ADV_DATA, 2);
  bd_addr rnd_addr;
  gecko_cmd_le_gap_set_advertise_random_address(HANDLE_ADV_DATA, 2, rnd_addr);
  gecko_cmd_le_gap_set_advertise_timing(HANDLE_ADV_DATA, interval, interval, 0, 0);
  // request event when scan request made
  gecko_cmd_le_gap_set_advertise_report_scan_request(HANDLE_ADV_DATA, 1);
  /* should probably set something here that is useful...
   ret = gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_ADV_DATA, 0, 9, adv_data)->result;
   printLog("set adv msg: %d\r\n", ret);
   ret = gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_ADV_DATA, 1, 9, scan_resp_data)->result;
   printLog("set scan response msg: %d\r\n", ret);
   */
  /* Start general advertising and enable connections. */
  gecko_cmd_le_gap_start_advertising(HANDLE_ADV_DATA, le_gap_general_discoverable,
      le_gap_undirected_connectable);

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
static int process_scan(struct gecko_msg_le_gap_extended_scan_response_evt_t * pResp)
{
  uint8_t channel = pResp->channel;
  int8_t rssi = pResp->rssi;

//  if (pResp->packet_type & 0x4)
//  {
    if (pResp->address.addr[5] == 0x58 && pResp->address.addr[4] == 0x8e)
    {
      printLog("detected ");
      print_mac(pResp->address);
      printLog("\r\n");
//      printLog("\r\nstarted recording\r\n");
//			printLog(", type: ");
//			bin(pResp->packet_type);
//			printLog(", ch:%d, rssi:%d, ", channel, rssi);
//			printLog("data len: %d, data:", pResp->data.len);
//			print_data(&(pResp->data));
//			printLog("\r\n");
      /* start microphone  */
//      TIMER0_IRQHandler();
    }
//  }
  return 0;
}

void init_dma_channels(void)
{
  uint32_t e;
  // Speaker Frequency
//	e = DMADRV_AllocateChannel(&ldma_channelTMR_TOPV, NULL);
//	printLog("DMADRV CH%d (rc: %lu) allocated for TMR_TOPV\r\n", ldma_channelTMR_TOPV, e);
  // Speaker PWM
  e = DMADRV_AllocateChannel(&ldma_channelTMR_COMP, NULL);
  printLog("DMADRV CH%d (rc: %lu) allocated for TMR_COMP\r\n", ldma_channelTMR_COMP, e);
  // Speaker PWM, Inverted (for Differential Drive Speakers only)
#ifdef DIFFERENTIAL_DRIVE
  e = DMADRV_AllocateChannel(&ldma_channelTMR_CINV, NULL);
  printLog("DMADRV CH%d (rc: %lu) allocated for TMR_CINV\r\n", ldma_channelTMR_CINV, e);
#endif
  // Microphone PDM
  e = DMADRV_AllocateChannel(&ldma_channelPDM, NULL);
  printLog("DMADRV CH%d (rc: %lu) allocated for PDM_MICS\r\n", ldma_channelPDM, e);
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
  pconfig->bluetooth.max_advertisers = 2;

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();
  init_speaker();
  initTIMER();
  // init_prs();
  initPDM();
  initTimer0();
  // set initial chirp freq to 1/2 of nyquist
  populateBuffers(BUFFER_SIZE >> 2);
  DMADRV_Init();
  printf("check float: %f\r\n", check_float());
  /* Initialize stack */
  gecko_init(pconfig);

  while (1)
  {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending())
    {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header))
    {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
        gecko_cmd_gatt_set_max_mtu(247);
        bootMessage(&(evt->data.evt_system_boot));
        printLog("boot event - starting advertising\r\n");

        init_dma_channels();
        startDMADRV_TMR();
        prs_gpio(gpioPortB, 0, gpioPortC, 0);

        startAdvertising(320);      //2 ms * 160
        startObserving(3200, 330);  //2  s * 160, just longer than advertising interval: (interval, window)
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
        if (scanner_address.addr[5] == 0x58 && scanner_address.addr[4] == 0x8e)
        {
          printLog("scan request from ");
          print_mac(scanner_address);
          printLog("\r\n");
//          printLog("\r\nstarting speaker!\r\n");
          // play speaker here
//          play_speaker();
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

      case gecko_evt_gatt_mtu_exchanged_id:
        /* Calculate maximum data per one notification / write-without-response, this depends on the MTU.
         * up to ATT_MTU-3 bytes can be sent at once  */
        _max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
        _min_packet_size = _max_packet_size; /* Try to send maximum length packets whenever possible */
        printLog("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
        break;

      case gecko_evt_gatt_server_attribute_value_id:
      {
        switch (evt->data.evt_gatt_server_attribute_value.attribute)
        {
          case gattdb_Read_Write:
          {
            int c = evt->data.evt_gatt_server_attribute_value.value.data[0];
            printLog("new rw value %c\r\n", c);
            parse_rw(c);

            break;
          }
          default:
            printLog("unhandled case: %d\r\n", evt->data.evt_gatt_server_attribute_value.attribute);
            break;
        }
        break;
      }

      case gecko_evt_le_connection_closed_id:
        printLog("connection closed, reason: 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
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
