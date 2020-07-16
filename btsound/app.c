#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "app.h"
#include "pdm_stereo_ldma.h"

static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);
static uint8_t boot_to_dfu = 0;

void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  initLog();
  gecko_init(pconfig);

  while (1)
  {
    struct gecko_cmd_packet * evt;
    if (!gecko_event_pending())
      flushLog();

    evt = gecko_wait_event(); // check for stack event. this is a blocking event listener. if you want non-blocking please see UG136.

    switch (BGLIB_MSG_ID(evt->header))
    {
      case gecko_evt_system_boot_id:
		#if DEBUG_LEVEL
    	  bootMessage(&(evt->data.evt_system_boot));
		#endif
        printLog("boot event - starting advertising\r\n");
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      case gecko_evt_le_connection_opened_id:
        printLog("connection opened\r\n");
        break;

      case gecko_evt_gatt_server_attribute_value_id:
		if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_Read_Write)
		{
			int c = evt->data.evt_gatt_server_attribute_value.value.data[0]; //gets value
			printLog("new rw value %c\r\n", c);
			if (c == 't') //t is for transmit
			{
				initLdma();
				pdmReset();
				while (!pdmDone())
				{
					EMU_EnterEM1();
					pdmPass();
				}
				pdmPrintLeft();
				pdmPrintRight();
				stopLdma();
			}
		}
		break;

      case gecko_evt_le_connection_closed_id:
        printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);
        if (boot_to_dfu)
          gecko_cmd_system_reset(2);
        else
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      case gecko_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control)
        {
          boot_to_dfu = 1;
          gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, gattdb_ota_control, bg_err_success);
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      /* Add additional event handlers as your application requires */

      default:
    	printLog("default\r\n");
        break;
    }
    printLog("ready\r\n"); flushLog();
// Infinite Loop
    char ch = 'c'; //for infinite, auto, replace this with 'r'
    while (1)
    {
    	if (ch == 'r')
    	{
    		pdmReset();
			while (!pdmDone())
			{
				EMU_EnterEM1();
				pdmPass();
			}
			pdmPrintLeft();
			pdmPrintRight();
			printLog("done:\r\n"); flushLog();
    	}
    	ch = RETARGET_ReadChar();
    }
  }
}

static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
  bd_addr local_addr;
  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;
  printLog("local BT device address: ");
  for (int i = 0; i < 5; i++)
    printLog("%2.2x:", local_addr.addr[5 - i]);
  printLog("%2.2x\r\n", local_addr.addr[0]);
}
