/*
 * print_usb.c
 *
 *  Created on: Aug 10, 2020
 *      Author: nams
 */

#include "app.h"
#include "print_usb.h"

void bin(uint8_t n) {
	uint8_t i;
	for (i = 1 << 7; i > 0; i = i >> 1)
		(n & i) ? printLog("1") : printLog("0");
}

void print_mac(bd_addr addr) {
	for (int i = 0; i < 5; i++) {
		printLog("%02x:", addr.addr[5 - i]);
	}
	printLog("%02x", addr.addr[0]);
}

void print_data(uint8array *data) {
	for (int i = 0; i < data->len; i++) {
		printLog("%c", data->data[i]);
	}
}

