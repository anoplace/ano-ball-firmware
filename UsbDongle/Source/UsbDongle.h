/**
 * @file UsbDongle.h
 * @author Yuuki Taguchi
 */

#ifndef USBDONGLE_USBDONGLE_H_
#define USBDONGLE_USBDONGLE_H_

#if defined __cplusplus
extern "C" {
#endif

#include "config.h"

#define UART_TXD_DIO 6  // DIO06
#define UART_RXD_DIO 7  // DIO07

#define STATUS_LED_RED_DIO 16   // DIO16
#define STATUS_LED_YELLOW_DO 1  // DIO1 PWM3

#if defined __cplusplus
}
#endif

#endif /* USBDONGLE_USBDONGLE_H_ */
