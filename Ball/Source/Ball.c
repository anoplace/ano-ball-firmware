/**
 * @file Ball.c
 * @author Yuuki Taguchi
 */

#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "utils.h"

#include "Ball.h"
#include "config.h"
#include "Version.h"

// DEBUG options
#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

// Select Modules (define befor include "ToCoNet.h")
//#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
//#define ToCoNet_USE_MOD_NBSCAN_SLAVE

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

#include "SMBus.h"
#include "LPR9201.h"
#include "PCA9685.h"
#include "FullColorLed.h"
#include "Effect.h"

typedef struct {
  // MAC
  uint8 u8channel;
  uint16 u16addr;

  // LED Counter
  uint32 u32LedCt;

  // sequence number
  uint32 u32Seq;

  // sleep counter
  uint8 u8SleepCt;
} tsAppData;

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);

void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt);
void vSerialInit2(uint32 u32Baud, tsUartOpt *pUartOpt);
static void vHandleSerialInput(void);

void lpr9201Send(uint8 *data, int length);

/* Local data used by the tag during operation */
static tsAppData sAppData;

PUBLIC tsFILE sSerStream;
PUBLIC tsFILE sSerStream2;
tsSerialPortSetup sSerPort;
tsSerialPortSetup sSerPort2;

// Wakeup port
const uint32 u32DioPortWakeUp = 1UL << 7;  // UART Rx Port

tsPCA9685 sPCA9685s[] = {
    {0x7F},  //
    {0x7E},  //
    {0x7D},  //
};

uint8 u8LedMappingTable[] = {
    8, 10, 11, 3, 2, 9, 5, 1, 7, 0, 4, 6,
};

tsFullColorLed sFullColorLed = {
    sPCA9685s,                                 //
    sizeof(sPCA9685s) / sizeof(sPCA9685s[0]),  //
    u8LedMappingTable                          //
};

tsEffect sEffects[12];

Result lpr9201Result;

/**
 * AppColdStart
 * @param bAfterAhiInit
 */
void cbAppColdStart(bool_t bAfterAhiInit) {
  // static uint8 u8WkState;
  if (!bAfterAhiInit) {
    // before AHI init, very first of code.

    // Register modules
    ToCoNet_REG_MOD_ALL();

  } else {
    // disable brown out detect
    vAHI_BrownOutConfigure(
        E_AHI_VBOREF_2V0,  // voltage threshold for brownout 2.0V
        FALSE,             // disable reset on brownout
        FALSE,             // disable SVM
        FALSE,             // disable brownout falling interrupt
        FALSE);            // disable brownout rising interrupt

    // clear application context
    memset(&sAppData, 0, sizeof(sAppData));
    sAppData.u8channel = CHANNEL;

    // ToCoNet configuration
    sToCoNet_AppContext.u32AppId = APP_ID;
    sToCoNet_AppContext.u8Channel = CHANNEL;
    sToCoNet_AppContext.bRxOnIdle = TRUE;

    // others
    SPRINTF_vInit128();

    // Register
    ToCoNet_Event_Register_State_Machine(vProcessEvCore);

    //
    memset(&lpr9201Result, 0, sizeof(lpr9201Result));
    lpr9201_parser_init(&lpr9201Result);
    //

    // Others
    vInitHardware(FALSE);

    // MAC start
    ToCoNet_vMacStart();
  }
}

static bool_t bWakeupByButton;

/**
 * AppWarmStart
 * @param bAfterAhiInit
 */
void cbAppWarmStart(bool_t bAfterAhiInit) {
  if (!bAfterAhiInit) {
    // before AHI init, very first of code.
    //  to check interrupt source, etc.
    bWakeupByButton = FALSE;

    if (u8AHI_WakeTimerFiredStatus()) {
      // wake up timer
    } else if (u32AHI_DioWakeStatus() & u32DioPortWakeUp) {
      // woke up from DIO events
      bWakeupByButton = TRUE;
    } else {
      bWakeupByButton = FALSE;
    }
  } else {
    // initialize hardware
    vInitHardware(TRUE);

    // MAC start
    ToCoNet_vMacStart();
  }
}

/**
 * AppMain
 */
void cbToCoNet_vMain(void) {
  // handle uart input
  vHandleSerialInput();
}

/**
 * NwkEvent
 * @param eEvent
 * @param u32arg
 */
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
  switch (eEvent) {
    case E_EVENT_TOCONET_PANIC: {
      // パニックが起きたら、原因をコンソールに表示
      tsPanicEventInfo *pInfo = (void *)u32arg;
      pInfo->bCancelReset = TRUE;  // 直後にリセットしない
      vfPrintf(&sSerStream, LB "PANIC! %d/%s",  //
               pInfo->u8ReasonCode, pInfo->strReason);
      SERIAL_vFlush(sSerStream.u8Device);
    } break;

    default:
      break;
  }
}

/**
 * RxEvent
 * @param pRx
 */
void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
  int i;
  static uint16 u16seqPrev = 0xFFFF;

  // print coming payload
  vfPrintf(&sSerStream, LB "[PKT Ad:%04x,Ln:%03d,Seq:%03d,Lq:%03d,Tms:%05d \"",
           pRx->u32SrcAddr,
           pRx->u8Len + 4,  // actual payload byte: the network layer uses
                            // additional 4 bytes.
           pRx->u8Seq, pRx->u8Lqi, pRx->u32Tick & 0xFFFF);
  for (i = 0; i < pRx->u8Len; i++) {
    if (i < 32) {
      sSerStream.bPutChar(sSerStream.u8Device,
                          (pRx->auData[i] >= 0x20 && pRx->auData[i] <= 0x7f)
                              ? pRx->auData[i]
                              : '.');
    } else {
      vfPrintf(&sSerStream, "..");
      break;
    }
  }
  vfPrintf(&sSerStream, "C\"]");

  // 打ち返す
  if (pRx->u8Seq != u16seqPrev  // シーケンス番号による重複チェック
      && !memcmp(pRx->auData, "PING:", 5)  // パケットの先頭は PING: の場合
      ) {
    u16seqPrev = pRx->u8Seq;

    // transmit Ack back
    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();  //
    tsTx.u32DstAddr = pRx->u32SrcAddr;         // 送り返す

    tsTx.bAckReq = TRUE;
    tsTx.u8Retry = 0;
    tsTx.u8CbId = pRx->u8Seq;
    tsTx.u8Seq = pRx->u8Seq;
    tsTx.u8Len = pRx->u8Len;
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

    if (tsTx.u8Len > 0) {
      memcpy(tsTx.auData, pRx->auData, tsTx.u8Len);
    }
    tsTx.auData[1] = 'O';  // メッセージを PONG に書き換える

    ToCoNet_bMacTxReq(&tsTx);

    // turn on LED a while
    sAppData.u32LedCt = u32TickCount_ms;

    // UARTに出力
    vfPrintf(&sSerStream, LB "Fire PONG Message to %08x" LB, pRx->u32SrcAddr);
  } else if (!memcmp(pRx->auData, "PONG:", 5)) {
    // UARTに出力
    vfPrintf(&sSerStream, LB "PONG Message from %08x" LB, pRx->u32SrcAddr);
  }
}

/**
 * TxEvent
 * @param u8CbId
 * @param bStatus
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) { return; }

/**
 * HwEvent
 * Process any hardware events.
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
  switch (u32DeviceId) {
    case E_AHI_DEVICE_TICK_TIMER:
      // LED blink
      // vPortSet_TrueAsLo(PORT_KIT_LED2, u32TickCount_ms & 0x400);

      if (u32TickCount_ms & 0x8) {
        for (uint8 i = 0; i < 12; i++) {
          if (vEffect_Update(&sEffects[i])) {
            vfPrintf(&sSerStream, LB "effect update %d" LB, i);

//            vfPrintf(&sSerStream, LB "effect red %d" LB, (int)(sEffects[i].color.fRed * 4096));
//            vfPrintf(&sSerStream, LB "effect green %d" LB, (int)(sEffects[i].color.fGreen * 4096));
//            vfPrintf(&sSerStream, LB "effect blue %d" LB, (int)(sEffects[i].color.fBlue * 4096));
//
//            SERIAL_vFlush(sSerStream.u8Device);

            vFullColorLed_setLedRaw(&sFullColorLed, i,                //
                                    sEffects[i].color.fRed * 4096,    //
                                    sEffects[i].color.fGreen * 4096,  //
                                    sEffects[i].color.fBlue * 4096,   //
                                    0x0000);
          }
        }
      }

      // LED on when receive
      if (u32TickCount_ms - sAppData.u32LedCt < 300) {
        // vPortSetLo(PORT_KIT_LED1);
      } else {
        // vPortSetHi(PORT_KIT_LED1);
      }
      break;

    default:
      break;
  }
}

/**
 * HwInt
 * Called during an interrupt.
 * @param u32DeviceId
 * @param u32ItemBitmap
 * @retval TRUE interrupt is handled, no further call.
 * @retval FALSE interrupt is not handled, escalated to further event call
 * (cbToCoNet_vHwEvent).
 */
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
  return FALSE;
}

/**
 * InitHardware
 */
static void vInitHardware(int f_warm_start) {
// serial initialize
#if 0
    /*
	tsUartOpt sUartOpt;
	memset(&sUartOpt, 0, sizeof(tsUartOpt));
	sUartOpt.bHwFlowEnabled = FALSE;
	sUartOpt.bParityEnabled = E_AHI_UART_PARITY_ENABLE;
	sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
	sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
	sUartOpt.u8WordLen = 7;

	vSerialInit(UART_BAUD, &sUartOpt);
	*/
#else
  vSerialInit(UART_BAUD, NULL);
  vSerialInit2(230400, NULL);
#endif

  ToCoNet_vDebugInit(&sSerStream);
  ToCoNet_vDebugLevel(0);

  // GPIOを初期化
  // vPortSetLo(PORT_KIT_LED1);
  // vPortSetHi(PORT_KIT_LED2);
  // vPortAsOutput(PORT_KIT_LED1);
  // vPortAsOutput(PORT_KIT_LED2);

  // I2Cを初期化
  vSMBusInit();

  // FullColorLedを初期化
  vFullColorLed_Init(&sFullColorLed);

  vWait(1000 * 10000);

  {
    // lpr9201を起動
    uint8 data[] = {0x5A, 0xA5, 0x0B, 0x00, 0xF4};
    lpr9201Send(data, sizeof(data) / sizeof(data[0]));
  }

  // Effectを初期化
  for (uint8 i = 0; i < sizeof(sEffects) / sizeof(sEffects[0]); i++) {
    vEffect_Init(&sEffects[i]);
  }
}

/**
 * SerialInit
 * @param u32Baud
 * @param pUartOpt
 */
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
  /* create the debug port transmit and receive queues */
  static uint8 au8SerialTxBuffer[96];
  static uint8 au8SerialRxBuffer[32];

  vAHI_UartSetLocation(UART_PORT_SLAVE, FALSE);

  /* initialise the serial port to be used for debug output */
  sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
  sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
  sSerPort.u32BaudRate = u32Baud;
  sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
  sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
  sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
  sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
  sSerPort.u8SerialPort = UART_PORT_SLAVE;
  sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
  SERIAL_vInitEx(&sSerPort, pUartOpt);

  sSerStream.bPutChar = SERIAL_bTxChar;
  sSerStream.u8Device = UART_PORT_SLAVE;
}

/**
 * SerialInit
 * @param u32Baud
 * @param pUartOpt
 */
void vSerialInit2(uint32 u32Baud, tsUartOpt *pUartOpt) {
  /* create the debug port transmit and receive queues */
  static uint8 au8SerialTxBuffer[96];
  static uint8 au8SerialRxBuffer[96];

  vAHI_UartSetLocation(E_AHI_UART_1, TRUE);

  /* initialise the serial port to be used for debug output */
  sSerPort2.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
  sSerPort2.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
  sSerPort2.u32BaudRate = u32Baud;
  sSerPort2.u16AHI_UART_RTS_LOW = 0xffff;
  sSerPort2.u16AHI_UART_RTS_HIGH = 0xffff;
  sSerPort2.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
  sSerPort2.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
  sSerPort2.u8SerialPort = E_AHI_UART_1;
  sSerPort2.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
  SERIAL_vInitEx(&sSerPort2, pUartOpt);

  sSerStream2.bPutChar = SERIAL_bTxChar;
  sSerStream2.u8Device = E_AHI_UART_1;
}

void lpr9201Send(uint8 *data, int length) {
  for (int i = 0; i < length; i++) {
    vPutChar(&sSerStream2, data[i]);
  }

  vfPrintf(&sSerStream, "\n\r# sended!");
}

/**
 * HandleSerialInput
 */
static void vHandleSerialInput(void) {
  // handle UART command
  while (!SERIAL_bRxQueueEmpty(sSerPort2.u8SerialPort)) {
    int16 i16Char = SERIAL_i16RxChar(sSerPort2.u8SerialPort);

    vfPrintf(&sSerStream, "\n\r# lpr9201 [0x%02X]", i16Char);
    SERIAL_vFlush(sSerStream.u8Device);

    if (lpr9201_parser_parse(i16Char, &lpr9201Result) &&
        lpr9201Result.resultCode == 0x83) {  // dataのみ
      vfPrintf(&sSerStream, "\n\r# lpr9201 success parse");

      for (int i = lpr9201Result.dataOffset;
           i < lpr9201Result.dataOffset + lpr9201Result.dataLength; i++) {
        vfPrintf(&sSerStream, "\n\r# lpr9201 parsed [0x%02X]",
                 lpr9201Result.receiveData[i]);
        SERIAL_vFlush(sSerStream.u8Device);
      }

      vfPrintf(&sSerStream, "\n\r# lpr9201 length: %d",
               lpr9201Result.dataLength);
      SERIAL_vFlush(sSerStream.u8Device);

      if (lpr9201Result.dataLength >= 36) {  // FIXME
        uint8 *u8Data = lpr9201Result.receiveData;

        uint16 duration = 1000;

        for (int i = 0; i < 12; i++) {
          uint8 offset = lpr9201Result.dataOffset + i * 3;

          tsColor c = {
              u8Data[offset + 0] / 255.0,  //
              u8Data[offset + 1] / 255.0,  //
              u8Data[offset + 2] / 255.0   //
          };

//          vEffect_Gradation(&sEffects[i],       //
//                            sEffects[i].color,  //
//                            c,                  //
//                            duration);
        }
      }
    }

    SERIAL_vFlush(sSerStream.u8Device);
  }

  // handle UART command
  while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
    int16 i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

    vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
    SERIAL_vFlush(sSerStream.u8Device);

    switch (i16Char) {
      /*
      case 'a': {
        uint8 data[] = {0x5A, 0xA5, 0x00, 0x00, 0xFF};
        lpr9201Send(data, sizeof(data) / sizeof(data[0]));

        vfPrintf(&sSerStream, "\n\r# connectionConfirmation sended!");
      } break;

      case 'b': {
        uint8 data[] = {0x5A, 0xA5, 0x0B, 0x00, 0xF4};
        lpr9201Send(data, sizeof(data) / sizeof(data[0]));

        vfPrintf(&sSerStream, "\n\r# activate sended!");
      } break;

      case 'c': {
        uint8 data[] = {0x5A, 0xA5, 0x03, 0x01, 0x02, 0xFF};
        lpr9201Send(data, sizeof(data) / sizeof(data[0]));

        vfPrintf(&sSerStream, "\n\r# read parameter 2 sended!");
      } break;

      case 'd': {
        uint8 data[] = {0x5a,
                        0xa5,
                        0x2,
                        0x3,
                        0x2,
                        0x3,
                        0xef,
                        0x10,};
        lpr9201Send(data, sizeof(data) / sizeof(data[0]));

        vfPrintf(&sSerStream, "\n\r# write 1007 parameter 2 sended!");
      } break;

      case 'e': {
        uint8 data[] = {
        0x5a,
        0xa5,
        0x06,
        0x01,
        0x00,
        0xf8,
        };
        lpr9201Send(data, sizeof(data) / sizeof(data[0]));

        vfPrintf(&sSerStream, "\n\r# write profile sended!");
      } break;
      */

      case 'r': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 4096, 0, 0, 0);
        }
      } break;

      case 'R': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 300, 0, 0, 0);
        }
      } break;

      case 'g': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 0, 4096, 0, 0);
        }
      } break;

      case 'G': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 0, 300, 0, 0);
        }
      } break;

      case 'b': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 0, 0, 4096, 0);
        }
      } break;

      case 'B': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 0, 0, 300, 0);
        }
      } break;

      case 'x': {
        for (int i = 0; i < 12; i++) {
          vFullColorLed_setLedRaw(&sFullColorLed, i, 0, 0, 0, 0);
        }
      } break;

      default:
        break;
    }

    vfPrintf(&sSerStream, LB);
    SERIAL_vFlush(sSerStream.u8Device);
  }
}

/**
 * ProcessEvCore
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
  if (eEvent == E_EVENT_START_UP) {
    // here it is safe if the output message of the UART
    if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
      vfPrintf(&sSerStream, LB "RAMHOLD");
    }
    if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
      vfPrintf(&sSerStream, LB "Wake up by %s. SleepCt=%d",
               bWakeupByButton ? "UART PORT" : "WAKE TIMER",
               sAppData.u8SleepCt);
    } else {
      vfPrintf(&sSerStream, "\r\n*** ToCoNet PINGPONG SAMPLE %d.%02d-%d ***",
               VERSION_MAIN, VERSION_SUB, VERSION_VAR);
      vfPrintf(&sSerStream, "\r\n*** %08x ***", ToCoNet_u32GetSerial());
    }
  }
}
