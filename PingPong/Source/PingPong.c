/**
 * @file PingPong.c
 * @author Yuuki Taguchi
 */

#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "utils.h"

#include "PingPong.h"
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
static void vHandleSerialInput(void);

/* Local data used by the tag during operation */
static tsAppData sAppData;

PUBLIC tsFILE sSerStream;
tsSerialPortSetup sSerPort;

// Wakeup port
const uint32 u32DioPortWakeUp = 1UL << 7;  // UART Rx Port

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
  vfPrintf(
      &sSerStream, LB "[PKT Ad:%04x,Ln:%03d,Seq:%03d,Lq:%03d,Tms:%05d \"",
      pRx->u32SrcAddr,
      pRx->u8Len +
          4,  // actual payload byte: the network layer uses additional 4 bytes.
      pRx->u8Seq,
      pRx->u8Lqi, pRx->u32Tick & 0xFFFF);
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
      vPortSet_TrueAsLo(PORT_KIT_LED2, u32TickCount_ms & 0x400);

      // LED on when receive
      if (u32TickCount_ms - sAppData.u32LedCt < 300) {
        vPortSetLo(PORT_KIT_LED1);
      } else {
        vPortSetHi(PORT_KIT_LED1);
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
	tsUartOpt sUartOpt;
	memset(&sUartOpt, 0, sizeof(tsUartOpt));
	sUartOpt.bHwFlowEnabled = FALSE;
	sUartOpt.bParityEnabled = E_AHI_UART_PARITY_ENABLE;
	sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
	sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
	sUartOpt.u8WordLen = 7;

	vSerialInit(UART_BAUD, &sUartOpt);
#else
  vSerialInit(UART_BAUD, NULL);
#endif

  ToCoNet_vDebugInit(&sSerStream);
  ToCoNet_vDebugLevel(0);

  // GPIO initialize
  vPortSetLo(PORT_KIT_LED1);
  vPortSetHi(PORT_KIT_LED2);
  vPortAsOutput(PORT_KIT_LED1);
  vPortAsOutput(PORT_KIT_LED2);
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
 * HandleSerialInput
 */
static void vHandleSerialInput(void) {
  // handle UART command
  while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
    int16 i16Char;

    i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

    vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
    SERIAL_vFlush(sSerStream.u8Device);

    switch (i16Char) {
      case '>':
      case '.': {  // channel up
        sAppData.u8channel++;
        if (sAppData.u8channel > 26) sAppData.u8channel = 11;
        sToCoNet_AppContext.u8Channel = sAppData.u8channel;
        ToCoNet_vRfConfig();
        vfPrintf(&sSerStream, "set channel to %d.", sAppData.u8channel);
      } break;

      case '<':
      case ',': {  // channel down
        sAppData.u8channel--;
        if (sAppData.u8channel < 11) sAppData.u8channel = 26;
        sToCoNet_AppContext.u8Channel = sAppData.u8channel;
        ToCoNet_vRfConfig();
        vfPrintf(&sSerStream, "set channel to %d.", sAppData.u8channel);
      } break;

      case 'd':
      case 'D': {
        static uint8 u8DgbLvl;

        u8DgbLvl++;
        if (u8DgbLvl > 5) u8DgbLvl = 0;
        ToCoNet_vDebugLevel(u8DgbLvl);

        vfPrintf(&sSerStream, "set NwkCode debug level to %d.", u8DgbLvl);
      } break;

      case 's':
      case 'S': {  // sleep test
        // print message.
        sAppData.u8SleepCt++;

        // stop interrupt source, if interrupt source is still running.

        vfPrintf(&sSerStream, "now sleeping" LB);
        SERIAL_vFlush(sSerStream.u8Device);  // flushing

        if (i16Char == 's') {
          vAHI_UartDisable(sSerStream.u8Device);
        }

        // set UART Rx port as interrupt source
        vAHI_DioSetDirection(u32DioPortWakeUp, 0);  // set as input

        (void)u32AHI_DioInterruptStatus();        // clear interrupt register
        vAHI_DioWakeEnable(u32DioPortWakeUp, 0);  // also use as DIO WAKE SOURCE
        // vAHI_DioWakeEdge(0, PORT_INPUT_MASK); //
        // 割り込みエッジ（立下りに設定）
        vAHI_DioWakeEdge(u32DioPortWakeUp,
                         0);  // 割り込みエッジ（立上がりに設定）
        // vAHI_DioWakeEnable(0, PORT_INPUT_MASK); // DISABLE DIO WAKE SOURCE

        // wake up using wakeup timer as well.
        // ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, 0, FALSE, TRUE); // RAM OFF
        // SLEEP USING WK0
        ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, 0, FALSE,
                       FALSE);  // RAM ON SLEEP USING WK0
      } break;

      case 'p': {                // RF power
        static uint8 u8pow = 3;  // (MIN)0..3(MAX)

        u8pow = (u8pow + 1) % 4;
        vfPrintf(&sSerStream, "set power to %d.", u8pow);

        sToCoNet_AppContext.u8TxPower = u8pow;
        ToCoNet_vRfConfig();
      } break;

      case 't': {  // send packet
        // transmit Ack back
        tsTxDataApp tsTx;
        memset(&tsTx, 0, sizeof(tsTxDataApp));

        sAppData.u32Seq++;

        tsTx.u32SrcAddr = ToCoNet_u32GetSerial();
        tsTx.u32DstAddr = 0xFFFF;  // broadcast

        tsTx.bAckReq = FALSE;
        tsTx.u8Retry = 2 | 0x80;  // retry 2 times
        tsTx.u8CbId = sAppData.u32Seq & 0xFF;
        tsTx.u8Seq = sAppData.u32Seq & 0xFF;
        tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

        // create message by SPRINTF
        SPRINTF_vRewind();
        vfPrintf(SPRINTF_Stream, "PING: %08X", ToCoNet_u32GetSerial());
        memcpy(tsTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
        tsTx.u8Len = SPRINTF_u16Length();

        // send
        ToCoNet_bMacTxReq(&tsTx);

        // LED control
        sAppData.u32LedCt = u32TickCount_ms;

        // output to UART
        vfPrintf(&sSerStream, LB "Fire PING Broadcast Message.");
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
