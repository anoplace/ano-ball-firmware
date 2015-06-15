#include <jendefs.h>
#include <AppHardwareApi.h>

#include "PCA9685.h"
#include "SMBus.h"

/**
 *
 */
PUBLIC void vPCA9685_Init(tsPCA9685 *psPCA9685) {
  // mode1
  vPCA9685_writeRegister(psPCA9685, PCA9685_MODE1,
                         (1 << PCA9685_MODE1_RESTART) |
                             (1 << PCA9685_MODE1_AI) |
                             (1 << PCA9685_MODE1_ALLCALL));

  // mode2
  vPCA9685_writeRegister(psPCA9685, PCA9685_MODE2, (1 << PCA9685_MODE2_OUTDRV));

  // prescale
  vPCA9685_prescale(psPCA9685, PCA9685_PRESCALE_INTERNAL_OSC,
                    PCA9685_PRESCALE_DEFAULT_FREQUENCY);

  // all led off
  vPCA9685_setAllLedRaw(psPCA9685, 0x0000);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_setLedRaw(tsPCA9685 *psPCA9685, uint8 index,
                               uint16 value) {
  uint8 u8Data[] = {
      0x00, 0x00,                                // led on
      U16_LOWER_U8(value), U16_UPPER_U8(value),  // led off
  };
  vPCA9685_writeRegisters(psPCA9685, PCA9685_LED0_ON_L + index * 4, u8Data,
                          sizeof(u8Data) / sizeof(u8Data[0]));
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_setLed(tsPCA9685 *psPCA9685, uint8 index, float fPercent) {
  vPCA9685_setLedRaw(psPCA9685, index, 4096 * fPercent);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_setAllLedRaw(tsPCA9685 *psPCA9685, uint16 value) {
  uint8 u8Data[] = {
      0x00, 0x00,                                // led on
      U16_LOWER_U8(value), U16_UPPER_U8(value),  // led off
  };
  vPCA9685_writeRegisters(psPCA9685, PCA9685_ALL_LED_ON_L, u8Data,
                          sizeof(u8Data) / sizeof(u8Data[0]));
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_setAllLed(tsPCA9685 *psPCA9685, float fPercent) {
  vPCA9685_setAllLedRaw(psPCA9685, 4096 * fPercent);
}

/**
 * //FIXME 色の配列は基板によって異なるので、対応できるようにする
 * @param
 */
PUBLIC void vPCA9685_setRgbwLedRaw(tsPCA9685 *psPCA9685, uint8 u8Index,
                                   uint16 u16Red, uint16 u16Green,
                                   uint16 u16Blue, uint16 u16White) {
  uint8 u8Data[] = {
      0x00, 0x00,                                    // blue led on
      U16_LOWER_U8(u16Blue), U16_UPPER_U8(u16Blue),  // blue led off

      0x00, 0x00,                                  // red led on
      U16_LOWER_U8(u16Red), U16_UPPER_U8(u16Red),  // red led off

      0x00, 0x00,                                      // green led on
      U16_LOWER_U8(u16Green), U16_UPPER_U8(u16Green),  // green led off

      0x00, 0x00,                                      // white led on
      U16_LOWER_U8(u16White), U16_UPPER_U8(u16White),  // white led off
  };
  vPCA9685_writeRegisters(psPCA9685, PCA9685_LED0_ON_L + u8Index * 16, u8Data,
                          sizeof(u8Data) / sizeof(u8Data[0]));
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_setRgbwLed(tsPCA9685 *psPCA9685, uint8 u8Index,
                                uint32 u32Color) {
  uint8 u16White = (u32Color >> 24) & 0xFF;
  uint8 u16Red = (u32Color >> 16) & 0xFF;
  uint8 u16Green = (u32Color >> 8) & 0xFF;
  uint8 u16Blue = (u32Color >> 0) & 0xFF;

  vPCA9685_setRgbwLedRaw(psPCA9685, u8Index,     //
                         u16Red * 4096 / 256,    //
                         u16Green * 4096 / 256,  //
                         u16Blue * 4096 / 256,   //
                         u16White * 4096 / 256);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_prescale(tsPCA9685 *psPCA9685, uint32 u32Clock,
                              uint16 u16Frequency) {
  u16Frequency = MIN(u16Frequency, 1526);  // 最大値を制限
  u16Frequency = MAX(u16Frequency, 24);    // 最小値を制限

  uint8 u8Prescale =
      (u32Clock / 4096.0 / u16Frequency) - 1 + 0.5;  // 0.5足すことで四捨五入

  vPCA9685_writeRegister(psPCA9685, PCA9685_PRESCALE, u8Prescale);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_testMode(tsPCA9685 *psPCA9685) {
  bSMBusWrite(psPCA9685->slaveAddress, PCA9685_TEST_MODE, 0, NULL);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_reset(tsPCA9685 *psPCA9685) {
  bSMBusWrite(0x00, PCA9685_SOFTWARE_RESET_ADDRESS, 0,
              NULL);  // general call address
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_readRegister(tsPCA9685 *psPCA9685, uint8 u8Register,
                                  uint8 *u8Data) {
  bSMBusWrite(psPCA9685->slaveAddress, u8Register, 0, NULL);
  bSMBusSequentialRead(psPCA9685->slaveAddress, 1, u8Data);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_writeRegister(tsPCA9685 *psPCA9685, uint8 u8Register,
                                   uint8 u8Value) {
  uint8 u8Data[] = {u8Value};
  bSMBusWrite(psPCA9685->slaveAddress, u8Register,
              sizeof(u8Data) / sizeof(u8Data[0]), u8Data);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_readRegisters(tsPCA9685 *psPCA9685, uint8 u8Register,
                                   uint8 *pu8Data, uint8 u8Length) {
  bSMBusWrite(psPCA9685->slaveAddress, u8Register, 0, NULL);
  bSMBusSequentialRead(psPCA9685->slaveAddress, u8Length, pu8Data);
}

/**
 *
 * @param
 */
PUBLIC void vPCA9685_writeRegisters(tsPCA9685 *psPCA9685, uint8 u8Register,
                                    uint8 *pu8Data, uint8 u8Length) {
  bSMBusWrite(psPCA9685->slaveAddress, u8Register, u8Length, pu8Data);
}
