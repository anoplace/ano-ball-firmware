#include <jendefs.h>
#include <AppHardwareApi.h>

#include "FullColorLed.h"

PRIVATE bool_t validateIndexRange(tsFullColorLed *psFullColorLed, uint8 u8Index);

/**
 *
 * @param
 */
PUBLIC void vFullColorLed_Init(tsFullColorLed *psFullColorLed) {
  for (int i = 0; i < psFullColorLed->u8PCA9685Length; i++) {
    vPCA9685_Init(&psFullColorLed->sPCA9685s[i]);
  }
}

/**
 *
 * @param
 */
PUBLIC void vFullColorLed_setLedRaw(tsFullColorLed *psFullColorLed,
                                    uint8 u8Index, uint16 u16Red,
                                    uint16 u16Green, uint16 u16Blue,
                                    uint16 u16White) {
  uint8 u8DeviceIndex = u8Index / PCA9685_LED_RGBW_NUMBER;
  uint8 u8LedIndex = u8Index % PCA9685_LED_RGBW_NUMBER;

  if (validateIndexRange(psFullColorLed, u8Index)) {
    vPCA9685_setRgbwLedRaw(&(psFullColorLed->sPCA9685s[u8DeviceIndex]), u8LedIndex,
                           u16Red, u16Green, u16Blue, u16White);
  }
}

/**
 *
 * @param
 */
PUBLIC void vFullColorLed_setLed(tsFullColorLed *psFullColorLed, uint8 u8Index,
                                 uint32 u32Color) {
  uint8 u8DeviceIndex = u8Index / PCA9685_LED_RGBW_NUMBER;
  uint8 u8LedIndex = u8Index % PCA9685_LED_RGBW_NUMBER;

  if (validateIndexRange(psFullColorLed, u8Index)) {
    vPCA9685_setRgbwLed(&(psFullColorLed->sPCA9685s[u8DeviceIndex]), u8LedIndex,
                        u32Color);
  }
}

/**
 *
 * @param
 */
PRIVATE bool_t
validateIndexRange(tsFullColorLed *psFullColorLed, uint8 u8Index) {
  return u8Index < psFullColorLed->u8PCA9685Length * PCA9685_LED_RGBW_NUMBER;
}
