#ifndef FULLCOLORLED_H_INCLUDED
#define FULLCOLORLED_H_INCLUDED

#include "PCA9685.h"

#if defined __cplusplus
extern "C" {
#endif

typedef struct {
  tsPCA9685 *sPCA9685s;     //
  uint8 u8PCA9685Length;    //
  uint8 *u8LedMappingTable  //
} tsFullColorLed;

PUBLIC void vFullColorLed_Init(tsFullColorLed *psFullColorLed);
PUBLIC void vFullColorLed_setLedRaw(tsFullColorLed *psFullColorLed,
                                    uint8 u8Index, uint16 u16Red,
                                    uint16 u16Green, uint16 u16Blue,
                                    uint16 u16White);
PUBLIC void vFullColorLed_setLed(tsFullColorLed *psFullColorLed, uint8 u8Index,
                                 uint32 u32Color);

#if defined __cplusplus
}
#endif

#endif /* FULLCOLORLED_H_INCLUDED */
