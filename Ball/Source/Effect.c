#include <jendefs.h>
#include <AppHardwareApi.h>

#include "Effect.h"
#include "ToCoNet.h"  //FIXME 追加しなくて良いようにする
#include "string.h"

PRIVATE bool_t bCompareColor(tsColor *a, tsColor *b);

/**
 *
 * @param
 */
PUBLIC void vEffect_Init(tsEffect *psEffect) {
  psEffect->startColor.fRed = 0.0;
  psEffect->startColor.fGreen = 0.0;
  psEffect->startColor.fBlue = 0.0;

  psEffect->targetColor.fRed = 0.0;
  psEffect->targetColor.fGreen = 0.0;
  psEffect->targetColor.fBlue = 0.0;
}

/**
 *
 * @param
 */
PUBLIC void vEffect_SetColor(tsEffect *psEffect, tsColor *psColor) {
  psEffect->color = *psColor;
  psEffect->startColor = *psColor;
  psEffect->targetColor = *psColor;

  // apply();
}

/**
 *
 * @param
 */
PUBLIC void vEffect_Gradation(tsEffect *psEffect, tsColor *psStartColor,
                              tsColor *psTargetColor, uint16 u16Duration) {
  psEffect->startColor = *psStartColor;
  psEffect->startColor = *psTargetColor;
  psEffect->startEffectTime = u32TickCount_ms;
  psEffect->u16Duration = u16Duration;

  vEffect_Update(psEffect);
}

/**
 *
 * @param
 */
PUBLIC void vEffect_FadeIn(tsEffect *psEffect, tsColor *psColor, uint16 u16Duration) {
  tsColor sColor = {0.0, 0.0, 0.0};
  vEffect_Gradation(psEffect, &sColor, psColor, u16Duration);
}

/**
 *
 * @param
 */
PUBLIC void vEffect_FadeOut(tsEffect *psEffect, uint16 u16Duration) {
  //vEffect_Gradation(psEffect, tsColor{0.0, 0.0, 0.0}, u16Duration);
}

/**
 *
 * @param
 */
PUBLIC bool_t vEffect_Update(tsEffect *psEffect) {
  bool_t bIsChanging =
      bCompareColor(&(psEffect->color), &(psEffect->targetColor));

  if (bIsChanging) {
    float fPercent = (float)(u32TickCount_ms - psEffect->startEffectTime) /
                     psEffect->u16Duration;
    fPercent = fPercent > 1.0 ? 1.0 : fPercent;

    psEffect->color.fRed =
        psEffect->startColor.fRed +
        (psEffect->targetColor.fRed - psEffect->startColor.fRed) * fPercent;

    psEffect->color.fGreen =
        psEffect->startColor.fGreen +
        (psEffect->targetColor.fGreen - psEffect->startColor.fGreen) * fPercent;

    psEffect->color.fBlue =
        psEffect->startColor.fBlue +
        (psEffect->targetColor.fBlue - psEffect->startColor.fBlue) * fPercent;

    // psEffect->apply();
  }

  return bIsChanging;
}

/**
 *
 * @param
 */
PRIVATE bool_t bCompareColor(tsColor *a, tsColor *b) {
  bool_t bIsMatch = TRUE;

  bIsMatch &= a->fRed == b->fRed;
  bIsMatch &= a->fGreen == b->fGreen;
  bIsMatch &= a->fBlue == b->fBlue;

  return bIsMatch;
}
