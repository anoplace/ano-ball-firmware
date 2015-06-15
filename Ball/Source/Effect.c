#include <jendefs.h>
#include <AppHardwareApi.h>

#include "Effect.h"

PRIVATE bool_t bCompareColor(tsColor *a, tsColor *b);

/**
 *
 * @param
 */
PUBLIC void vEffect_Init(tsEffect *psEffect) {}

/**
 *
 * @param
 */
PUBLIC void vEffect_SetColor(tsEffect *psEffect) {}

/**
 *
 * @param
 */
PUBLIC void vEffect_Gradation(tsEffect *psEffect) {}

/**
 *
 * @param
 */
PUBLIC void vEffect_FadeIn(tsEffect *psEffect) {}

/**
 *
 * @param
 */
PUBLIC void vEffect_FadeOut(tsEffect *psEffect) {}

/**
 *
 * @param
 */
PUBLIC bool_t vEffect_Update(tsEffect *psEffect) {
  bool_t bIsChanging = bCompareColor(&(psEffect->color), &(psEffect->targetColor));

  //  if (bIsChanging) {
  //    float percent = (float)(u32TickCount_ms - psEffect->startEffectTime) /
  //    psEffect->duration;
  //    percent = percent > 1.0 ? 1.0 : percent;
  //
  //    psEffect->color =
  //        Color(psEffect->startColor.red +
  //                  (psEffect->targetColor.red - psEffect->startColor.red) *
  //                  percent,
  //              psEffect->startColor.green +
  //                  (this->targetColor.green - psEffect->startColor.green) *
  //                  percent,
  //              psEffect->startColor.blue +
  //                  (psEffect->targetColor.blue - psEffect->startColor.blue) *
  //                  percent);
  //    psEffect->apply();
  //  }

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
