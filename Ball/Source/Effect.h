#ifndef EFFECT_H_INCLUDED
#define EFFECT_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

typedef struct {
  float fRed;
  float fGreen;
  float fBlue;
} tsColor;

typedef struct {
  tsColor color;
  tsColor startColor;
  tsColor targetColor;
  uint32 startEffectTime;
  uint16 u16Duration;
} tsEffect;

PUBLIC void vEffect_Init(tsEffect *psEffect);
PUBLIC void vEffect_SetColor(tsEffect *psEffect, tsColor sColor);
PUBLIC void vEffect_Gradation(tsEffect *psEffect, tsColor sStartColor,
                              tsColor sTargetColor, uint16 u16Duration);
PUBLIC void vEffect_FadeIn(tsEffect *psEffect, tsColor sColor,
                           uint16 u16Duration);
PUBLIC void vEffect_FadeOut(tsEffect *psEffect, uint16 u16Duration);
PUBLIC bool_t vEffect_Update(tsEffect *psEffect);

#if defined __cplusplus
}
#endif

#endif /* EFFECT_H_INCLUDED */
