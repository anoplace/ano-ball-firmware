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

PUBLIC bool_t vEffect_Update(tsEffect *psEffect);

#if defined __cplusplus
}
#endif

#endif /* EFFECT_H_INCLUDED */
