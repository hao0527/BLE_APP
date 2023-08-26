
#ifndef _ARM_COMMON_TABLES_H
#define _ARM_COMMON_TABLES_H

#include "arm_math.h"

extern const uint16_t armBitRevTable[1024];
extern const q15_t armRecipTableQ15[64];
extern const q31_t armRecipTableQ31[64];
extern const q31_t realCoefAQ31[1024];
extern const q31_t realCoefBQ31[1024];
extern const float32_t twiddleCoef[6144];
extern const q31_t twiddleCoefQ31[6144];
extern const q15_t twiddleCoefQ15[6144];

#endif /*  ARM_COMMON_TABLES_H */
