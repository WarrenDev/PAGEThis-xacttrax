#ifndef _ADL_SHORING_H_
#define _ADL_SHORING_H_

#include "common.h"

#define _ADL_ARG_MAX(n)         (((unsigned)(n) > 15) ? 15 : (n))
#define ADL_CMD_MIN_ARGS(n)     (_ADL_ARG_MAX(n) << 0)
#define ADL_CMD_MAX_ARGS(n)     (_ADL_ARG_MAX(n) << 4)

/* Some ADL function parameters are just dumb */

#define ADL_P2U8P(var)      RECAST(u8 *, (var))

#define ADL_CCP2U8P(var)    ADL_P2U8P(UNCONST(char *, (var)))
#define ADL_CCP2CP(var)     UNCONST(char *, (var))

#define ADL_CVP2U8P(var)    ADL_P2U8P(UNCONST(void *, (var)))

/* widened versions of the ADL types so we can pass them around without overhead */

typedef s32 ws8;
typedef s32 ws16;

typedef u32 wu8;
typedef u32 wu16;

#define widen_s8(n)     ((ws8)(n))
#define widen_s16(n)    ((ws16)(n))

#define widen_u8(n)     ((wu8)(n))
#define widen_u16(n)    ((wu16)(n))

#endif
