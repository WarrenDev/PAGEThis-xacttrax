#ifndef SUPPORT_H
#define SUPPORT_H

#include "adl_global.h"

#ifndef __GNUC__ // ARM Developer Suite
//#define EISDIR 0xf001
#define VA_LIST_IS_ARRAY
#endif

//BIT Set/Clear MACROS
#define SET_MASK(v, mask)       v |= (mask)
#define CLR_MASK(v, mask)       v &= ~(mask)
#define SET_BIT(v, bit)         SET_MASK((v), (1 << (bit)))
#define CLR_BIT(v, bit)         CLR_MASK((v), (1 << (bit)))
#define TEST_MASK(v, mask)      ((v) & (mask))
#define TEST_BIT(v, bit)        TEST_MASK((v), (1 << (bit)))

#endif
