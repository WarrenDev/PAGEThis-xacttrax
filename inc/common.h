#ifndef _COMMON_H_
#define _COMMON_H_

#include <adl_global.h>
#include "bele.h"

#if defined(__GNUC__)
#define HINT_FUNC_FORMAT(archetype, format_at, args_at)    __attribute__((format(archetype, format_at, args_at)))
#define HINT_FUNC_DEPRECATED                __attribute__((deprecated))
#define HINT_FUNC_MALLOC_RETURN             __attribute__((malloc))
#define HINT_FUNC_NON_NULL                  __attribute__((nonnull))
#define HINT_FUNC_NON_NULL_ARGS(...)    __attribute__((nonnull(__VA_ARGS__)))
#define HINT_FUNC_NO_RETURN                 __attribute__((noreturn))
#define HINT_FUNC_CONST                     __attribute__((const))
#define HINT_FUNC_PURE                      __attribute__((pure))
#define HINT_FUNC_CALLED_FREQUENTLY         __attribute__((hot))
#define HINT_FUNC_CALLED_INFREQUENTLY       __attribute__((cold))
#define HINT_FUNC_DISCARD_IF_UNUSED         __attribute__((unused))
#define HINT_FUNC_KEEP_IF_UNUSED            __attribute__((used))
#define HINT_FUNC_WARN_IF_RESULT_UNUSED     __attribute__((warn_unused_result))
#define HINT_FUNC_WEAK                      __attribute__((weak))

#define HINT_VAR_DEPRECATED                 __attribute__((deprecated))
#define HINT_VAR_DISCARD_IF_UNUSED          __attribute__((unused))
#define HINT_VAR_KEEP_IF_UNUSED             __attribute__((used))
#define HINT_VAR_WEAK                       __attribute__((weak))
#else
#define HINT_FUNC_FORMAT(archetype, format_at, args_at)
#define HINT_FUNC_DEPRECATED
#define HINT_FUNC_MALLOC_RETURN
#define HINT_FUNC_NON_NULL
#define HINT_FUNC_NON_NULL_ARGS(...)
#define HINT_FUNC_NO_RETURN
#define HINT_FUNC_CONST
#define HINT_FUNC_PURE
#define HINT_FUNC_CALLED_FREQUENTLY
#define HINT_FUNC_CALLED_INFREQUENTLY
#define HINT_FUNC_DISCARD_IF_UNUSED
#define HINT_FUNC_KEEP_IF_UNUSED
#define HINT_FUNC_WARN_IF_RESULT_UNUSED
#define HINT_FUNC_WEAK

#define HINT_VAR_DEPRECATED
#define HINT_VAR_DISCARD_IF_UNUSED
#define HINT_VAR_KEEP_IF_UNUSED
#define HINT_VAR_WEAK
#endif

/* If we don't have packing, at least this will throw an error */
/* TODO: This needs to go away ASAP, packing and unpacking should be done by hand. -ASK */
#define HINT_STRUCT_PACKED    __attribute__((__packed__))

#define VALUE_TO_STRING(value)      # value
#define MACRO_TO_STRING(macro)      VALUE_TO_STRING(macro)

#define _COMBINE(a,b)               a##b
#define COMBINE(a,b)                _COMBINE(a,b)

#define UNUSED_VARIABLE(v)          (void)(v)
#define UNUSED_PARAMETER(p)         UNUSED_VARIABLE(p)

#define MAKE_TAG(b3, b2, b1, b0)    MAKE32_4(b3, b2, b1, b0)

/* INLINING */
/* These are really only for functions defined in header files. */
#define INLINE_PROTOTYPE_(type, name, params)       static inline type name params
#define INLINE_DEFINITION(type, name, params)       INLINE_PROTOTYPE_(type, name, params) HINT_FUNC_DISCARD_IF_UNUSED; INLINE_PROTOTYPE_(type, name, params)

/* BOOLEAN */
/* Because boolean should be the native size of a processor register */
typedef unsigned int bool32_t;

#define IntToBool(value)    !!(value)
#define BoolToInt(value)    ((int)(value))

/* TIME */
#define hours_per_day                   24
#define minutes_per_hour                60
#define seconds_per_minute              60
#define milliseconds_per_second         1000
#define microseconds_per_millisecond    1000

#define microseconds_to_milliseconds(us)    ((us) / microseconds_per_millisecond)
#define seconds_to_milliseconds(s)          ((s) * milliseconds_per_second)
#define minutes_to_milliseconds(m)          seconds_to_milliseconds(minutes_to_seconds(m))

#define minutes_to_seconds(m)               ((m) * seconds_per_minute)
#define hours_to_seconds(h)                 minutes_to_seconds(hours_to_minutes(h))

#define hours_to_minutes(h)                 ((h) * minutes_per_hour)
#define days_to_minutes(d)                  hours_to_minutes(days_to_hours(d))

#define milliseconds_to_ticks(ms)           ADL_TMR_MS_TO_TICK(ms)
#define seconds_to_ticks(s)                 ADL_TMR_S_TO_TICK(s)
#define minutes_to_ticks(m)                 ADL_TMR_MN_TO_TICK(m)
#define hours_to_ticks(h)                   minutes_to_ticks(hours_to_minutes(h))

/* SPACE */
#define bytes_per_kibibyte    1024

#define bytes_to_kibibytes(b)       ((b) / bytes_per_kibibyte)
#define kibibytes_to_bytes(k)       ((k) * bytes_per_kibibyte)

/* MISC */
#define ElementCount(array)         (sizeof array / sizeof array[0])
#define ASCIIZ_strlen(array)        (ElementCount(array) - 1)
#define StringIsEmpty(string)       (string[0] == '\0')

#define CRLF    "\r\n"

/* NUMBERS */
#define round_up(v, m)      ((v) + (-(v) % (m)))
#define round_down(v, m)    ((v) - (+(v) % (m)))

/* For casting when we have to and ONLY when we HAVE to. */

#define RECAST(type, var)       ((type)(var))
#define UNCONST(type, var)      RECAST(type, (var))

/* BITS */
#define BIT(b)                  (1 << (b))
#define BitIsSet(v, b)          IntToBool((v) & BIT(b))
#define BitIsClear(v, b)        IntToBool(~(v) & BIT(b))

#define SetBit(v, b)            (v) |= BIT(b)
#define ClearBit(v, b)          (v) &= ~BIT(b)

#define BIT32(b)                (1 << (b))

#define LOW4(x8)                ((x8) & 0xf)
#define HIGH4(x8)               LOW4((x8) >> 4)

#define MAKE8(h4, l4)           ((LOW4(h4) << 4) | LOW4(l4))

#endif
