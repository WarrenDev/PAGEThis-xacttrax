#ifndef _TRACES_H_
#define _TRACES_H_

#include <stddef.h>
#include "common.h"

typedef enum
{
	PRINTF_TRACE_LEVEL = 1,     /* 1  Trace level for printf statements */
	DEFAULT_TRACE_LEVEL,        /* 2  Trace level for main program traces */
	ERROR_TRACE_LEVEL,          /* 3  Trace level for error traces */
	SPI_FLASH_DEBUG_TRACE,
	SPI_FLASH_ERROR_TRACE,
	SPI_FLASH_STATUS_TRACE,
	I2C_TRACE_LEVEL,
} eTraceLevels;

#define DEFAULT_OUTPUT_TRACE_LEVEL      1
#define HEAP_TRACE_LEVEL                26

void trace_puts_(int const level, char const *const function, char const *const filename, size_t const line, char const *const message);
void trace_printf_(int const level, char const *const function, char const *const filename, size_t const line, char const *const format, ...) HINT_FUNC_FORMAT(printf, 5, 6);
void trace_hex_dump_(int const level, char const *const function, char const *const filename, size_t const line, void const *const data, size_t const bytes);

#define HAVE_TRACING
#if !defined(HAVE_TRACING)
#define RELEASE_TRACE_LEVEL_PUTS(level, message)
#define RELEASE_TRACE_LEVEL_PRINTF(level, format, ...)
#else

/*! @brief Release build trace string with level
 */
#define RELEASE_TRACE_LEVEL_PUTS(level, message)            trace_puts_(level, __func__, filename_, __LINE__, message)

/*! @brief Release build trace formatted string with level
 */
#define RELEASE_TRACE_LEVEL_PRINTF(level, format, ...)      trace_printf_(level, __func__, filename_, __LINE__, format, __VA_ARGS__)
#endif

/*! @brief	Release build trace dump with level
 */
#define RELEASE_TRACE_LEVEL_DUMP(level, data, bytes)    trace_hex_dump_(level, __func__, filename_, __LINE__, data, bytes)

/*! @brief Release build trace string
 */
#define RELEASE_TRACE_PUTS(message)                     RELEASE_TRACE_LEVEL_PUTS(DEFAULT_OUTPUT_TRACE_LEVEL, message)

/*! @brief Release build trace formatted string
 */
#define RELEASE_TRACE_PRINTF(format, ...)               RELEASE_TRACE_LEVEL_PRINTF(DEFAULT_OUTPUT_TRACE_LEVEL, format, __VA_ARGS__)

/*! @brief	Release build trace dump
 */
#define RELEASE_TRACE_DUMP(data, bytes)                 RELEASE_TRACE_LEVEL_DUMP(DEFAULT_OUTPUT_TRACE_LEVEL, data, bytes)

#define ON_FALSE_DO_(cond, what) \
    do { if (!(cond)) { what } \
	}                                                                                  \
    while (0)

#if !defined(FAILURE_MESSAGE_)
#define FAILURE_MESSAGE_(how, cond)                     RELEASE_TRACE_PUTS(how " FAILED: " # cond)
#endif

#if defined(DEBUG)

#if defined(__GNUC__)

/*! @brief	Halt debugger
 *
 *  @note	Only available in debug build
 */
#define breakpoint()    __asm__("bkpt")
#else
#error  "missing code for breakpoint()"
#endif

#define ON_ASSERT_DO_(cond, output, code) \
    ON_FALSE_DO_((cond), { FAILURE_MESSAGE_("ASSERT", cond); output; breakpoint(); code; } \
                 )

/*! @brief	Debug build trace string with level
 *
 *  @note	Trace removed in release build
 */
#define DEBUG_TRACE_LEVEL_PUTS(level, message)          RELEASE_TRACE_LEVEL_PUTS(level, message)

/*! @brief	Debug build trace formatted string with level
 *
 *  @note	Trace removed in release build
 */
#define DEBUG_TRACE_LEVEL_PRINTF(level, format, ...)    RELEASE_TRACE_LEVEL_PRINTF(level, format, __VA_ARGS__)

/*! @brief	Debug build trace string
 *
 *  @note	Trace removed in release build
 */
#define DEBUG_TRACE_PUTS(message)                       RELEASE_TRACE_PUTS(message)

/*! @brief	Debug build trace formatted string
 *
 *  @note	Trace removed in release build
 */
#define DEBUG_TRACE_PRINTF(format, ...)                 RELEASE_TRACE_PRINTF(format, __VA_ARGS__)

/*! @brief	Debug build trace dump with level
 *
 *  @note	Trace removed in release build
 */
#define DEBUG_TRACE_LEVEL_DUMP(level, data, bytes)      RELEASE_TRACE_LEVEL_DUMP(level, data, bytes)

/*! @brief	Debug build trace dump
 *
 *  @note	Trace removed in release build
 */
#define DEBUG_TRACE_DUMP(data, bytes)                   RELEASE_TRACE_DUMP(data, bytes)

#else

#define ON_ASSERT_DO_(cond, output, code) \
    ON_FALSE_DO_((cond), { code; } \
                 )

#define DEBUG_TRACE_LEVEL_PUTS(level, message)
#define DEBUG_TRACE_LEVEL_PRINTF(level, format, ...)

#define DEBUG_TRACE_PUTS(message)
#define DEBUG_TRACE_PRINTF(format, ...)

#define DEBUG_TRACE_LEVEL_DUMP(level, data, bytes)
#define DEBUG_TRACE_DUMP(data, bytes)

#endif

#define ON_ASSERT_PUTS_DO_(cond, pre_bp, post_bp, message) \
    ON_ASSERT_DO_((cond), { DEBUG_TRACE_PUTS(message); pre_bp }, { post_bp } \
                  )
#define ON_ASSERT_PRINTF_DO_(cond, pre_bp, post_bp, format, ...) \
    ON_ASSERT_DO_((cond), { DEBUG_TRACE_PRINTF(format, __VA_ARGS__); pre_bp }, { post_bp } \
                  )

/*! @brief	Assertion of program state
 *
 *  @note	debug: If @a cond is false, output trace of failure and break
 *          release: Nothing
 */
#define ASSERT(cond) \
    ON_ASSERT_DO_((cond), {}, {} \
                  )

/*! @brief	Assertion of program state with message
 *
 *  @note	If @a cond is false, output trace of failure and string then break
 *          release: Nothing
 */
#define ASSERT_PUTS(cond, message)                                  ON_ASSERT_PUTS_DO_((cond), {}, {}, message)

/*! @brief	Assertion of program state with formatted message
 *
 *  @note	If @a cond is false, output trace of failure and formatted string then break
 *          release: Nothing
 */
#define ASSERT_PRINTF(cond, format, ...)                            ON_ASSERT_PRINTF_DO_((cond), {}, {}, format, __VA_ARGS__)

/*! @brief	Assertion of program state with goto on error
 *
 *  @note	debug: If @a cond is false, output trace of failure, break, followed by goto @a label
 *          release: If @a cond is false, goto @a label
 */
#define ASSERT_ERRNO_GOTO(cond, error, label) \
    ON_ASSERT_DO_((cond), {}, { errno = (error); goto label; } \
                  )

/*! @brief	Assertion of program state with goto on error with setting of errno
 *
 *  @note	debug: If @a cond is false, output trace of failure, break, followed by set errno to @a error then goto @a label
 *          release: If @a cond is false, set errno to @a error then goto @a label
 */
#define ASSERT_GOTO(cond, label) \
    ON_ASSERT_DO_((cond), {}, { goto label; } \
                  )

/*! @brief	Assertion of program state with message followed by goto on error
 *
 *  @note	debug: If @a cond is false, output trace of failure and message, break, followed by goto @a label
 *          release: If @a cond is false, goto @a label
 */
#define ASSERT_PUTS_GOTO(cond, label, message)                      ON_ASSERT_PUTS_DO_((cond), {}, { goto label; }, message)

/*! @brief	Assertion of program state with formatted message followed by goto on error
 *
 *  @note	debug: If @a cond is false, output trace of failure and formatted message, break, followed by goto @a label
 *          release: If @a cond is false, goto @a label
 */
#define ASSERT_PRINTF_GOTO(cond, label, format, ...)                ON_ASSERT_PRINTF_DO_((cond), {}, { goto label; }, format, __VA_ARGS__)

/*! @brief	Insistence of program state with message followed by abort
 *
 *  @note	debug: If @a cond is false, output trace of failure, followed by abort()
 *          release: If @a cond is false, abort()
 */
#define INSIST(cond) \
    ON_FALSE_DO_((cond), { FAILURE_MESSAGE_("INSIST", cond); abort(); } \
                 )

/*! @brief	Required by any module calling any trace or assert macro
 */
#define FILENAME_STORAGE_       static char const HINT_VAR_DISCARD_IF_UNUSED filename_[] = __FILE__
#define DEBUG_TRACE_STORAGE     FILENAME_STORAGE_

/* These calls use a two-tier tracing system, we use the primary tag for our level. */
/* They really aren't part of our standard offering. I think only mesh uses them. */
/* TODO: Move somewhere else? -ASK */
#if !defined(HAVE_TRACING)
#define trace_hex_dump(tag, level, ptr, len)
#define trace(tag, level, format, ...)
#else
#define trace_hex_dump(tag, level, ptr, len)    trace_hex_dump_(tag, __func__, __FILE__, __LINE__, ptr, len)
#define trace(tag, level, format, ...)          trace_printf_(tag, __func__, __FILE__, __LINE__, format, ## __VA_ARGS__)
#endif

#if defined(DEBUG)
#define BUILD_TAG(type)                     type ## _TAG

#define VALID_TAGGED_PTR(type, ptr)         (((ptr) != NULL) && ((ptr)->tag == BUILD_TAG(type)))
#define STATIC_ASSIGN_TAG(type)             .tag = BUILD_TAG(type),
#define DYNAMIC_ASSIGN_TAG(ptr, type)       (ptr)->tag = BUILD_TAG(type);
#else
#define VALID_TAGGED_PTR(type, ptr)         ((ptr) != NULL)
#define STATIC_ASSIGN_TAG(type)
#define DYNAMIC_ASSIGN_TAG(ptr, type)
#endif

#endif
