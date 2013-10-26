#include "Traces.h"

#include <stdarg.h>
#include <string.h>

#if defined(__DEBUG_APP__)
#undef __DEBUG_APP__
#endif
#if defined(__DEBUG_FULL__)
#undef __DEBUG_FULL__
#endif
#include <adl_global.h>
#include <adl_traces.h>

#include "support.h"

DEBUG_TRACE_STORAGE;

#define GrabAccess()
#define ReleaseAccess()

char const FAIL_STRING[] = "_exit or abort() was called";

#define fail()    adl_errHalt(ADL_ERR_LEVEL_APP + 1, FAIL_STRING)

void abort(void)
{
#if defined(DEBUG)
	breakpoint();
#else
	fail();
#endif
	for ( ; ;)
	{
	}
}


static void trace_output(int const level, char *const message)
{
	char        *storage = message;
	char const  *current;

	ASSERT_GOTO(level >= 1, error);
	ASSERT_GOTO(level <= 32, error);

#if (defined(TRACE_TO_USB) || defined(TRACE_TO_UART))
	if (level == MUTEX_TRACE_LEVEL)
	{
		return;
	}
#endif

	while ((current = strsep(&storage, "\n")) != NULL)
	{
#if defined(TRACE_TO_USB)
		adl_atSendResponsePort(ADL_AT_UNS, ADL_PORT_USB, UNCONST(char *, current));
		adl_atSendResponsePort(ADL_AT_UNS, ADL_PORT_USB, "\r\n");
		adl_ctxSleep(2);        /* TODO: make sure there are no traces in the XBee LED code, or this sleep will cause a deadlock */
#elif defined(TRACE_TO_UART)
		adl_atSendResponsePort(ADL_AT_UNS, ADL_PORT_UART1, UNCONST(char *, current));
		adl_atSendResponsePort(ADL_AT_UNS, ADL_PORT_UART1, "\r\n");
		adl_ctxSleep(2);
#else
		ASSERT_PUTS(strlen(current) <= 255, "String exceeds maximum length specified by ADL");
		adl_trcPrint(level, current);
#endif
	}

error:
	return;
}


static void trace_banner(int const level, char const *const function, char const *const filename, size_t const line)
{
	char *mutable = NULL;
	int const size = asprintf(&mutable, "%s[%s(%zu)]", function, filename, line);

	ASSERT(size > 0);
	ASSERT_GOTO(mutable != NULL, error);

	trace_output(level, mutable);

error:
	if (mutable != NULL)
	{
		free(mutable);
	}
}


void trace_puts_(int const level, char const *const function, char const *const filename, size_t const line, char const *const message)
{
	char *mutable = NULL;

	GrabAccess();

	ASSERT_GOTO(message != NULL, error);

	mutable = strdup(message);
	ASSERT_GOTO(mutable != NULL, error);

	trace_banner(level, function, filename, line);
	trace_output(level, mutable);

error:
	ReleaseAccess();

	if (mutable != NULL)
	{
		free(mutable);
	}
}


void trace_printf_(int const level, char const *const function, char const *const filename, size_t const line, char const *const format, ...)
{
	va_list args;
	char *mutable = NULL;

	GrabAccess();

	ASSERT_GOTO(format != NULL, error);

	va_start(args, format);
	{
		int const size = vasprintf(&mutable, format, args);
		ASSERT(size > 0);
		ASSERT_GOTO(mutable != NULL, error);

		trace_banner(level, function, filename, line);
		trace_output(level, mutable);
	}
	va_end(args);

error:
	ReleaseAccess();

	if (mutable != NULL)
	{
		free(mutable);
	}
}


void trace_hex_dump_(int const level, char const *const function, char const *const filename, size_t const line, void const *const data, size_t const bytes)
{
	if (bytes != 0)
	{
		unsigned char const *byte = data;
		size_t const items_per_line = 32;
		char result[(ASCIIZ_strlen("00 ") * items_per_line) + sizeof ""];
		char                *current = result;
		size_t i;

		GrabAccess();

		trace_banner(level, function, filename, line);

		for (i = 1; i <= bytes; i++)
		{
			int const size = sprintf(current, "%02x ", *byte++);

			ASSERT(size > 0);
			current += size;
			if ((i % items_per_line) == 0)
			{
				trace_output(level, result);
				current = result;
			}
		}

		if (current != result)
		{
			trace_output(level, result);
		}

		ReleaseAccess();
	}
}
