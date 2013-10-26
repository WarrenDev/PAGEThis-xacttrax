#ifndef PISTD_H
#define PISTD_H

/*-----------------------------------------------------------------------------
 *   INCLUDES
 *  -----------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
 *  TYPE DEFINITIONS
 *  --------------------------------------------------------------------------*/

/* current standard Paragon portability types (October 2001) */
typedef signed char SINT8;              /* signed 8-bit integer */
typedef signed short SINT16;            /* signed 16-bit integer */
typedef signed long SINT32;             /* signed 32-bit integer */

typedef unsigned char UINT8;            /* unsigned 8-bit integer */
typedef unsigned short UINT16;          /* unsigned 16-bit integer */
typedef unsigned long UINT32;           /* unsigned 32-bit integer */

typedef volatile UINT8 UINT8V;          /* volatile UINT8 */
typedef volatile UINT16 UINT16V;        /* volatile UINT16 */
typedef volatile UINT32 UINT32V;        /* volatile UINT32 */

typedef volatile SINT8 SINT8V;          /* volatile SINT8 */
typedef volatile SINT16 SINT16V;        /* volatile SINT16 */
typedef volatile SINT32 SINT32V;        /* volatile SINT32 */

typedef unsigned int boolean;

typedef enum
{
	TRUE1 = 1,          //TRUE is already defined in WIP libraries
	FALSE1 = 0,
	ON = 1,
	OFF = 0,
	YES = 1,
	NO = 0,
	PASS = 1,
	FAIL = 0
} BOOL;

/*--------------------------------------------------------------------------
 *  CONSTANT & MACRO DEFINITIONS
 *  --------------------------------------------------------------------------*/

#ifndef  NULL
#define  NULL       0
#endif /* ifndef NULL */

#define  NULLPTR    ((void *)0)
#define  NIL        (~(word)0)
#define  NILPTR     ((void *)(~(long)0))

#define  FOREVER    for ( ; ;)

#define  MAX(a, b)      (((a) > (b)) ? (a) : (b))
#define  MIN(a, b)      (((a) > (b)) ? (b) : (a))

#define  KB     1024UL
#define  MB     (KB * KB)

#define  KHZ    1000UL
#define  MHZ    1000000UL

/* new extraction/combination macros */
#define  COMB_8_16(msB, lsB)        (UINT16)((((UINT16)((UINT8)(msB))) << 8) | (UINT16)((UINT8)(lsB)))
#define  COMB_16_32(msW, lsW)       (UINT32)((((UINT32)((UINT16)(msW))) << 16) | (UINT32)((UINT16)(lsW)))

//#define  LOW8(w)              (UINT8)( (UINT16)(w) )
//#define  HIGH8(w)             (UINT8)( (UINT16)(w) >> 8 )

//#define  LOW16(l)             (UINT16)( (UINT32)(l) )
//#define  HIGH16(l)            (UINT16)( (UINT32)(l) >> 16 )

/* Return offset of element in structure */
#define  FLD_DISP(r, f)    ((word)(&((r *)0)->f))

#endif   /* ifndef PISTD_H */
