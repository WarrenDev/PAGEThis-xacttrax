/*****************************************************************************
* Copyright (c) 2002 Digi International Inc., All Rights Reserved
*
* This software contains proprietary and confidential information of Digi
* International Inc.  By accepting transfer of this copy, Recipient agrees
* to retain this software in confidence, to prevent disclosure to others,
* and to make no use of this software other than that for which it was
* delivered.  This is an unpublished copyrighted work of Digi International
* Inc.  Except as permitted by federal law, 17 USC 117, copying is strictly
* prohibited.
*
* Restricted Rights Legend
*
* Use, duplication, or disclosure by the Government is subject to
* restrictions set forth in sub-paragraph (c)(1)(ii) of The Rights in
* Technical Data and Computer Software clause at DFARS 252.227-7031 or
* subparagraphs (c)(1) and (2) of the Commercial Computer Software -
* Restricted Rights at 48 CFR 52.227-19, as applicable.
*
* Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
*
*****************************************************************************/

/*
 * These macros implement byte-extraction, and are endian-independent:
 *
 *	LOW8(x16)		Get bottom  byte      of 16-bit value
 *	HIGH8(x16)		Get top     byte      of 16-bit value
 *
 *	LOW16(x32)		Get bottom  halfword  of 32-bit value
 *	HIGH16(x32)		Get top     halfword  of 32-bit value
 *
 *	BYTE32_3(x32)		Get highest      byte of 32-bit value
 *	BYTE32_2(x32)		Get next-highest byte of 32-bit value
 *	BYTE32_1(x32)		Get next-lowest  byte of 32-bit value
 *	BYTE32_0(x32)		Get lowest       byte of 32-bit value
 *
 * These macros implement creation of multi-byte values, and are endian-independent:
 *
 *	MAKE16(hi8,lo8)			Make 16-bit value from specified  8-bit values
 *	MAKE32(hi16,lo16)		Make 32-bit value from specified 16-bit values
 *	MAKE64(hi32,lo32)		Make 64-bit value from specified 32-bit values
 *
 *	MAKE32_4(b3, b2, b1, b0)	Make 32-bit value from specified 8-bit values (left-to-right)
 *
 * These macros handle little-endian vs big-endian byte-swapping
 * for 16- and 32-bit quantities. Note that the FROM and TO are
 * really the same thing (ie, they swap if endianness is different)
 * but are useful in documenting the developer's intent:
 *
 *	FROM_LExx		Translates value FROM LittleEndian  format to native (host) format
 *	FROM_BExx		Translates value FROM BigEndian     format to native (host) format
 *	TO_LExx			Translates value from native (host) format TO LittleEndian  format
 *	TO_BExx			Translates value from native (host) format TO BigEndian     format
 *
 * These macros are used to read/write values using unaligned pointers.
 * Typically this is a buffer received from some device (ie,
 * ethernet) at an arbitrary address that may not be aligned.
 * The macros will accept pointers of any type.
 *
 *	Word  LoadNative16(  char[2]        );	Load  NativeEndian 16-bits from unaligned buffer
 *	DWord LoadNative32(  char[4]        );	Load  NativeEndian 32-bits from unaligned buffer
 *	void  StoreNative16( char[2], Word  );	Store NativeEndian 16-bits to   unaligned buffer
 *	void  StoreNative32( char[4], DWord );	Store NativeEndian 32-bits to   unaligned buffer
 *
 *	Word  LoadLE16(      char[2]        );	Load  LittleEndian 16-bits from unaligned buffer
 *	DWord LoadLE32(      char[4]        );	Load  LittleEndian 32-bits from unaligned buffer
 *	void  StoreLE16(     char[2], Word  );	Store LittleEndian 16-bits to   unaligned buffer
 *	void  StoreLE32(     char[4], DWord );	Store LittleEndian 32-bits to   unaligned buffer
 *
 *	Word  LoadBE16(      char[2]        );	Load  BigEndian    16-bits from unaligned buffer
 *	DWord LoadBE32(      char[4]        );	Load  BigEndian    32-bits from unaligned buffer
 *	void  StoreBE16(     char[2], Word  );	Store BigEndian    16-bits to   unaligned buffer
 *	void  StoreBE32(     char[4], DWord );	Store BigEndian    32-bits to   unaligned buffer
 *
 * These macros implement a Little-endian 16-bit wide interface (a 'WE16' type).
 * 8-bit strings need a byte swap.
 * Aligned 16-bit values arrive properly.
 * Unaligned 16-bit values are split between words.
 * 32-bit values need a 32-bit organized 16-bit swap.
 *
 *	FROM_WExx			Translates FROM WE16/32 format to native CPU format
 *	TO_WExx				Translates from native CPU format TO WE16/32 format
 *
 *	void WE16cpy(void *dst, void *src, int n);
 *					Copy n bytes from src -> dst converting string as you go
 *					NOTE: If n is odd, this will write one byte past the end!
 *
 *	Word  LoadWE16( char[2]        );	Load 16-bit WE16 value from unaligned buffer
 *	DWord LoadWE32( char[4]        );	Load 32-bit WE16 value from unaligned buffer
 *	void  StoreWE16(char[2], Word  );	Store native 16-bit value as 16-bit WE16 value in unaligned buffer
 *	void  StoreWE32(char[4], DWord );	Store native 32-bit value as 32-bit WE16 value in unaligned buffer
 */

#ifndef BELE_H_
#define BELE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 *	Endian-independent byte-extraction macros
 */

#define LOW8(x16)           ((unsigned char)(x16))
#define HIGH8(x16)          ((unsigned char)(((unsigned short)(x16)) >> 8))

#define LOW16(x32)          ((unsigned short)(x32))
#define HIGH16(x32)         ((unsigned short)(((unsigned int)(x32)) >> 16))

#define BYTE32_3(x32)       ((unsigned char)(((unsigned long)(x32)) >> 24))
#define BYTE32_2(x32)       ((unsigned char)(((unsigned long)(x32)) >> 16))
#define BYTE32_1(x32)       ((unsigned char)(((unsigned long)(x32)) >> 8))
#define BYTE32_0(x32)       ((unsigned char)((unsigned long)(x32)))

/*
 *	Endian-independent multi-byte-creation macros
 */

#define MAKE16(hi8, lo8)            ((unsigned short)(((unsigned short)(((unsigned char)(hi8)) << 8)) | ((unsigned char)(lo8))))
#define MAKE32(hi16, lo16)          ((unsigned int)(((unsigned int)(((unsigned short)(hi16)) << 16)) | ((unsigned short)(lo16))))
#define MAKE64(hi32, lo32)          ((unsigned long long)(((unsigned long long)(((unsigned int)(hi32)) << 32)) | ((unsigned int)(lo32))))

#define MAKE32_4(b3, b2, b1, b0)    MAKE32(MAKE16(b3, b2), MAKE16(b1, b0))

/*
 *	WE16 load/store macros
 */

#define StoreWE16(array, w)     StoreNative16(array, w)

#define LoadWE16(array)         LoadNative16(array)

#define StoreWE32(array, dw)                                          \
    do { StoreNative16(((unsigned char *)(array)), LOW16(dw));        \
		 StoreNative16(((unsigned char *)(array)) + 2, HIGH16(dw)); } \
    while (0)

#define LoadWE32(array) \
    MAKE32(LoadNative16(((unsigned char)(array)) + 2), LoadNative16(((unsigned char)(array))))

/*
 *	Unaligned access macros
 */

#define LoadLE16(array)    MAKE16(((unsigned char *)(array))[1], ((unsigned char *)(array))[0])
#define LoadLE32(array)                                                    \
    MAKE32_4(((unsigned char *)(array))[3], ((unsigned char *)(array))[2], \
             ((unsigned char *)(array))[1], ((unsigned char *)(array))[0])
#define StoreLE16(array, w)                          \
    do { ((unsigned char *)(array))[0] = LOW8(w);    \
		 ((unsigned char *)(array))[1] = HIGH8(w); } \
    while (0)

#define StoreLE32(array, dw)                             \
    do { ((unsigned char *)(array))[0] = BYTE32_0(dw);   \
		 ((unsigned char *)(array))[1] = BYTE32_1(dw);   \
		 ((unsigned char *)(array))[2] = BYTE32_2(dw);   \
		 ((unsigned char *)(array))[3] = BYTE32_3(dw); } \
    while (0)

#define LoadBE16(array)    MAKE16(((unsigned char *)(array))[0], ((unsigned char *)(array))[1])
#define LoadBE32(array)                                                    \
    MAKE32_4(((unsigned char *)(array))[0], ((unsigned char *)(array))[1], \
             ((unsigned char *)(array))[2], ((unsigned char *)(array))[3])

#define StoreBE16(array, w)                         \
    do { ((unsigned char *)(array))[0] = HIGH8(w);  \
		 ((unsigned char *)(array))[1] = LOW8(w); } \
    while (0)

#define StoreBE32(array, dw)                             \
    do { ((unsigned char *)(array))[0] = BYTE32_3(dw);   \
		 ((unsigned char *)(array))[1] = BYTE32_2(dw);   \
		 ((unsigned char *)(array))[2] = BYTE32_1(dw);   \
		 ((unsigned char *)(array))[3] = BYTE32_0(dw); } \
    while (0)

/*
 * Endian-dependent stuff
 */

// Define swap macros for internal use only. Note that I am purposely
// avoiding defining "SWAP" macros for general use because they require
// that the caller know (or assume) the endianness. It's safer and
// more self-documenting to use the TO/FROM macros instead.
#define bele_SWAP16(x16)    MAKE16(LOW8(x16), HIGH8(x16))
#define bele_SWAP32(x32)    MAKE32(bele_SWAP16(LOW16(x32)), bele_SWAP16(HIGH16(x32)))   /* For ARM use function */

// We're not consistent with how we name this, so check all the variants
#if defined(LITTLE_ENDIAN) || defined(_LITTLE_ENDIAN) || defined(__LITTLE_ENDIAN)

/* These are the definitions for LITTLE_ENDIAN */

#define TO_LE32(x)                  (x)
#define TO_LE16(x)                  (x)
#define FROM_LE32(x)                (x)
#define FROM_LE16(x)                (x)

#define TO_BE32(x)                  (bele_SWAP32(x))
#define TO_BE16(x)                  (bele_SWAP16(x))
#define FROM_BE32(x)                (bele_SWAP32(x))
#define FROM_BE16(x)                (bele_SWAP16(x))

#define TO_WE32(x)                  (x)
#define TO_WE16(x)                  (x)
#define FROM_WE32(x)                (x)
#define FROM_WE16(x)                (x)

// WE16 is by definition little-endian so this is just a copy
#define WE16cpy(dst, src, n)        (memcpy((dst), (src), (n)))

#define LoadNative16(array)         LoadLE16(array)
#define LoadNative32(array)         LoadLE32(array)
#define StoreNative16(array, w)     StoreLE16(array, w)
#define StoreNative32(array, dw)    StoreLE32(array, dw)

#else /* !_LITTLE_ENDIAN */

/* These are the definitions for BIG_ENDIAN */

#define TO_LE32(x)      (bele_SWAP32(x))
#define TO_LE16(x)      (bele_SWAP16(x))
#define FROM_LE32(x)    (bele_SWAP32(x))
#define FROM_LE16(x)    (bele_SWAP16(x))

#define TO_BE32(x)      (x)
#define TO_BE16(x)      (x)
#define FROM_BE32(x)    (x)
#define FROM_BE16(x)    (x)

#define TO_WE32(x)      ((unsigned long)(x) >> 16 | (unsigned long)(x) << 16)
#define TO_WE16(x)      (x)
#define FROM_WE32(x)    ((unsigned long)(x) >> 16 | (unsigned long)(x) << 16)
#define FROM_WE16(x)    (x)

// WE16 is by definition little-endian so call a function for BigEndian
extern void WE16cpy(void *dst, void *src, int n);

#define LoadNative16(array)         LoadBE16(array)
#define LoadNative32(array)         LoadBE32(array)
#define StoreNative16(array, w)     StoreBE16(array, w)
#define StoreNative32(array, dw)    StoreBE32(array, dw)

#endif /* _LITTLE_ENDIAN */

/* This probably isn't the best place for these, but where is? */
/* See NACACHE.C for memory map information. */
/* NOTE: Don't use these macros on ARM chip addresses (0xFFxxxxx)! */
#define UNCACHED(x)     ((DWord)(x) & ~0xFC000000)
#define DCACHED(x)      (((DWord)(x) & ~0xFC000000) | 0x04000000)
#define ICACHED(x)      (((DWord)(x) & ~0xFC000000) | 0x08000000)
#ifdef __cplusplus
}
#endif
#endif /* BELE_H_ */
