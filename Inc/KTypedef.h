/**********************************************************************
 * Copyright (c) 2004, 
 * All rights reserved.
 * 【文件名称】: KTypedef.h
 * 【主要功能】: 
 * 【内容描述】: 
 * 【作    者】: 
 * 【创建日期】: 2005-4-8 11:30:47
 * 【完成日期】: 
 * 【当前版本】: 1.0
 **********************************************************************/
#ifndef __KTYPEDEF_H__
#define __KTYPEDEF_H__

#include <limits.h>

#ifndef __STDC__ 
#error ERROR: Environment not ISO C conforming.
#endif

#if CHAR_BIT != 8
#error Basic storage unit size must be defined as 8 bits in <limits.h>
#endif

/* Unconditional typedef */ 
typedef char            CHAR;

#if UCHAR_MAX == 255
typedef unsigned char	INT8U;
#endif /* end UCHAR_MAX */ 

#if SCHAR_MAX == 127
typedef signed char		INT8S;
#endif /* end SCHAR_MAX */ 

#if USHRT_MAX == 65535
typedef	unsigned short	INT16U;
#endif

#if SHRT_MAX == 32767
typedef	signed short	INT16S;
#endif

#if UINT_MAX == 0xFFFFFFFFU
typedef	unsigned int	INT32U;
#elif ULONG_MAX == 0xFFFFFFFFU
typedef	unsigned long	INT32U;
#endif

#if INT_MAX == 0x7FFFFFFF
typedef	signed int		INT32S;
#elif LONG_MAX == 0x7FFFFFFF
typedef	signed long		INT32S;
#endif

#if LLONG_MAX == 0x7fffffffffffffffLL
typedef long long INT64S;
#endif

#if ULLONG_MAX == 0xffffffffffffffffULL
typedef unsigned long long INT64U;
#endif

/* 1. data type definitions */
typedef volatile INT8U		VINT8U;      		/* Unsigned volatile  8 bits  */
typedef volatile INT8S		VINT8S;      		/* Signed   volatile  8 bits  */
typedef volatile INT16U		VINT16U;      		/* Unsigned volatile 16 bits  */
typedef volatile INT16S		VINT16S;      		/* Signed   volatile 16 bits  */
typedef volatile INT32U		VINT32U;       		/* Unsigned volatile 32 bits  */
typedef volatile INT32S		VINT32S;       		/* Signed   volatile 32 bits  */
typedef volatile INT64U		VINT64U;      		/* Unsigned volatile 64 bits  */
typedef volatile INT64S		VINT64S;      		/* Signed   volatile 64 bits  */
typedef unsigned int tBoolean;

/* 2. Macro symbol definitions */
#define NULL				0
#define TRUE				(0==0)
#define FALSE				(1==0)
#define true				TRUE
#define false				FALSE

/********************************************************************
 * Global Macros
 *******************************************************************/

/* _BIT(n) sets the bit at position "n"
 * _BIT(n) is intended to be used in "OR" and "AND" expressions:
 * e.g., "(_BIT(3) | _BIT(7))".
 */
#undef _BIT
#define _BIT(n)	(((UINT_32)(1)) << (n))

/* _SBF(f,v) sets the bit field starting at position "f" to value "v".
 * _SBF(f,v) is intended to be used in "OR" and "AND" expressions:
 * e.g., "((_SBF(5,7) | _SBF(12,0xF)) & 0xFFFF)"
 */ 
#undef _SBF
#define _SBF(f,v) (((UINT_32)(v)) << (f))

/* _BITMASK constructs a symbol with 'field_width' least significant
 * bits set.
 * e.g., _BITMASK(5) constructs '0x1F', _BITMASK(16) == 0xFFFF
 * The symbol is intended to be used to limit the bit field width
 * thusly:
 * <a_register> = (any_expression) & _BITMASK(x), where 0 < x <= 32.
 * If "any_expression" results in a value that is larger than can be
 * contained in 'x' bits, the bits above 'x - 1' are masked off.  When
 * used with the _SBF example above, the example would be written:
 * a_reg = ((_SBF(5,7) | _SBF(12,0xF)) & _BITMASK(16))
 * This ensures that the value written to a_reg is no wider than 
 * 16 bits, and makes the code easier to read and understand.
 */ 
#undef _BITMASK
#define _BITMASK(field_width) ( _BIT(field_width) - 1)

/* Generic "error" definitions */ 
#define _NO_ERROR   0
#define _ERROR      -1

/* Generic BOOL types */
typedef INT32S BOOL_32;
typedef INT16S BOOL_16;
typedef INT8S BOOL_8;
#if !defined FALSE
#define FALSE (0==1)
#endif
#if !defined TRUE
#define TRUE (!(FALSE))
#endif

// added by 2004-7-7
#undef NULL
#define NULL 0

#ifndef MAX_VALUE_2
#define MAX_VALUE_2(a, b) ((a > b) ? a : b)
#endif
#ifndef MIN_VALUE_2
#define MIN_VALUE_2(a, b) ((a < b) ? a : b)
#endif
#ifndef MAX_VALUE_3
#define MAX_VALUE_3(a, b, c) ((MAX_VALUE_2(a, b) > c) ? MAX_VALUE_2(a, b) : c)
#endif
#ifndef MIN_VALUE_3
#define MIN_VALUE_3(a, b, c) ((MIN_VALUE_2(a, b) < c) ? MIN_VALUE_2(a, b) : c)
#endif


#ifndef VAR_DEF
#define _EXTERN	extern
#else
#define _EXTERN
#endif

#endif //__KTYPEDEF_H__

