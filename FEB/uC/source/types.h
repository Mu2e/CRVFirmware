#ifndef _TYPE_H_
#define _TYPE_H_
#include  "hal_stdtypes.h"

//typedef code	code_area;

#ifndef NULL
#define NULL		((void *) 0)
#endif

//typedef enum { false, true } bool;

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif

/**
 * The 8-bit signed data type.
 */
typedef char int8;
/**
 * The volatile 8-bit signed data type.
 */
typedef volatile char vint8;
/**
 * The 8-bit unsigned data type.
 */
//typedef unsigned char uint8;
/**
 * The volatile 8-bit unsigned data type.
 */
typedef volatile unsigned char vuint8;

/**
 * The 16-bit signed data type.
 */
typedef short int16;
/**
 * The volatile 16-bit signed data type.
 */
typedef volatile short vint16;
/**
 * The 16-bit unsigned data type.
 */
//typedef unsigned short uint16;
/**
 * The volatile 16-bit unsigned data type.
 */
typedef volatile unsigned short vuint16;
/**
 * The 32-bit signed data type.
 */
typedef long int32;
/**
 * The volatile 32-bit signed data type.
 */
typedef volatile long vint32;
/**
 * The 32-bit unsigned data type.
 */
//typedef unsigned long uint32;  //tek defined in compilers "hal_stdtypes.h"
/**
 * The volatile 32-bit unsigned data type.
 */
typedef volatile unsigned long vuint32;

/* bsd */
typedef uint8			u_char;		/* 8-bit value */
typedef uint8 			SOCKET;
typedef uint16			u_short;	/* 16-bit value */
typedef uint16			u_int;		/* 16-bit value */
typedef uint32			u_long;		/* 32-bit value */

typedef union _un_l2cval {
	u_long	lVal;
	u_char	cVal[4];
}un_l2cval;

typedef union _un_i2cval {
	u_int	iVal;
	u_char	cVal[2];
}un_i2cval;


#endif		/* _TYPE_H_ */
