#ifndef PVG_DEFS_H
#define PVG8_DEFS_H

#ifndef TRUE
    #define TRUE    1
#endif
#ifndef FALSE
    #define FALSE   0
#endif
#ifndef NULL
    #define NULL   0
#endif
typedef unsigned char       BYTE;
typedef unsigned char       BOOLEAN;
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef signed short        INT16;
typedef unsigned long       UINT32;
typedef signed long         INT32;
typedef signed char		    INT8;


//----------------------------------------------------
// ENUMS
//----------------------------------------------------

#ifndef MAX
    #define MAX(A,B) ((A)>(B) ? (A):(B))
#endif
#ifndef MIN
    #define MIN(A,B) ((A)<(B) ? (A):(B))
#endif

#ifndef ABS
    #define  ABS(A)  (((A) < 0) ? -(A) : (A))
#endif

#ifndef SIGN
    #define SIGN(A)     (((A)<0) ? (-1) : (1))
#endif


#endif      // DEFS_H

