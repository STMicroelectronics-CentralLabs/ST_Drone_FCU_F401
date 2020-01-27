/*This is system type define.
File name : def32.h
All rights reserved,if the code is not authorized by STMicroelectronics.
----by tom.xiao
E-mail:tom.xiao@ST.com
2013-4-10 11:09:45
*/


#ifndef         __def32
#define         __def32

#include        <stdint.h>

#ifndef         U8
typedef        unsigned char     U8;
#endif

#ifndef         S8
typedef        signed char       S8;
#endif

#ifndef         U16
typedef        unsigned short    U16;
#endif

#ifndef         S16
typedef        signed short      S16;
#endif

#ifndef         U32
typedef        unsigned int      U32;
#endif

#ifndef         S32
typedef        signed int        S32;
#endif

#ifndef         F32
typedef        float             F32;
#endif

#ifndef         U64
typedef        unsigned long long              U64;
#endif

#ifndef         S64
typedef        signed long long                S64;
#endif

#ifndef         D64
typedef        double                          D64;
#endif

/*8 bits devided 8 1 bit;
*/
typedef struct
{ uint8_t   bit0:1;
  uint8_t   bit1:1;
  uint8_t   bit2:1;
  uint8_t   bit3:1;
  uint8_t   bit4:1;
  uint8_t   bit5:1;
  uint8_t   bit6:1;
  uint8_t   bit7:1;
}bits8;

typedef union
{ uint8_t all;
  bits8   flag;
}u8_bit;

/*16 bits devided 16 1 bit;
*/
typedef struct
{ uint8_t   bit0:1;
  uint8_t   bit1:1;
  uint8_t   bit2:1;
  uint8_t   bit3:1;
  uint8_t   bit4:1;
  uint8_t   bit5:1;
  uint8_t   bit6:1;
  uint8_t   bit7:1;
  uint8_t   bit8:1;
  uint8_t   bit9:1;
  uint8_t   bit10:1;
  uint8_t   bit11:1;
  uint8_t   bit12:1;
  uint8_t   bit13:1;
  uint8_t   bit14:1;
  uint8_t   bit15:1;
}bits16;

typedef union
{ uint16_t all;
  bits16   flag;
}u16_bit;

/* An uint16_t devided 2 uint8_t-bits parts.
*/
typedef struct
{ uint8_t low;
  uint8_t high;
}u8u8;

typedef union
{ uint16_t  all;
  u8u8      high_low;
}u16_u8u8;

/* A signed int16 is devided as a sigened int8_t(high part) and an uint8_t(low part).
*/
typedef struct
{ uint8_t low;
  int8_t  shigh;
}s8u8;

typedef union
{ int16_t all;
  s8u8    high_low;
}s16_s8u8;

/* A uint32_t is devided as 2 uint16_t parts.
*/
typedef struct
{ uint16_t  low;
  uint16_t  high;
}u16u16;

typedef union
{ uint32_t  all;
  u16u16    high_low;
}u32_u16u16;

/* A uint32_t is devided as 4 uint8_t parts.
*/
typedef struct
{ uint8_t  byte0;
  uint8_t  byte1;
  uint8_t  byte2;
  uint8_t  byte3;
}u8u8u8u8;

typedef union
{ uint32_t  all;
  u8u8u8u8  bytes;
}u32_u8u8u8u8;

/* A signed int32_t is devided as a sigened int16_t(high part) and an uint16_t(low part).
*/
typedef struct
{ uint16_t  low;
  int16_t   shigh;
}s16u16;

typedef union
{ int32_t all;
  s16u16  high_low;
}s32_s16u16;


#endif /*__DEF_H__*/


/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

