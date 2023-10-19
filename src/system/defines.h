#ifndef __DEFINES_H__
#define __DEFINES_H__

// Uncomment for debug messages
//#define __INKPLATE__DEBUG__

// Debug meesage print.
#ifdef __INKPLATE__DEBUG__
#define DEBUG_MGS(X) printf("[IP DEBUG] %s\r\n", X);
#else
#define DEBUG_MGS(X)
#endif

#define BLACK         1
#define WHITE         0

// Different modes for the epaper
#define INKPLATE_1BW    0
#define INKPLATE_GL16   1


#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                                                            \
{                                                                                                                  \
    int16_t t = a;                                                                                                 \
    a = b;                                                                                                         \
    b = t;                                                                                                         \
}
#endif

#endif