/**
 * @file utils.h 
 * @author Carlos Duarte (carlosduarte.molina97@gmail.com)
 * @brief This header serves as a toolbox containing useful functions and macro definition
 * @version 0.1
 * @date 2025-01-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef UTILS_H
#define UTILS_H


#include <stdint.h>
#include <sys/time.h>


/* Scenario: Enable Counter */
#define TCPWM_MICROSECONDS   (0UL)
#define TCPWM_SECONDS        (1UL)

#define UNIX_OFFSET          (2208988800UL)
#define FRACTION_MAX         (4294967296ULL)

#define TVTOTS(seconds, fraction, tv) \
    seconds = tv.tv_sec + UNIX_OFFSET; \
    fraction = (uint32_t)(((uint64_t)(tv.tv_usec * 4294967296)) / 1000000.0);


typedef union
{
    float    f32;
    int      i32;
    uint32_t u32;
    uint16_t u16;
    uint8_t  u8;
} val_type_t;

enum {
    CLOCK_STEP,
    CLOCK_SLEW
};


uint32_t crc32( char* buffer, uint32_t buffer_length );
void xGetTimeTV( struct timeval* tv );

void adjustTimer( int32_t seconds_offset, int32_t fraction_offset, uint8_t mode );

#endif