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


typedef union
{
    float    f32;
    int      i32;
    uint32_t u32;
    uint16_t u16;
    uint8_t  u8;
} val_type_t;


uint32_t crc32( char* buffer, uint32_t buffer_length );


#endif