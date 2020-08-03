#ifndef __buffer_h__
#define __buffer_h__


#include <stdio.h> 
#include <stdlib.h>//malloc

typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;

typedef int ElementType;
typedef struct Buffer * Buff;
struct Buffer{
    ElementType *data;
    uint16_t size;
    uint16_t in;
    uint16_t out;
};

#endif
