
// code from http://web.mit.edu/freebsd/head/sys/libkern/crc32.c
// calculates the crc32 of a buffer

#ifndef __CRC32_H__
#define __CRC32_H__

#include <stdint.h>
#include <string.h>
#include <unistd.h>

#define CRC32_SIZE 4

uint32_t crc32(const uint8_t * buf, ssize_t size);

#endif