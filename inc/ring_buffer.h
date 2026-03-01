/*
 * Title: ring_buffer.h
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include "lsm6dsm_defs.h"

#define BUFFER_SIZE  IMU_BUFFER_SIZE


typedef struct {
    LSM6DSM_RawData_t buffer[BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile uint32_t overflow_count;
} RingBuffer_t;


// Ring buffer initialization function
void RB_Init(RingBuffer_t* rb);

// Check if ring buffer is full
int RB_IsFull(RingBuffer_t* rb);

// Check if ring buffer is empty
int RB_IsEmpty(RingBuffer_t* rb);

// Write raw data to ring buffer
int RB_Write(RingBuffer_t* rb, LSM6DSM_RawData_t data);

// Read raw data from ring buffer
int RB_Read(RingBuffer_t* rb, LSM6DSM_RawData_t* data);

// Get current data count in ring buffer
uint16_t RB_GetCount(RingBuffer_t* rb);

// Get free space count in ring buffer
uint16_t RB_GetFree(RingBuffer_t* rb);

// Reset ring buffer overflow count
void RB_ClearOverflow(RingBuffer_t* rb);

#endif
