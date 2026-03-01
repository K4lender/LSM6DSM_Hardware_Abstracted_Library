/*
 * Title: ring_buffer.c
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#include "ring_buffer.h"
#include <string.h>

void RB_Init(RingBuffer_t* rb) {
	if (rb == NULL) return;
    
    memset(rb->buffer, 0, sizeof(rb->buffer));

	rb->head = 0;
	rb->tail = 0;
	rb->count = 0;
	rb->overflow_count = 0;
}

int RB_IsFull(RingBuffer_t* rb) {
	return rb->count == BUFFER_SIZE;
}

int RB_IsEmpty(RingBuffer_t* rb) {
	return rb->count == 0;
}

int RB_Write(RingBuffer_t* rb, LSM6DSM_RawData_t data) {

	if (rb == NULL) return -1;

	if (RB_IsFull(rb)) {
		rb->overflow_count++;
		rb->tail = (rb->tail + 1) & (BUFFER_SIZE - 1);
	}
	else {
		rb->count++;
	}

	rb->buffer[rb->head] = data;
	rb->head = (rb->head + 1) & (BUFFER_SIZE - 1);
	return 0;
}

int RB_Read(RingBuffer_t* rb, LSM6DSM_RawData_t* data) {
	
	if (rb == NULL || data == NULL || RB_IsEmpty(rb)) {
		return -1;
	}

	*data = rb->buffer[rb->tail];
	rb->tail = (rb->tail + 1) & (BUFFER_SIZE - 1);
	rb->count--;
	return 0;
}

uint16_t RB_GetCount(RingBuffer_t* rb) {
    return (rb != NULL) ? rb->count : 0;
}


uint16_t RB_GetFree(RingBuffer_t* rb) {
    return (rb != NULL) ? (BUFFER_SIZE - rb->count) : 0;
}


void RB_ClearOverflow(RingBuffer_t* rb) {
    if (rb != NULL) rb->overflow_count = 0;
}
