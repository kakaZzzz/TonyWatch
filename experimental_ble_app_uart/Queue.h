#ifndef QUEUE_H__
#define QUEUE_H__

#include "ble_nus.h"


typedef struct
{
	uint8_t hr;//heart rate value
//	uint8_t rawd[3];//raw data
	uint8_t prod[10];//process data
}up_queue_data_t;

#define BLE_NUS_HR_CHAR_HANDLE			1
#define BLE_NUS_RAWD_CHAR_HANDLE		2
#define BLE_NUS_PROD_CHAR_HANDLE		3

extern uint8_t flagBleTxCplt;
//extern ble_nus_t   m_nus;

void QueueInit(void);
void upQueueWrite(void);
uint32_t upQueueRead(ble_nus_t * p_nus);

#endif /* QUEUE_H__ */

