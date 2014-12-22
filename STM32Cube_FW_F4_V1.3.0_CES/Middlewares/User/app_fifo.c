/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "app_fifo.h"
#include "nrf_error.h"
#include "app_util.h"

#include "string.h"
#include "Hal_defs.h"
#include "app_error.h"
//#include "ble_nus.h"


app_fifo_t upQueue;
//app_fifo_t downQueue;
uint8_t * pQueueMem;
//uint8_t writeByte=0;

#define FIFO_LENGTH (p_fifo->write_pos - p_fifo->read_pos)  /**< Macro for calculating the FIFO length. */


uint32_t app_fifo_init(app_fifo_t * p_fifo, uint8_t * p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Check that the buffer size is a power of two.
    if (!IS_POWER_OF_TWO(buf_size))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_fifo->p_buf         = p_buf;
    p_fifo->buf_size_mask = buf_size - 1;
    p_fifo->read_pos      = 0;
    p_fifo->write_pos     = 0;

    return NRF_SUCCESS;
}


uint32_t app_fifo_put(app_fifo_t * p_fifo, uint8_t byte)
{
    if (FIFO_LENGTH <= p_fifo->buf_size_mask)
    {
        p_fifo->p_buf[p_fifo->write_pos & p_fifo->buf_size_mask] = byte;
        p_fifo->write_pos++;
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}


uint32_t app_fifo_get(app_fifo_t * p_fifo, uint8_t * p_byte)
{
    if (FIFO_LENGTH != 0)
    {
        *p_byte = p_fifo->p_buf[p_fifo->read_pos & p_fifo->buf_size_mask];
        p_fifo->read_pos++;
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NOT_FOUND;

}

uint32_t app_fifo_flush(app_fifo_t * p_fifo)
{
    p_fifo->read_pos = p_fifo->write_pos;
    return NRF_SUCCESS;
}

uint16_t app_fifo_length(app_fifo_t * p_fifo)
{
	return FIFO_LENGTH;
}

void QueueInit(void)
{
	pQueueMem=malloc(HRS_UPQUENE_SIZE);
	app_fifo_init(&upQueue,pQueueMem,HRS_UPQUENE_SIZE);//(app_fifo_t * p_fifo, uint8_t * p_buf, uint16_t buf_size)
}


up_queue_data_t upQueueGetStruct(app_fifo_t * p_fifo)
{
	up_queue_data_t queueData;
	uint8_t i;

	app_fifo_get(p_fifo,&queueData.hr);
	for(i=0;i<10;i++)
	{
		app_fifo_get(p_fifo,&queueData.prod[i]);	
	}
	return queueData;
}
/*
void upQueueWrite(void)
{
	//if nrf uart get data, then write into upQueue
	//this is done in UART0_IRQHandler(), so here is nothing to do, but we can just simulate

	uint32_t err_code;
	if(app_fifo_length(&upQueue)<=HRS_UPQUENE_SIZE)
	{
		err_code=app_fifo_put(&upQueue,writeByte++);
	}
//	err_code=app_fifo_put(&upQueue,simple_uart_get());
	if(NRF_SUCCESS==err_code)
	{
		APP_ERROR_CHECK(err_code);
	}
}

uint32_t upQueueRead(ble_nus_t * p_nus)
{
	uint16_t qLength;
	uint32_t err_code;
//	up_queue_data_t queueData;
	uint8_t hrBuf[2];
//	uint8_t rawdBuf[12];
	uint8_t prodBuf[10];
	uint8_t i;
	uint8_t *pData;
	//check if queue is empty
	qLength=app_fifo_length(&upQueue);
	//if not, send data
	//check if ble is connected, if chars can be notified
	if((qLength>=11)&&(p_nus->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_nus->is_notification_enabled))// 4 groups of data in 100ms
	{
		//get hr data 2 bytes
		err_code=app_fifo_get(&upQueue,pData);
		hrBuf[0]=*pData;
		err_code=app_fifo_get(&upQueue,pData);
		hrBuf[1]=*pData;
		//get rawd data 10 bytes
		for(i=0;i<10;i++)
		{
			err_code=app_fifo_get(&upQueue,pData);
			prodBuf[i]=*pData;
		}
		while(ble_nus_send_string_byhandle(p_nus, hrBuf, sizeof(hrBuf),BLE_NUS_HR_CHAR_HANDLE)==NRF_ERROR_BUSY)
		{}
//		while(ble_nus_send_string_byhandle(p_nus, rawdBuf, sizeof(rawdBuf),BLE_NUS_RAWD_CHAR_HANDLE)==NRF_ERROR_BUSY)
//		{}
	while(ble_nus_send_string_byhandle(p_nus, prodBuf, sizeof(prodBuf),BLE_NUS_PROD_CHAR_HANDLE)==NRF_ERROR_BUSY)
		{}
	}
	return err_code;
}
void downQueueWrite(void)
{
	
}

void downQueueRead(void)
{
	
}

void upQueueWriteSTMThread(void)
{
	//get data from afe4490
	
	//write data into queue
	
}


void upQueueReadSTMThread(void)
{
	//get data from queue

	//do package

	//if tx not busy, send it to nrf 
	
}
*/



