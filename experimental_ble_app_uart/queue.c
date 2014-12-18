/*
what does this document use for ?
coding about a circular array
just a little codes

if there is no threads
how to implemented?

*/
#include "app_fifo.h"
#include "queue.h"
#include "string.h"
#include "Hal_defs.h"
#include "app_error.h"

#define HRS_UPQUENE_SIZE		128

app_fifo_t upQueue;
app_fifo_t downQueue;
uint8_t * pQueueMem;
uint8_t writeByte=0;

void QueueInit(void)
{
	pQueueMem=malloc(HRS_UPQUENE_SIZE);
	app_fifo_init(&upQueue,pQueueMem,HRS_UPQUENE_SIZE);//(app_fifo_t * p_fifo, uint8_t * p_buf, uint16_t buf_size)
}

up_queue_data_t upQueueGetStruct(app_fifo_t * p_fifo)
{
//	uint32_t err_code;
	up_queue_data_t queueData;
	uint8_t *pData;
/*	app_fifo_get(p_fifo,pData);		
	queueData.hr=*pData;
	app_fifo_get(p_fifo,pData);	
	queueData.rawd[0]=*pData;
	app_fifo_get(p_fifo,pData);	
	queueData.rawd[1]=*pData;
	app_fifo_get(p_fifo,pData);	
	queueData.rawd[2]=*pData;
	app_fifo_get(p_fifo,pData);	
	queueData.prod=*pData;
*/

	app_fifo_get(p_fifo,&queueData.hr);
	app_fifo_get(p_fifo,&queueData.rawd[0]);	
	app_fifo_get(p_fifo,&queueData.rawd[1]);	
	app_fifo_get(p_fifo,&queueData.rawd[2]);	
	app_fifo_get(p_fifo,&queueData.prod);	
	
	return queueData;
}

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
	up_queue_data_t queueData;
	uint8_t *hrBuf;
	uint8_t rawdBuf[12];
	uint8_t prodBuf[4];
	uint8_t i;
	//check if queue is empty
	qLength=app_fifo_length(&upQueue);
	//if not, send data
	//check if ble is connected, if chars can be notified
	if((qLength>=20)&&(p_nus->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_nus->is_notification_enabled))// 4 groups of data in 100ms
	{
		for(i=0;i<4;i++)
		{
			queueData=upQueueGetStruct(&upQueue);
			*hrBuf=queueData.hr;
			rawdBuf[i+0]=queueData.rawd[0];
			rawdBuf[i+1]=queueData.rawd[1];
			rawdBuf[i+2]=queueData.rawd[2];
			prodBuf[i+0]=queueData.prod;
		}
//		flagBleTxCplt=false;
//		err_code=ble_nus_send_string_byhandle(p_nus, hrBuf, 1,BLE_NUS_HR_CHAR_HANDLE);
//		while(flagBleTxCplt==false)
//		{}
		while(ble_nus_send_string_byhandle(p_nus, hrBuf, 1,BLE_NUS_HR_CHAR_HANDLE)==NRF_ERROR_BUSY)
		{}
//		flagBleTxCplt=false;
//		err_code=ble_nus_send_string_byhandle(p_nus, rawdBuf, sizeof(rawdBuf),BLE_NUS_RAWD_CHAR_HANDLE);
//		while(flagBleTxCplt==false)
//		{}		
		while(ble_nus_send_string_byhandle(p_nus, rawdBuf, sizeof(rawdBuf),BLE_NUS_RAWD_CHAR_HANDLE)==NRF_ERROR_BUSY)
		{}
		
//		flagBleTxCplt=false;
//		err_code=ble_nus_send_string_byhandle(p_nus, prodBuf, sizeof(prodBuf),BLE_NUS_PROD_CHAR_HANDLE);
//		while(flagBleTxCplt==false)
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



/*
#define FIFO_QUEUE_LENGTH	32
uint8_t *gFifoQueue;
uint8_t *pWrite;
uint8_t *pRead;

void InitFifoQueue(void)
{
	memset(gFifoQueue,0,FIFO_QUEUE_LENGTH);
	gFifoQueue=malloc(FIFO_QUEUE_LENGTH);
	pWrite=gFifoQueue;
	pRead=gFifoQueue;	
}

void FifoQueuePut(uint8_t *pData, uint8_t length)
{
	
}*/

















































