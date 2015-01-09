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

#define HRS_UPQUENE_SIZE		512

app_fifo_t upQueue;
app_fifo_t downQueue;
uint8_t * pQueueMem;
uint8_t writeByte=0;
uint8_t gBleTxBuf[512];
uint32_t gBleTxBufDeepth;

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
/****
@param
sr1  length=25 //srouce data to be depackage
dr_hr  length=2  //destination data of heart rate array to be stored
dr_prod   length 10//destination data of process data array to be stored 
change ASCII into HEX
@return NRF_SUCCESS all done

****/
uint32_t depackageHRD(uint8_t *sr1, uint8_t *dr_hr, uint8_t *dr_prod)
{
	uint8_t i;
	//hr
	dr_hr[0]=(((sr1[0]&0x0f)<<4)&(sr1[1]&0x0f));
	dr_hr[1]=(((sr1[2]&0x0f)<<4)&(sr1[3]&0x0f));
	//prod
	for(i=0;i<10;i++)
	{
		dr_prod[i]=(((sr1[i*2+4]&0x0f)<<4)&(sr1[i*2+5]&0x0f));		
	}
	//check star char
	if(sr1[24]!='*')
	{
		return NRF_ERROR_INVALID_DATA;
	}
	return NRF_SUCCESS;
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

void upQueueRead(void)
{
	uint16_t qLength;
	uint32_t err_code;
	uint8_t dataTemp;
	uint16_t i;
	uint8_t dataView[512];
	qLength=app_fifo_length(&upQueue);
	if(qLength<256)
	{
		memset(gBleTxBuf,0,sizeof(gBleTxBuf));
		for(i=0;i<qLength;i++)
		{
			err_code=app_fifo_get(&upQueue,&gBleTxBuf[i]);
		}
	}
	else if((qLength>256)&&(qLength<=442)) //if qLength >=256 and  <442, code error
	{
		memset(gBleTxBuf,0,sizeof(gBleTxBuf));
		for(i=0;i<qLength;i++)
		{
			err_code=app_fifo_get(&upQueue,&gBleTxBuf[i]);
		}
//		dataTemp=dataView[0];
//		if(dataTemp!=0)
//		{
//			gBleTxBufDeepth=(uint32_t)qLength;
//		}
	}		
	else //if qLength >442, code error
	{
		for(i=0;i<qLength;i++)
		{
			err_code=app_fifo_get(&upQueue,&dataView[i]);
		}
		dataTemp=dataView[0];
		if(dataTemp!=0)
		{
			gBleTxBufDeepth=(uint32_t)qLength;
		}
	}		
	gBleTxBufDeepth=(uint32_t)qLength;
}

uint32_t processDataRxed(ble_nus_t * p_nus)
{
	uint8_t segCnt;
	uint8_t segHipLength;
	uint8_t i,k;
	uint8_t txbuf[20];

	if((p_nus->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_nus->is_notification_enabled))
	{
		//calculate seg cnt and hip length
		segCnt=gBleTxBufDeepth/20;
		if(gBleTxBufDeepth>segCnt*20)
		{
			segHipLength=gBleTxBufDeepth-segCnt*20;
		}
		else
		{
			segHipLength=0;
		}
		//send seg
		for(i=0;i<segCnt;i++)
		{
			for(k=0;k<20;k++)
			{
				txbuf[k]=gBleTxBuf[i*20+k];
			}
			while(ble_nus_send_string_byhandle(p_nus, txbuf, sizeof(txbuf),BLE_NUS_PROD_CHAR_HANDLE)==NRF_ERROR_BUSY)
			{}
		}
		//send hip
		if(segHipLength!=0)
		{
			memset(txbuf,0,sizeof(txbuf));
			for(k=0;k<segHipLength;k++)
			{
				txbuf[k]=gBleTxBuf[segCnt*20+k];
			}
			while(ble_nus_send_string_byhandle(p_nus, txbuf, segHipLength,BLE_NUS_PROD_CHAR_HANDLE)==NRF_ERROR_BUSY)
			{}
		}
	}
	return 1;
}
/*
uint32_t processDataRxed2(ble_nus_t * p_nus)
{
	uint16_t qLength;
	uint32_t err_code=NRF_ERROR_INVALID_LENGTH;
//	up_queue_data_t queueData;
	uint8_t hrBuf[2];
//	uint8_t rawdBuf[12];
	uint8_t prodBuf[10];
	uint8_t queueDataBuf[25];
	uint8_t i;
	uint8_t hrDataTemp;
	
	//check if queue is empty
	qLength=app_fifo_length(&upQueue);
	//if not, send data
	//check if ble is connected, if chars can be notified
	if((qLength>=26)&&(p_nus->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_nus->is_notification_enabled))// 4 groups of data in 100ms
	{
		//find #
		for(i=0;i<qLength;i++)
		{
			err_code=app_fifo_get(&upQueue,&hrDataTemp);
			if(hrDataTemp=='#')
				break;
		}
		if(hrDataTemp!='#')
		{
			return NRF_ERROR_DATA_SIZE;
		}
		//check length
		qLength=app_fifo_length(&upQueue);
		if(qLength<25)//remain qlength<25
		{
			return NRF_ERROR_DATA_SIZE;
		}
		else//remain qLength>=25
		{	
			//read 25 bytes
			for(i=0;i<25;i++)
			{
				err_code=app_fifo_get(&upQueue,&hrDataTemp);
				queueDataBuf[i]=hrDataTemp;
			}
			//depackage  and get heart rate data
			//err_code=depackageHRD(&queueDataBuf[0],&hrBuf[0],&prodBuf[0]);
			//hr
			hrBuf[0]=(((queueDataBuf[0]&0x0f)<<4)|(queueDataBuf[1]&0x0f));
			hrBuf[1]=(((queueDataBuf[2]&0x0f)<<4)|(queueDataBuf[3]&0x0f));
			//prod
			for(i=0;i<10;i++)
			{
				prodBuf[i]=(((queueDataBuf[i*2+4]&0x0f)<<4)|(queueDataBuf[i*2+5]&0x0f));		
			}
			//check star char
			if(queueDataBuf[24]!='*')
			{
				return NRF_ERROR_INVALID_DATA;
			}
			//send
			while(ble_nus_send_string_byhandle(p_nus, hrBuf, sizeof(hrBuf),BLE_NUS_HR_CHAR_HANDLE)==NRF_ERROR_BUSY)
			{}
			while(ble_nus_send_string_byhandle(p_nus, prodBuf, sizeof(prodBuf),BLE_NUS_PROD_CHAR_HANDLE)==NRF_ERROR_BUSY)
			{}
		}
	}
	return err_code;
}

*/
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

















































