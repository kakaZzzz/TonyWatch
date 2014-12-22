#ifndef __BLE_NUS51422_H
#define __BLE_NUS51422_H

#include "stdint.h"
#include "cmsis_os.h"
#ifdef __cplusplus
 extern "C" {
#endif 
	 
#define TX_BUF_DATA_LENGTH 37

#define BLE_STATUS_DISCONNECTED   0
#define BLE_STATUS_CONNECTED           1

#define HR_PACK_LENGTH		129
//extern ITStatus UartTxReady;
extern uint8_t flagBleConStatus;
extern  void TxSampleSend(uint8_t *pData, uint16_t length);
//extern osSemaphoreId xBinSemaHeartRateSend;
extern uint8_t gHeartRateTxPackBuf[HR_PACK_LENGTH];
#ifdef __cplusplus
}
#endif

#endif /* __BLE_NUS51422_H */

