
#define BLE_UUID_SERVICE1            0x2500                      
#define BLE_UUID_SERVICE1_CHARACTERISTIC1  0x2501
#define BLE_UUID_SERVICE1_CHARACTERISTIC2  0x2502 

#define BLE_UUID_SERVICE2            0x2600                      
#define BLE_UUID_SERVICE2_CHARACTERISTIC1  0x2601
#define BLE_UUID_SERVICE2_CHARACTERISTIC2  0x2602    

#define BLE_UUID_SERVICE3            0x2700                      
#define BLE_UUID_SERVICE3_CHARACTERISTIC1  0x2701
#define BLE_UUID_SERVICE3_CHARACTERISTIC2  0x2702        

//char_data_t.state enum
#define BLE_CHAR_STATE_DEFAULT 0
#define BLE_CHAR_STATE_PENDING_UPDATE_TOPEER 1
#define BLE_CHAR_STATE_PENDING_UPDATE_TOCPU 2
#define BLE_CHAR_STATE_UPDATED 3

#define BLE_CHAR_PACKTYPE_DONE 0
#define BLE_CHAR_PACKTYPE_PENDING 1


uint8_t bleService1;
uint8_t bleService1characteristic1;
uint8_t bleService1characteristic2;
uint8_t bleService2;
uint8_t bleService2characteristic1;
uint8_t bleService2characteristic2;
uint8_t bleService3;
uint8_t bleService3characteristic1;
uint8_t bleService3characteristic2;

typedef struct
{
	uint8 *pData;
	uint16 length;
	uint8 state=BLE_CHAR_STATE_DEFAULT;
	uint8 packtype=BLE_CHAR_PACKTYPE_DONE;
}char_data_t;

char_data_t *timeSyncChar_s;
char_data_t *heartRateRawData_s;
