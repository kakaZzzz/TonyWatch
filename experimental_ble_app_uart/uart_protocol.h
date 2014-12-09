
#ifndef UART_PROTOCOL_H__
#define UART_PROTOCOL_H__
#include <stdint.h>
#include <stddef.h>


typedef struct
{
	uint16_t length;
	uint8_t *pData;
}package_data_t;

typedef struct
{
	uint8_t *pHeader;
	uint16_t length;
	uint8_t *pData;
	uint8_t *pHip;
	uint8_t checksum;
}frame_data_t;

extern uint8_t frameHeader[];
extern uint8_t frameHip[];
extern uint8_t heartRateDataFrameSample[];
extern uint8_t heartRateDataPackageSample[];
extern uint8_t *pHRD;

frame_data_t *framePackage(package_data_t *packageData);
package_data_t *frameUnPackage(frame_data_t *frameData);

#endif

