#ifndef __USER_H
#define __USER_H
#include "stm32f4xx.h"
#include <stdio.h>
// -----------------------------------------------------------------------------
// 4.????
// -----------------------------------------------------------------------------


#define LOW_PASS_50HZ_SIZE 5

#define LENS     2800
extern const uint16_t w1[LENS];
extern const uint16_t w2[LENS];
extern const uint16_t w3[LENS];

extern float LED1_DATA[300];
extern float disp_buf[300];
extern float data_buf[300];

void wp (void);

int8_t usr_low_pass_50hz (uint32_t data_r, float *r, uint32_t data_i, float *i);

void usr_low_pass_50hz_init (void);

void data_save (float data);

void data_cpy (void);
void led_init (void);
void debug_led_set (uint8_t led);
void bsp_init (void);
void printf_init (void);
void time_cls (void);
uint32_t time_get (void);
void afe_init (void);
void w_init (void);
//wave
int main_w1(void);
void wave_test (void);
void send_block(char *ch, uint16_t len);
void led_data_in_buf(float data);
void WaveP (void);
void led_cpy (void);

void rest_auto_light (void);

void light_ctrl (uint8_t r, uint8_t i);
int8_t auto_light (uint32_t r, uint32_t i);

//***********************************?????**************************************
#define	 R_L_MAX						2000000UL

#define		R_LIGHT_MAX				2097151UL
#define		I_LIGHT_MAX				2097151UL	

#define		R_LIGHT_3					1600000UL
#define		I_LIGHT_3					1600000UL	

#define		R_LIGHT_2					1500000UL
#define		I_LIGHT_2					1500000UL	

#define		R_LIGHT_1					1400000UL
#define		I_LIGHT_1					1400000UL	

#define		R_LIGHT_0					1300000UL
#define		I_LIGHT_0					1300000UL


#define	WAVE_DETECT_IDLE						0
//??+?y??-????'??
#define	WAVE_DETECT_MAX_DATA_OK			1
//??+?y??-????D???
#define	WAVE_DETECT_MIN_DATA_OK			-1
//PI??Y??-????|
#define	WAVE_DETECT_PI_OK						2


void usr_wave_detect (int32_t signal_r, int32_t *max_r, int32_t *min_r,
			int32_t *max_hr,
			int8_t *flag);

void standard_deviat (int32_t data, int32_t *out_data);

void AFE_READ_DATA (int32_t *a1, int32_t *a2);
void afe_write (uint8_t adr, uint32_t data);


#endif
