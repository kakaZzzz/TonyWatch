/**
  ******************************************************************************
  * @file    AFE4490.c
  * @author  ADDING Team
  * @version V1.0.0
  * @date    26-11-2014
  * @brief   This file provides the AFE4490 Codec driver.   
  ******************************************************************************/
#include "AFE4490.h"
/*************************************/
#include "stdio.h"
#include "math.h"
#include "user.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4_tony.h"
#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"

/***************************************/
// -----------------------------------------------------------------------------
#define FILTER_ARRAY_SIZE    18          // ??????????
#define ROW_NUM              300       // ???????
#define COL_NUM              4          // ???????
#define LEVEL_NUM            9          // ???????
///////**************************************************/

#define ENABLE_SELECT()              HAL_GPIO_WritePin(AFE44XX_CS_GPIO, AFE44XX_CS_PIN,GPIO_PIN_RESET) 
#define DISABLE_SELECT()              HAL_GPIO_WritePin(AFE44XX_CS_GPIO, AFE44XX_CS_PIN,GPIO_PIN_SET)

#define ENABLE_PDN()              HAL_GPIO_WritePin(AFE44XX_PDN_GPIO, AFE44XX_PDN_PIN,GPIO_PIN_RESET) 
#define DISABLE_PDN()              HAL_GPIO_WritePin(AFE44XX_PDN_GPIO, AFE44XX_PDN_PIN,GPIO_PIN_SET)

#define ENABLE_RESET()              HAL_GPIO_WritePin(AFE44XX_RESET_GPIO, AFE44XX_RESET_PIN,GPIO_PIN_RESET) 
#define DISABLE_RESET()              HAL_GPIO_WritePin(AFE44XX_RESET_GPIO, AFE44XX_RESET_PIN,GPIO_PIN_SET)

#define ENABLE_AFEXXEN()              HAL_GPIO_WritePin(AFE44XX_EN_GPIO, AFE44XX_EN_PIN,GPIO_PIN_RESET) 
#define DISABLE_AFEXXEN()              HAL_GPIO_WritePin(AFE44XX_EN_GPIO, AFE44XX_EN_PIN,GPIO_PIN_SET)


#define Len_Mem 300
//////////////////////////////////////////////////////////////////////////
#define FilterCofdepth		11
#define SlideLenght   100
#define mmaDataACnt		SlideLenght*3
#define timeWindowCountMax 144
#define timeWindowCountMin  36
#define DYNAMIC_PRECISE  10
unsigned char filterCof[6] =  {14,30,73,127,171,188}; 
unsigned long mmaDataFilterPredata[FilterCofdepth-1] = {165590,165440,165332,165358,165395,165284,165287,165257,165262,165251};
/*********************************************************************
  ÄÚ²¿±äÁ¿
*********************************************************************/
uint32_t   AFE4400_Diag;
int32_t    AFE4400_data;
uint8_t    FingerOutCheckDelay;
uint8_t    AFE4400_AdcRdy;
unsigned long mmaDataFilterTemp[mmaDataACnt+FilterCofdepth-1];
unsigned long mmaDataFilter[mmaDataACnt];
unsigned long xyzAxisAbsdataNew[mmaDataACnt];
unsigned long xyzAxisAbsdataOld[mmaDataACnt];
//unsigned char KeyFingerOut; //ÊÖÖ¸ÍÑÂä±êÖ¾
unsigned char oldKeyFingerOut; //ÊÖÖ¸ÍÑÂä±êÖ¾,¾ÉµÄ
unsigned int  IR_T; //Ê±¼ä¼ÆÊýÏà¹ØÁ¿
unsigned int  RD_T; //
unsigned int  index_Mem_in =0;
unsigned char index_Mem_out;
unsigned long Temp_Time2;
unsigned long Mem_IRVP[ Len_Mem*2 ]; 
unsigned long Mem_RDVP[ Len_Mem*2 ];
unsigned long Mem_RDV0[ Len_Mem*2 ];
long gIRVP; // ºìÍâ¹âÇ¿
long gRDVP; //ºì¹â¹âÇ¿
float gRDVP_gIRVP; //ºì¹â¹âÇ¿
long gRDV0; //
uint8_t   FingerOut_Timer;
uint32_t   AFE44xx_Diag;


uint8_t HRCountFIFOAdd = 0;
uint8_t HRPerMinTemp = 0;
unsigned long HRPerMin = 0;
unsigned long uartCount1 = 0;
unsigned long uartCount2 = 100;
unsigned long uartCount3 = 1000;
unsigned long uartCount4 = 10000;
unsigned long HRCount = 0;
unsigned long HRCountFIFO[20];
unsigned long mmaDataA[Len_Mem];
unsigned int  HeartRatio_RealTimeVaule = 0;
unsigned int  HeartRatio_RealTimeVaule_FIFO[256];
unsigned int  HeartRatio_RealTimeVaule_FIFOaddr = 0;
unsigned int  HeartRatio_RealTimeVauleTemp = 0;
unsigned int  timeWindowCount = 0;

uint8_t AFE44xx_ADRDY = 0;
uint8_t aTxBuffer_test1[4] = {0,0,0,0};
char aTxBuffer_temp[25] = "4294967295\n4294967295\n";
/*********************************************************************/
//ÄÚ²¿º¯Êý
/*********************************************************************/
/********************************************************************/
//wave process
/*****************************************************************/
// -----------------------------------------------------------------------------
// 3.????
// -----------------------------------------------------------------------------

// ?????,?????????
typedef signed   char       INT8S;      // 8??????
typedef unsigned char       INT8U;      // 8??????  
typedef signed short int    INT16S;     // 16??????
typedef unsigned short int  INT16U;     // 16??????
typedef signed   long       INT32S;     // 32??????
typedef unsigned long       INT32U;     // 32??????
//typedef unsigned long long INT64U;       // ????64??

typedef signed   char       int8;       // 8??????
typedef unsigned char       uint8;      // 8??????  
typedef signed short int    int16;      // 16??????
typedef unsigned short int  uint16;     // 16??????
typedef signed   long       int32;      // 32??????
typedef unsigned long       uint32;     // 32??????
//typedef unsigned long long uint64;      // ????64??



// -----------------------------------------------------------------------------
// 5.???
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// 6.????
// -----------------------------------------------------------------------------
float Lp_D[FILTER_ARRAY_SIZE] = {
0.0000393473199950261000,
-0.0002519631889981790000,
0.0002303857639954130000,
0.0018476468829611300000,
-0.0042815036819047200000,
-0.0047232047578948300000,
0.0223616621235152000000,
0.0002509471149919380000,
-0.0676328290595240000000,
0.0307256814783229000000,
0.1485407493347600000000,
-0.0968407832208790000000,
-0.2932737832725870000000,
0.1331973858220890000000,
0.6572880780366390000000,
0.6048231236767790000000,
0.2438346746376670000000,
0.0380779473631673000000 
};

// ??????????
float Hp_D[FILTER_ARRAY_SIZE] = {
-0.0380779473631673000000 	,
0.2438346746376670000000 	,
-0.6048231236767790000000 	,
0.6572880780366390000000 	,
-0.1331973858220890000000 	,
-0.2932737832725870000000 	,
0.0968407832208790000000 	,
0.1485407493347600000000 	,
-0.0307256814783229000000 	,
-0.0676328290595240000000 	,
-0.0002509471149919380000 	,
0.0223616621235152000000 	,
0.0047232047578948300000 	,
-0.0042815036819047200000 	,
-0.0018476468829611300000 	,
0.0002303857639954130000 	,
0.0002519631889981790000 	,
0.0000393473199950261000 
};

// ??????????
float Lp_R[FILTER_ARRAY_SIZE] = {
0.0380779473631673000000,
 	0.2438346746376670000000 	,
	0.6048231236767790000000 	,
	0.6572880780366390000000 	,
	0.1331973858220890000000 	,
	-0.2932737832725870000000 	,
	-0.0968407832208790000000 	,
	0.1485407493347600000000 	,
	0.0307256814783229000000 	,
	-0.0676328290595240000000 	,
	0.0002509471149919380000 	,
	0.0223616621235152000000 	,
	-0.0047232047578948300000 	,
	-0.0042815036819047200000 	,
	0.0018476468829611300000 	,
	0.0002303857639954130000 	,
	-0.0002519631889981790000 	,
	0.0000393473199950261000 
};

// ??????????
float Hp_R[FILTER_ARRAY_SIZE] = {
0.0000393473199950261000 	,
0.0002519631889981790000 	,
0.0002303857639954130000 	,
-0.0018476468829611300000 	,
-0.0042815036819047200000 	,
0.0047232047578948300000 	,
0.0223616621235152000000 	,
-0.0002509471149919380000 ,
-0.0676328290595240000000 	,
-0.0307256814783229000000 	,
0.1485407493347600000000 	,
0.0968407832208790000000 	,
-0.2932737832725870000000 	,
-0.1331973858220890000000 	,
0.6572880780366390000000 	,
-0.6048231236767790000000 	,
0.2438346746376670000000 	,
-0.0380779473631673000000 
};


FILE *InFile;                           // ??????
FILE *OutFile;                          // ??????
int nLine;                              // ????????
//float LED1_DATA[ROW_NUM];              // ????????????,????
//float LED1_DATA1[ROW_NUM];              // ????????????,????
int DecLevel = 9;                           // ???????
int SampleRate = 100;                         // ??? 
int Amount_Wavelet_Remain;              // ???????
int Period_Sample_Window = 300;               // ?????? 
int Tmp_PSW;
int Length_A = 448;
int DecLevel_Counter;
int A_next;
// 300
int L[LEVEL_NUM+2] = {17,17,18,19,21,25,34,52,87,158,300};

float C[500];
float C_rec[500];


/*********************************************************************/
extern void FingerOutCheck(void);
extern void ProbOutCheck(void);
uint32_t  AFE44xx_Processor( void );
/*********************************************************************
  Íâ²¿º¯Êý
*********************************************************************/
extern  void  DelayMS( uint32_t MS );
extern  void  DelayuS( uint32_t uS );
unsigned char heartRateDataProcess(unsigned long *mmaDataA);
void WaveProcess(float *Input,float *Output);
extern uint8_t AFE44xx_ADRDY;
//extern long* AFE44xx_RecData(void);
/**********************************************************************/
static void AFE44xx_gpio_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
	
  AFE44XX_SPI_CLK_ENABLE();
	
	  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* SPI SCK GPIO pin configuration  */
	AFE44XX_SCK_CLK_ENABLE();  //must put front of gpio_init
	GPIO_InitStruct.Pin       = AFE44XX_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = AFE44XX_GPIO_AF;
  HAL_GPIO_Init(AFE44XX_SCK_GPIO, &GPIO_InitStruct);
	  
  /* SPI MISO GPIO pin configuration  */
	AFE44XX_MISO_CLK_ENABLE();//must put front of gpio_init
  GPIO_InitStruct.Pin       = AFE44XX_MISO_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = AFE44XX_GPIO_AF;
  HAL_GPIO_Init(AFE44XX_MISO_GPIO, &GPIO_InitStruct);
	 
  /* SPI MOSI GPIO pin configuration  */
	AFE44XX_MOSI_CLK_ENABLE();//must put front of gpio_init
  GPIO_InitStruct.Pin       = AFE44XX_MOSI_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = AFE44XX_GPIO_AF;  
  HAL_GPIO_Init(AFE44XX_MOSI_GPIO, &GPIO_InitStruct);	

	/* Configure the cs pin */
	AFE44XX_CS_CLK_ENABLE();//must put front of gpio_init
  GPIO_InitStruct.Pin = AFE44XX_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;  
  HAL_GPIO_Init(AFE44XX_CS_GPIO, &GPIO_InitStruct);

	/* Configure the RESET pin */
	AFE44XX_RESET_CLK_ENABLE();//must put front of gpio_init
  GPIO_InitStruct.Pin = AFE44XX_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AFE44XX_RESET_GPIO, &GPIO_InitStruct);

	/* Configure the PDN pin */
	AFE44XX_PDN_CLK_ENABLE();   //must put front of gpio_init
  GPIO_InitStruct.Pin = AFE44XX_PDN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AFE44XX_PDN_GPIO, &GPIO_InitStruct);
	
	AFE44XX_ADRDY_CLK_ENABLE();//must put front of gpio_init
  GPIO_InitStruct.Pin = AFE44XX_ADRDY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AFE44XX_ADRDY_GPIO, &GPIO_InitStruct); 

	/* Configure the EN pin */
	AFE44XX_EN_CLK_ENABLE();//must put front of gpio_init
  GPIO_InitStruct.Pin = AFE44XX_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AFE44XX_EN_GPIO, &GPIO_InitStruct);
}

static void AFE44xx_spi_init(void)
{
	SPI_HandleTypeDef SpiHandle;
  
  AFE44XX_SPI_CLK_ENABLE();
	
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = AFE44XX_SPI;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;	
	SpiHandle.Init.Mode              = SPI_MODE_MASTER;
   if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  } 
        /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(& SpiHandle);	
}

/*********************************************************************

*********************************************************************/
uint8_t  Spi2WriteByte( uint8_t   data )
{
	   while (!(AFE44XX_SPI->SR & (1 << 1)));
		 AFE44XX_SPI->DR = data;
	   while (!(AFE44XX_SPI->SR & 1));
	   return   (uint8_t)(AFE44XX_SPI->DR);
}

/*********************************************************************

*********************************************************************/
void  AFE44xx_Write( uint8_t   adr,  uint32_t   data )
{
	    ENABLE_SELECT();
	    Spi2WriteByte( adr );
	    Spi2WriteByte( (uint8_t)(data>>16) );	     
	    Spi2WriteByte( (uint8_t)(data>>8) );
	    Spi2WriteByte( (uint8_t)(data>>0) );	
	    DISABLE_SELECT();
}

/*********************************************************************

*********************************************************************/
uint32_t  AFE44xx_Read( uint8_t   adr )
{
	    uint32_t  result;
	
	    ENABLE_SELECT();
	    Spi2WriteByte( adr );
	    result = Spi2WriteByte( 0xff );
      result <<= 8;	
	    result += Spi2WriteByte( 0xff );
	    result <<= 8;
	    result += Spi2WriteByte( 0xff );	
	    DISABLE_SELECT();
      
	    return result;
}

/*********************************************************************

*********************************************************************/
int32_t  AFE44xx_ReadADC( uint8_t   adr )
{
	    int32_t  result;
	    ENABLE_AFEXXEN() ;//POWER UP
	    ENABLE_SELECT();
	    Spi2WriteByte( adr );
	    result = Spi2WriteByte( 0xff );
      result *= 256;	
	    result += Spi2WriteByte( 0xff );
	    result *= 256;
	    result += Spi2WriteByte( 0xff );	
	    DISABLE_SELECT();

	    if( result & (1<<22) )
			{
          result = result- 0x1000000;  
      }
			else
			{
          result &= 0x1fffff; 
      }
	  result = (result+10000)/4;
    	if(result<0)
			{
				result = 0;
			}
	    return result;
}

/*********************************************************************

*********************************************************************/

 void  AFE44xx_Init( void )
 {     
//		  uint32_t tickstart = 0;
	    AFE44xx_gpio_init();
			DISABLE_SELECT();
      AFE44xx_spi_init();		 
	    DISABLE_PDN();
	    DelayMS( 200 );
	    
	    ENABLE_RESET();
	    DelayMS( 200 );
	    DISABLE_RESET();

// // // //¿ØÖÆ¼Ä´æÆ÷0
// // // //¸´Î»Ê±ÖÓ¼ÆÊýÆ÷
			AFE44xx_Write( 0x00,  0x0002); //Timer counter reset	
	 
      AFE44xx_Write( 0x03,  0); //LED2 start
      AFE44xx_Write( 0x04,  1999);

	    AFE44xx_Write( 0x09,  4000); //LED1 start
      AFE44xx_Write( 0x0a,  5999);	
	 
			AFE44xx_Write( 0x01,  80); //Sample LED2 start
      AFE44xx_Write( 0x02,  1998);

      AFE44xx_Write( 0x07,  4080); //Sample LED1 start
      AFE44xx_Write( 0x08,  5998);

      AFE44xx_Write( 0x05,  2080); //Sample ambient LED2 start
      AFE44xx_Write( 0x06,  3998);			

		
      AFE44xx_Write( 0x0b,  6080); //Sample ambient LED1 start
      AFE44xx_Write( 0x0c,  7998);
			
			AFE44xx_Write( 0x0d,  2006);     //LED2 convert start
      AFE44xx_Write( 0x0e,  3999);  
      AFE44xx_Write( 0x15,  2000);     //ADC reset 0 start
      AFE44xx_Write( 0x16,  2005);			
			
      AFE44xx_Write( 0x0f,  4006);  //LED2 ambient convert start
      AFE44xx_Write( 0x10,  5999);
      AFE44xx_Write( 0x17,  4000);  //ADC reset 1 start
      AFE44xx_Write( 0x18,  4005);	

      AFE44xx_Write( 0x11,  6006); //LED1 convert start
      AFE44xx_Write( 0x12,  7999);
      AFE44xx_Write( 0x19,  6000); //ADC reset 2 start
      AFE44xx_Write( 0x1a,  6005);
			
      AFE44xx_Write( 0x13,  8006); //LED1 ambient convert start
      AFE44xx_Write( 0x14,  9999);
      AFE44xx_Write( 0x1b,  8000); //ADC reset 3 start
      AFE44xx_Write( 0x1c,  8005);
      AFE44xx_Write( 0x1d,   39999);			//100HZ
//			//¿ØÖÆ¼Ä´æÆ÷0

//			//¿ØÖÆ¼Ä´æÆ÷1 0x1e
//      //4400Ã»ÓÐ¹ý²ÉÑù¹¦ÄÜ£¬20uS²ÉÑù±£³Ö+50uS×ª»»Ê±¼ä==·¢¹âÊ±¼ä
//      //Ê¹ÄÜ¾¯±¨¹¦ÄÜ
	    AFE44xx_Write( 0x1e,  0x102);//4400
//	 
//      //ÅäÖÃTIA¼Ä´æÆ÷ 0x20£¬³õÊ¼»¯Ê±ËùÓÐÊý¾ÝÎ»ÖÃÁã
		  AFE44xx_Write( 0x20,  0x00); 
//			
//      //TIAÔöÒæºÍ»·¾³¹âµÖÏû¼Ä´æÆ÷ÅäÖÃ0x21
//      //10KµÄ·´À¡µç×è¡¢20pfµÄ·´À¡µçÈÝ
//      //¹Ø±ÕµÚ¶þ½×·Å´óÆ÷
//      //¹Ø±Õ»·¾³¹âµÖÏûµçÂ·
		  //AFE44xx_Write( 0x21,  0x00f3 );
			AFE44xx_Write( 0x21,  0x00f2 );
//     	
//      //LED¿ØÖÆ¼Ä´æÆ÷0x22
//      //Êä³öµçÁ÷==CODE*×î´óÊä³öµçÁ÷/256
//      //Full-Scale LED Current 50 mA
 		 AFE44xx_Write( 0x22,  0x1ffff);
		 //AFE44xx_Write( 0x22,  0x11010);
//		

//      //LED¿ØÖÆ¼Ä´æÆ÷2 0x23
//      //LEDÇý¶¯ÅäÖÃÎª¹²ÑôÄ£Ê½
//      //´ò¿ªTX¡¢RXÊ¹ÄÜ
//      //´ò¿ªAFEÊ¹ÄÜ
      AFE44xx_Write( 0x23,  0x20900); 
//		
//		
//      //Disables the monitoring of internal clocks
     // AFE44xx_Write( 0x29,  0x0000);			

			//DelayMS( 100 ); 
//			
		  AFE44xx_Write( 0x00,  0x01); // enable read
//			
     AFE44xx_Diag = AFE44xx_Read( 0x1d );
//		 while(1)
//		 {
//	       if(AFE44xx_ADRDY == 1)
//		     {
//					 
//					 	HAL_NVIC_DisableIRQ(ADRDY_EXTI_IRQn);
//             AFE44xx_RecData();
//	         HAL_NVIC_EnableIRQ(ADRDY_EXTI_IRQn);					 
//			       AFE44xx_ADRDY = 0;
//		     }
// 
//		 }
//					
}

/*********************************************************************

*********************************************************************/
//void PIOINT0_IRQHandler( void )
//{
//      LPC_GPIO0->IC   =  (1 << 7);
//	    AFE4400_AdcRdy = 1;
//}

/*********************************************************************

*********************************************************************/
//void WAKEUP_IRQHandler( void )
//{
//	    LPC_SYSCON->STARTRSRP0CLR  = (1<<7);
//	    AFE4400_AdcRdy = 1;
//}

/*********************************************************************
*********************************************************************/
//uint32_t  AFE44xx_Diagnostic( void )
//{
//	    AFE44xx_Write( 0x00,  0x04); //Diagnostic enable
//	    while( !( GPIO_ReadInputDataBit(AFE44XX_DIAG_GPIO, AFE44XX_DIAG_PIN))&(1<<1) );//Diagnostic complete
//	    AFE44xx_Write( 0x00,  0x01); // enable read
//      AFE44xx_Diag = AFE44xx_Read( 0x30 );
//	    return  0;
//}

/*********************************************************************
	    
*********************************************************************/
//unsigned long Mem1[8]={0};
//unsigned long Mem2[8]={0};
//int cnt = 0;
//void inital_adjust()
//{
//    for(cnt=0;cnt<8;cnt++)
//	  {
//	 	    Mem1[cnt]=0;
//		    Mem2[cnt]=0;
//	  }
//	  cnt = 0;
//}

//void adjust()
//{
//    signed long s1=0;
//    signed long s2=0;
//    unsigned long i;
//    unsigned long nn=0;
//    for(i=7;i>0;i--)
//	  { 
//		    Mem1[i]= Mem1[i-1];
//		    Mem2[i]= Mem2[i-1];
//	  }
//	  Mem1[0]= gRDVP;
//	  Mem2[0]= gIRVP;
//	
//    for(i=0;i<8;i++)
//	  { 
//		    if(Mem1[i]>0&&Mem2[i]>0)
//		    {
//			      s1 +=  Mem1[i];
//			      s2 +=  Mem2[i];
//			      nn++;
//		    }
//	  }
//	  if(nn>0)
//	  {
//		    s1 /=nn;
//		    s2 /=nn;
//	  }
//	
//    CurCtrl_Data = s1;
//    CurCtrl_Data2 = s2;
//	  FingerOutCheck();
//	  ProbOutCheck();
//}
long * AFE44xx_RecData(void)
{
			static long AFE44xxInitialData[3];
	     AFE44xx_Write( 0x00,  0x01);
	      gRDV0 = AFE44xx_ReadADC( 0x2b);//+10000l;
	      gRDV0 = gRDV0 & 0x3FFFFF;
	      if(gRDV0 & 0x200000) //²¹Âë×ª»»
				{
				    gRDV0 =  gRDV0 & 0x1FFFFF;
				}
				else
				{
				    gRDV0 = gRDV0 + 0x400000;
				}
	      gRDV0 = (uartCount3 ++)& 0xFFFFFF ; 
		    //gRDVP = AFE44xx_ReadADC( 0x2e);//+10000l;
	      gRDVP = AFE44xx_ReadADC( 0x2A);//+10000l;
	      gRDVP = gRDVP & 0x3FFFFF;
	      if(gRDVP & 0x200000) //²¹Âë×ª»»
				{
				    gRDVP =  gRDVP & 0x1FFFFF;
				}
				else
				{
				    gRDVP = gRDVP + 0x400000;
				}
	      gRDVP = (uartCount2 ++)& 0xFFFFFF  ;//+10000l;
		    gIRVP = AFE44xx_ReadADC( 0x2c);//+10000l;
	      gIRVP = gIRVP & 0x3FFFFF;
	      if(gIRVP & 0x200000) //²¹Âë×ª»»
				{
				    gIRVP =  gIRVP & 0x1FFFFF;
				}
				else
				{
				    gIRVP = gIRVP + 0x400000;
				}
	     gIRVP  = (uartCount1 ++)& 0xFFFFFF  ;
	      AFE44xx_Write( 0x00,  0x00);
//		    if ( KeyFingerOut == 0)
//        {	    
//            PROG_F |= 0x04;//ADJ_F;
//            Mem_IRVP[ index_Mem_in ] = gIRVP&0x7fffff;
//            Mem_RDVP[ index_Mem_in ] = gRDVP&0x7fffff;
//            Mem_RDV0[ index_Mem_in++ ] = gRDV0;//gRDV0;
//				    //gRDVP_gIRVP = (float)((gIRVP + gRDVP)/2);
//	         // Mem_VPavg[ index_Mem_in ] = gRDVP_gIRVP;
				AFE44xxInitialData[0] =  (unsigned long)gRDVP|0x01000000;
				//AFE44xxInitialData[0] =  0x5a5a5a5a;
				AFE44xxInitialData[1] =  (unsigned long)gIRVP|0x03000000;
				//AFE44xxInitialData[1] =  0x6a5a5a5a;
				AFE44xxInitialData[2] =  (unsigned long)gRDV0|0x02000000;
				//AFE44xxInitialData[2] =  0x7a5a5a5a;
				return(AFE44xxInitialData);
						//sprintf(aTxBuffer_temp,"%lx\n%lx\n",(unsigned long)gRDVP|0x01000000,(unsigned long)gIRVP|0x03000000);
				   //sprintf(aTxBuffer_temp,"%lx\n%lx\n",(unsigned long)4294967295,(unsigned long)4294967295);
			      //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)aTxBuffer_temp,COUNTOF(aTxBuffer_temp)-1);
				//HAL_UART_Transmit_DMA(&huart1, aTxBuffer_test2,COUNTOF(aTxBuffer_test2));

}
uint32_t  AFE44xx_Processor( void )
{
	      //unsigned long HRCount;
	      unsigned int i;
			  float Mem_VPavg[ Len_Mem ];
        float Mem_VPavg_processed[Len_Mem]; 
//	      unsigned long Mem_VPavg_processed_uint32[Len_Mem];
	      unsigned char dataModifyflag = 0;
	      unsigned char dataModifyAddr = 0;
	      unsigned long mmaDataA_Pre;
	      long Mem_RDV0_Initial[Len_Mem];
            if ( index_Mem_in >= Len_Mem )
               {              
							 	 
							 for (i = 0 ;i<Len_Mem;i++)   //´Ó»º´æÖÐÈ¡Êý¾Ý
								{
								Mem_VPavg[i] =(float)((Mem_IRVP[i] + Mem_RDVP[i])>>1);
								Mem_RDV0_Initial[i] = Mem_RDV0[i];
								}
								for(i = 0 ;i<Len_Mem;i++)
								{
								  //debug_printf("%lx\n",(unsigned long)Mem_VPavg[i]|0x01000000);
									// sprintf(aTxBuffer_temp,"%lx\n%lx\n",(unsigned long)Mem_RDVP[i]|0x01000000,(unsigned long)Mem_IRVP[i]|0x01000000);
			            // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)aTxBuffer_temp,COUNTOF(aTxBuffer_temp));
									//debug_printf("%lx\n",(unsigned long)Mem_RDVP[i]|0x01000000);
								 // debug_printf("%lx\n",Mem_RDV0_Initial[i]|0x03000000);
                 //debug_printf("%lx\n",Mem_IRVP[i]|0x03000000);									
								}
//							 Interrupt_Disable();				//ÆÁ±ÎÖÐ¶Ï	
							 WaveProcess(Mem_VPavg,Mem_VPavg_processed); //µ÷ÓÃ¿¹ÔË¶¯Ëã·¨
							 dataModifyflag = 0;
							 dataModifyAddr = 0;
							 mmaDataA_Pre = mmaDataA[Len_Mem-1]; 
							 for (i = 0 ;i<Len_Mem;i++)                  //¸ñÊ½×ª»»£¬½«Êý¾Ý±ä³É×ÔÈ»Êý
								 { 	 
								 //Mem_VPavg_processed_uint32[i] = (unsigned long)(Mem_VPavg_processed[i]*100+100000);
									 
									 mmaDataA[i] = (unsigned long)(Mem_VPavg_processed[i]*10+100000);
//									 if((i>=1)&&(dataModifyflag == 0)) //¿¹ÔË¶¯Ëã·¨ÆðÊ¼µã²»¶Ô£¬ÐèÒªÆ´½ÓÉÏ´Î¼ÆËã½á¹û,ÒÔdataModifyflag×÷ÎªÊý¾ÝÐÞÕý±êÖ¾
//									 {
//										 if(((mmaDataA[i]> mmaDataA_Pre)&&(mmaDataA[i-1]<=mmaDataA_Pre))||((mmaDataA[i]<mmaDataA_Pre)&&(mmaDataA[i-1]>=mmaDataA_Pre)))
//									   {
//										 dataModifyAddr = i;//ÕÒµ½½áÊøÐÞÕýµã
//										 dataModifyflag = 1;//ÕÒµ½½áÊøÐÞÕýµã
//										 }
//									 }
								 }
//							for(i =0;i<dataModifyAddr;i++)
//                  {
//									 mmaDataA[i] = mmaDataA_Pre;
//									 }
							 HRCount = (unsigned long)heartRateDataProcess(mmaDataA);	//µ÷ÓÃ¼ì²¨Ëã·¨
							 //HRCount = (uartCount4 ++)& 0xFFFFFF  ;					
								/////////////////////////get Heart ratio Per minuts             
							 HRCountFIFO[HRCountFIFOAdd] = HRCount;
							 for(i = 0;i<20;i++)                        //Æ½»¬Í³¼Æ1·ÖÖÓÄÚµÄÐÄÂÊ×ÜÊý£¬FIFOÖÐÇóºÍ
							 {
							 HRPerMinTemp = HRCountFIFO[i] + HRPerMinTemp;					
							 }
							 HRPerMin = HRPerMinTemp;						 
							 for(i=Len_Mem;i<index_Mem_in;i++)           //»º´æÇøÊý¾Ý°áÒÆ,Ê£ÓàFIFOÊý¾ÝÇ¨µ½Ç°¶Ë
							 {
                  Mem_IRVP[ i - Len_Mem ] = Mem_IRVP[ i ] ;
                  Mem_RDVP[  i - Len_Mem ] = Mem_RDVP[  i];
                  Mem_RDV0[ i - Len_Mem ] = Mem_RDV0[ i];							 
							 
							 }
							 index_Mem_in = index_Mem_in-Len_Mem;		    //»º´æÇø¸öÊý¼õÈ¥ÒÑ¾­¶ÁÈ¡µÄ¸öÊý£¬Ö¸Õë»ØÎ»
							// Interrupt_Init();                          //ÖØÐÂ¿ªÆôÖÐ¶Ï
							 //OLED_Clear();
               //OLED_ShowNum(HeartRatio_RealTimeVaule,15);							 
							 HRPerMinTemp = 0;                           //1·ÖÖÓÄÚÐÄÂÊÖµ¹é0
							 HRCountFIFOAdd = HRCountFIFOAdd + 1;        //Ã¿3SÐÄÂÊÖµFIFOµØÖ·Ö¸Õë¼Ó1
							 if(HRCountFIFOAdd >= 20)
							 {
							  HRCountFIFOAdd = 0;
							 }						 
							 /////////////////////////////////////////////////////////// 
							 for (i = 0 ;i<Len_Mem;i++) 
							 {
								//debug_printf("%lx\n",mmaDataA[i]|0x02000000);
								//debug_printf("%lx\n",xyzAxisAbsdataNew[i]|0x04000000);	
							 }								 
							// debug_printf("%lx\n",HRPerMin|0x05000000); 
               //debug_printf("%lx\n",(unsigned long )HeartRatio_RealTimeVaule|0x05000000); 						 
						   //debug_printf("%lx\n",HRCount|0x04000000);	 
                						 
                }


//        }	
				return HRCount;
}

/*********************************************************************

*********************************************************************/
void  AFE44xx_DriveCurentOff( void )
{ 
	    AFE44xx_Write( 0x22,  0x30000);//0ma
}


/*********************************************************************

*********************************************************************/
void  AFE44xx_DriveCurentOn( void )
{
	    AFE44xx_Write( 0x22,  0x1A0A0);//5ma
}

/*********************************************************************
	    
*********************************************************************/
//void  FingerInCheck( void )
//{
//       	if( KeyFingerOut == 1 )
//				{				
//					   if( ( gRDVP < 400000 ) && (gIRVP < 400000 ) )//²åÈë¼ì²â
//						 {
//							     FingerOutCheckDelay++;
//							     if( FingerOutCheckDelay >= 2 )
//									 {
//                            	InitCtrl_Data()	;

//								Ram_Clr3();
//								inital_adjust();
//								AFE4400_DriveCurentOn();
//					//			 AFE4400_Init();
//                           		 KeyFingerOut = 0;//37.5ma,½øÈëÕý³£²âÁ¿Ä£Ê½
//                   }
//             }
//						 else
//						 {
//                   FingerOutCheckDelay = 0;  
//             }
//		}  
//}
unsigned char heartRateDataProcess(unsigned long *mmaDataA)
{
  unsigned int i;
unsigned long xyzAxisAbsdataAvg[3] = {0,0,0};
unsigned int  stepCount = 0;
unsigned long xyzAxisAbsdataMax[3] = {0,0,0};
unsigned long xyzAxisAbsdataMin[3] = {10000000,10000000,10000000};
//unsigned int Sigama;

  ///////////////get initial data
	for(i=0;i< FilterCofdepth-1; i++)
		{
		mmaDataFilterTemp[i] = (unsigned long)mmaDataFilterPredata[i];
		}
	for(i=FilterCofdepth-1;i< mmaDataACnt+FilterCofdepth; i++)
		{
		mmaDataFilterTemp[i] = (unsigned long)mmaDataA[i-FilterCofdepth+1];
		}	  
   for(i=0;i<mmaDataACnt;i++)
	 {
	    mmaDataFilter[i] = ((mmaDataFilterTemp[0+i]+ mmaDataFilterTemp[10+i])*filterCof[0]+\
	                       (mmaDataFilterTemp[1+i]+ mmaDataFilterTemp[9+i])*filterCof[1]+\
	                       (mmaDataFilterTemp[2+i]+ mmaDataFilterTemp[8+i])*filterCof[2]+\
	                       (mmaDataFilterTemp[3+i]+ mmaDataFilterTemp[7+i])*filterCof[3]+\
	                       (mmaDataFilterTemp[4+i]+ mmaDataFilterTemp[6+i])*filterCof[4]+\
	                       mmaDataFilterTemp[5+i]*filterCof[5])>>10;  		 
    }
    for(i=0;i<mmaDataACnt;i++)
       {
		    if(i < SlideLenght)
			  {
			   if(mmaDataFilter[i] > xyzAxisAbsdataMax[0])
			     {
			     xyzAxisAbsdataMax[0]  = mmaDataFilter[i];
			 	   }
			   else
			 	   {
			       xyzAxisAbsdataMax[0]  = xyzAxisAbsdataMax[0];
			 	   }
			   if(mmaDataFilter[i] <  xyzAxisAbsdataMin[0])
			 	   {
			       xyzAxisAbsdataMin[0]  = mmaDataFilter[i];
			 	   }
			   else
			 	   {
			        xyzAxisAbsdataMin[0]  =  xyzAxisAbsdataMin[0];
			 	   }
			  }
		    else if(i < SlideLenght*2)
			    {
			     if(mmaDataFilter[i] > xyzAxisAbsdataMax[1])
			     {
			       xyzAxisAbsdataMax[1]  = mmaDataFilter[i];
			 	   }
			     else
			 	   {
			       xyzAxisAbsdataMax[1]  = xyzAxisAbsdataMax[1];
			 	   }
			    if(mmaDataFilter[i] <  xyzAxisAbsdataMin[1])
			 	    {
			       xyzAxisAbsdataMin[1]  = mmaDataFilter[i];
			 	    }
			    else
			 	   {
			        xyzAxisAbsdataMin[1]  =  xyzAxisAbsdataMin[1];
			 	   }
			     }	
		    else
			    {
			     if(mmaDataFilter[i] > xyzAxisAbsdataMax[2])
			     {
			       xyzAxisAbsdataMax[2]  = mmaDataFilter[i];
			 	   }
			     else
			 	   {
			       xyzAxisAbsdataMax[2]  = xyzAxisAbsdataMax[2];
			 	   }
			    if(mmaDataFilter[i] <  xyzAxisAbsdataMin[2])
			 	    {
			       xyzAxisAbsdataMin[2]  = mmaDataFilter[i];
			 	    }
			    else
			 	   {
			        xyzAxisAbsdataMin[2]  =  xyzAxisAbsdataMin[2];
			 	   }
			     }	
         	}
	    
        xyzAxisAbsdataAvg[0]  = (xyzAxisAbsdataMax[0] + xyzAxisAbsdataMin[0])>>1;
        xyzAxisAbsdataAvg[1]  = (xyzAxisAbsdataMax[1] + xyzAxisAbsdataMin[1])>>1;
        xyzAxisAbsdataAvg[2]  = (xyzAxisAbsdataMax[2] + xyzAxisAbsdataMin[2])>>1;
		///////////////////////////////////////////???? ???? ??????
        xyzAxisAbsdataNew[0] =  mmaDataFilter[0];
				xyzAxisAbsdataOld[0] =  mmaDataFilter[0];	
        for (i=0;i<mmaDataACnt-1;i++)
            {
            if(mmaDataFilter[i+1]>mmaDataFilter[i])
             	 {
			         if(mmaDataFilter[i+1] - mmaDataFilter[i]>DYNAMIC_PRECISE)
			 	          {
			 	          xyzAxisAbsdataNew[i+1] = mmaDataFilter[i+1];
			 	          }
			         else
			 	          {
			 	          xyzAxisAbsdataNew[i+1] = xyzAxisAbsdataNew[i];
			           	}
             	}
	           else
	            	{
			          if(mmaDataFilter[i] - mmaDataFilter[i+1]>DYNAMIC_PRECISE)
			 	        {
			 	          xyzAxisAbsdataNew[i+1] = mmaDataFilter[i+1];
			         	}
			          else
			 	         {
			 	           xyzAxisAbsdataNew[i+1] = xyzAxisAbsdataNew[i];
			          	}		
             	  }           
			
          xyzAxisAbsdataOld[i+1] = xyzAxisAbsdataNew[i];
			if(i <SlideLenght)
				{
            if(((xyzAxisAbsdataOld[i+1]>=xyzAxisAbsdataAvg[0]) && (xyzAxisAbsdataAvg[0]>= xyzAxisAbsdataNew[i+1] )) )
                 {
                 if ((timeWindowCount < timeWindowCountMax) && (timeWindowCount > timeWindowCountMin) )
				            {
                       stepCount = stepCount + 1;
											 HeartRatio_RealTimeVaule_FIFO[HeartRatio_RealTimeVaule_FIFOaddr] = timeWindowCount;
											 HeartRatio_RealTimeVaule_FIFOaddr = HeartRatio_RealTimeVaule_FIFOaddr + 1;										
                    }
                 timeWindowCount = 0;										
                }    
			      else
			           {
                 timeWindowCount = timeWindowCount + 1;
			           }
				}
			else if(i <SlideLenght*2)
				{
            if(((xyzAxisAbsdataOld[i+1]>=xyzAxisAbsdataAvg[1]) && (xyzAxisAbsdataAvg[1]>= xyzAxisAbsdataNew[i+1] )) )
                 {
                 if ((timeWindowCount < timeWindowCountMax) && (timeWindowCount > timeWindowCountMin) )
				            {
                       stepCount = stepCount + 1; 
                      // HeartRatio_RealTimeVaule = 	60000/(timeWindowCount*10);//get realtime value	
											 HeartRatio_RealTimeVaule_FIFO[HeartRatio_RealTimeVaule_FIFOaddr] = timeWindowCount;
											 HeartRatio_RealTimeVaule_FIFOaddr = HeartRatio_RealTimeVaule_FIFOaddr + 1;												
                    }
                 timeWindowCount = 0;										
                }    
			      else
			           {
                 timeWindowCount = timeWindowCount + 1;
			           }
				}				
			else
				{
//            if(((xyzAxisAbsdataOld[i+1]>=xyzAxisAbsdataAvg[2]) && (xyzAxisAbsdataAvg[2]>= xyzAxisAbsdataNew[i+1] )) || ((xyzAxisAbsdataOld[i+1]<=xyzAxisAbsdataAvg[2]) && (xyzAxisAbsdataAvg[2]<= xyzAxisAbsdataNew[i+1] )) )
					    if(((xyzAxisAbsdataOld[i+1]>=xyzAxisAbsdataAvg[2]) && (xyzAxisAbsdataAvg[2]>= xyzAxisAbsdataNew[i+1] )) )
                 {
                 if ((timeWindowCount < timeWindowCountMax) && (timeWindowCount > timeWindowCountMin) )
				            {
                       stepCount = stepCount + 1;  
                       //HeartRatio_RealTimeVaule = 	60000/(timeWindowCount*10);//get realtime value
                       HeartRatio_RealTimeVaule_FIFO[HeartRatio_RealTimeVaule_FIFOaddr] = timeWindowCount;
											 HeartRatio_RealTimeVaule_FIFOaddr = HeartRatio_RealTimeVaule_FIFOaddr + 1;												
                    }
                 timeWindowCount = 0;										
                }    
			      else
			           {
                 timeWindowCount = timeWindowCount + 1;
			           }
				}						
			}	
      for(i=0;i<HeartRatio_RealTimeVaule_FIFOaddr;i++)  //Çó3SÄÚµÄ¾ùÖµ
			{
			    HeartRatio_RealTimeVauleTemp = HeartRatio_RealTimeVauleTemp + HeartRatio_RealTimeVaule_FIFO[i];
      }				
			HeartRatio_RealTimeVauleTemp = 	60000*HeartRatio_RealTimeVaule_FIFOaddr/(HeartRatio_RealTimeVauleTemp*10);//get realtime value
			//»µµã¼ì²â £¬¾²Ì¬£¬Ð¡ÓÚ100£¬·½²îÖµÎ»sigama = 3 ,´óÓÚ100£¬·½²îÖµÎ»3% HRPerMin¡
			//Ñ¡ÔñºÏÊÊµÄsigama
//			if(HRPerMin <100)
//		      Sigama = 3;
//		  else
//				  Sigama = HRPerMin*3/100;
//			if(HeartRatio_RealTimeVauleTemp > HRPerMin)
//			{
//			if(HeartRatio_RealTimeVauleTemp - HRPerMin < Sigama)
//			    HeartRatio_RealTimeVaule = HeartRatio_RealTimeVauleTemp;
//			else if(HeartRatio_RealTimeVauleTemp - HRPerMin < 2*Sigama)
//				  HeartRatio_RealTimeVaule = HRPerMin + Sigama;
//			else if(HeartRatio_RealTimeVauleTemp - HRPerMin < 3*Sigama)
//				  HeartRatio_RealTimeVaule = HRPerMin + 2*Sigama;
//			else
//				  HeartRatio_RealTimeVaule = HRPerMin + 3*Sigama;
//			}
//			else
//			{
//			if(HRPerMin -HeartRatio_RealTimeVauleTemp< Sigama)
//			    HeartRatio_RealTimeVaule = HeartRatio_RealTimeVauleTemp;
//			else if(HRPerMin - HeartRatio_RealTimeVauleTemp< 2*Sigama)
//				  HeartRatio_RealTimeVaule = HRPerMin - Sigama;
//			else if(HRPerMin -HeartRatio_RealTimeVauleTemp< 3*Sigama)
//				  HeartRatio_RealTimeVaule = HRPerMin - 2*Sigama;
//			else
//				  HeartRatio_RealTimeVaule = HRPerMin - 3*Sigama;			
//			}		
      HeartRatio_RealTimeVaule = HeartRatio_RealTimeVauleTemp;
			HeartRatio_RealTimeVaule_FIFOaddr = 0;
			HeartRatio_RealTimeVauleTemp = 0;
//	//******************************************************************//
//			if((stepCount & 0x0001) == 1)
//			{
//			stepCount = (stepCount>>1) + 1;
//			}
//			else
//		  {
//			stepCount = stepCount>>1;
//			}
	//**********************************************??5S?,4?TIMEZONE??
        ///////////////////////////////////////??
        for(i=0;i< FilterCofdepth-1; i++)
	    {
	     mmaDataFilterPredata[i] = mmaDataA[mmaDataACnt-FilterCofdepth+i+1];
	     }        
        return stepCount;
} 

//////////////////////////////////////////////////////////////////////////////////
//@fn£ºheartrate signal process
///////////////////////////////////////////////////////////////////////////////////
void zjw_conv(float *Input,float *Filter,float *Output,int lx)
{
    int i;
    int j;
	int lf = 18;
    float sum;

	Output[0] = Input[0] * Filter[lf-18];

    Output[1] = Input[0] * Filter[lf-17]
		      + Input[1] * Filter[lf-18];

    Output[2] = Input[0] * Filter[lf-16]
		      + Input[1] * Filter[lf-17]
		      + Input[2] * Filter[lf-18];
		        
    Output[3] = Input[0] * Filter[lf-15]
		      + Input[1] * Filter[lf-16]
		      + Input[2] * Filter[lf-17]
		      + Input[3] * Filter[lf-18];

    Output[4] = Input[0] * Filter[lf-14]
		      + Input[1] * Filter[lf-15]
		      + Input[2] * Filter[lf-16]
		      + Input[3] * Filter[lf-17]
		      + Input[4] * Filter[lf-18];

    Output[5] = Input[0] * Filter[lf-13]
		      + Input[1] * Filter[lf-14]
		      + Input[2] * Filter[lf-15]
		      + Input[3] * Filter[lf-16]
		      + Input[4] * Filter[lf-17]
		      + Input[5] * Filter[lf-18];

    Output[6] = Input[0] * Filter[lf-12]
		      + Input[1] * Filter[lf-13]
		      + Input[2] * Filter[lf-14]
		      + Input[3] * Filter[lf-15]
		      + Input[4] * Filter[lf-16]
		      + Input[5] * Filter[lf-17]
		      + Input[6] * Filter[lf-18];
	
	Output[7] = Input[0] * Filter[lf-11]
		      + Input[1] * Filter[lf-12]
		      + Input[2] * Filter[lf-13]
		      + Input[3] * Filter[lf-14]
		      + Input[4] * Filter[lf-15]
		      + Input[5] * Filter[lf-16]
		      + Input[6] * Filter[lf-17];
			  + Input[6] * Filter[lf-18];

	Output[8] = Input[0] * Filter[lf-10]
		      + Input[1] * Filter[lf-11]
		      + Input[2] * Filter[lf-12]
		      + Input[3] * Filter[lf-13]
		      + Input[4] * Filter[lf-14]
		      + Input[5] * Filter[lf-15]
		      + Input[6] * Filter[lf-16]
			  + Input[6] * Filter[lf-17]
			  + Input[6] * Filter[lf-18]; 

	Output[9] = Input[0] * Filter[lf-9]
		      + Input[1] * Filter[lf-10]
		      + Input[2] * Filter[lf-11]
		      + Input[3] * Filter[lf-12]
		      + Input[4] * Filter[lf-13]
		      + Input[5] * Filter[lf-14]
		      + Input[6] * Filter[lf-15]
			  + Input[6] * Filter[lf-16] 
			  + Input[6] * Filter[lf-17]	
			  + Input[6] * Filter[lf-18];
			 
	Output[10] = Input[0] * Filter[lf-8]
		      + Input[1] * Filter[lf-9]
		      + Input[2] * Filter[lf-10]
		      + Input[3] * Filter[lf-11]
		      + Input[4] * Filter[lf-12]
		      + Input[5] * Filter[lf-13]
		      + Input[6] * Filter[lf-14]
			  + Input[6] * Filter[lf-15] 
			  + Input[6] * Filter[lf-16]	
			  + Input[6] * Filter[lf-17]
			  + Input[6] * Filter[lf-18];

	Output[11] = Input[0] * Filter[lf-7]
		      + Input[1] * Filter[lf-8]
		      + Input[2] * Filter[lf-9]
		      + Input[3] * Filter[lf-10]
		      + Input[4] * Filter[lf-11]
		      + Input[5] * Filter[lf-12]
		      + Input[6] * Filter[lf-13]
			  + Input[6] * Filter[lf-14] 
			  + Input[6] * Filter[lf-15]	
			  + Input[6] * Filter[lf-16]
			  + Input[6] * Filter[lf-17]	
			  + Input[6] * Filter[lf-18];

	Output[12] = Input[0] * Filter[lf-6]
		      + Input[1] * Filter[lf-7]
		      + Input[2] * Filter[lf-8]
		      + Input[3] * Filter[lf-9]
		      + Input[4] * Filter[lf-10]
		      + Input[5] * Filter[lf-11]
		      + Input[6] * Filter[lf-12]
			  + Input[6] * Filter[lf-13] 
			  + Input[6] * Filter[lf-14]	
			  + Input[6] * Filter[lf-15]
			  + Input[6] * Filter[lf-16]	
			  + Input[6] * Filter[lf-17]		
			  + Input[6] * Filter[lf-18];			  

	Output[13] = Input[0] * Filter[lf-5]
		      + Input[1] * Filter[lf-6]
		      + Input[2] * Filter[lf-7]
		      + Input[3] * Filter[lf-8]
		      + Input[4] * Filter[lf-9]
		      + Input[5] * Filter[lf-10]
		      + Input[6] * Filter[lf-11]
			  + Input[6] * Filter[lf-12] 
			  + Input[6] * Filter[lf-13]	
			  + Input[6] * Filter[lf-14]
			  + Input[6] * Filter[lf-15]	
			  + Input[6] * Filter[lf-16]		
			  + Input[6] * Filter[lf-17]	
			  + Input[6] * Filter[lf-18];
			  
	Output[14] = Input[0] * Filter[lf-4]
		      + Input[1] * Filter[lf-5]
		      + Input[2] * Filter[lf-6]
		      + Input[3] * Filter[lf-7]
		      + Input[4] * Filter[lf-8]
		      + Input[5] * Filter[lf-9]
		      + Input[6] * Filter[lf-10]
			  + Input[6] * Filter[lf-11] 
			  + Input[6] * Filter[lf-12]	
			  + Input[6] * Filter[lf-13]
			  + Input[6] * Filter[lf-14]	
			  + Input[6] * Filter[lf-15]		
			  + Input[6] * Filter[lf-16]	
			  + Input[6] * Filter[lf-17]
			  + Input[6] * Filter[lf-18];

	Output[15] = Input[0] * Filter[lf-3]
		      + Input[1] * Filter[lf-4]
		      + Input[2] * Filter[lf-5]
		      + Input[3] * Filter[lf-6]
		      + Input[4] * Filter[lf-7]
		      + Input[5] * Filter[lf-8]
		      + Input[6] * Filter[lf-9]
			  + Input[6] * Filter[lf-10] 
			  + Input[6] * Filter[lf-11]	
			  + Input[6] * Filter[lf-12]
			  + Input[6] * Filter[lf-13]	
			  + Input[6] * Filter[lf-14]		
			  + Input[6] * Filter[lf-15]	
			  + Input[6] * Filter[lf-16]
			  + Input[6] * Filter[lf-17]
			  + Input[6] * Filter[lf-18];

	Output[16] = Input[0] * Filter[lf-2]
		      + Input[1] * Filter[lf-3]
		      + Input[2] * Filter[lf-4]
		      + Input[3] * Filter[lf-5]
		      + Input[4] * Filter[lf-6]
		      + Input[5] * Filter[lf-7]
		      + Input[6] * Filter[lf-8]
			  + Input[6] * Filter[lf-9] 
			  + Input[6] * Filter[lf-10]	
			  + Input[6] * Filter[lf-11]
			  + Input[6] * Filter[lf-12]	
			  + Input[6] * Filter[lf-13]		
			  + Input[6] * Filter[lf-14]	
			  + Input[6] * Filter[lf-15]
			  + Input[6] * Filter[lf-16]
			  + Input[6] * Filter[lf-17]
			  + Input[6] * Filter[lf-18];
			  
	// ????????,?????lf?lx-1,?????lx-(lf-1)
	// ???????????
    for(i = lf-1; i< lx; i++)
	{
		
		sum = 0.0;
        for(j=0;j<lf;j++)
		{
			sum = sum + Input[j+i-(lf-1)] * Filter[lf-1-j];
		}
        Output[i] = sum;
	}

	// ???????lf-1???,?????lx?lx+(lf-1)-1
	// lx
    Output[lx+0] = Input[lx-17+0] * Filter[lf-1-0]
		         + Input[lx-17+1] * Filter[lf-1-1]
		         + Input[lx-17+2] * Filter[lf-1-2]
		         + Input[lx-17+3] * Filter[lf-1-3]
		         + Input[lx-17+4] * Filter[lf-1-4]
		         + Input[lx-17+5] * Filter[lf-1-5]
		         + Input[lx-17+6] * Filter[lf-1-6]				 
				 + Input[lx-17+7] * Filter[lf-1-7]
				 + Input[lx-17+8] * Filter[lf-1-8]
				 + Input[lx-17+9] * Filter[lf-1-9]
				 + Input[lx-17+10] * Filter[lf-1-10]
				 + Input[lx-17+11] * Filter[lf-1-11]
				 + Input[lx-17+12] * Filter[lf-1-12]
				 + Input[lx-17+13] * Filter[lf-1-13]
				 + Input[lx-17+14] * Filter[lf-1-14]
				 + Input[lx-17+15] * Filter[lf-1-15]
				 + Input[lx-17+16] * Filter[lf-1-16];
				 
    
	// lx+1??
    Output[lx+1] = Input[lx-16+0] * Filter[lf-1-0]
		         + Input[lx-16+1] * Filter[lf-1-1]
		         + Input[lx-16+2] * Filter[lf-1-2]
		         + Input[lx-16+3] * Filter[lf-1-3]
		         + Input[lx-16+4] * Filter[lf-1-4]
		         + Input[lx-16+5] * Filter[lf-1-5]
				 + Input[lx-16+6] * Filter[lf-1-6]
				 + Input[lx-16+7] * Filter[lf-1-7]
				 + Input[lx-16+8] * Filter[lf-1-8]
				 + Input[lx-16+9] * Filter[lf-1-9]
				 + Input[lx-16+10] * Filter[lf-1-10]
				 + Input[lx-16+11] * Filter[lf-1-11]
				 + Input[lx-16+12] * Filter[lf-1-12]
				 + Input[lx-16+13] * Filter[lf-1-13]
				 + Input[lx-16+14] * Filter[lf-1-14]
				 + Input[lx-16+15] * Filter[lf-1-15];

    // lx+2??
    Output[lx+2] = Input[lx-15+0] * Filter[lf-1-0]
		         + Input[lx-15+1] * Filter[lf-1-1]
		         + Input[lx-15+2] * Filter[lf-1-2]
		         + Input[lx-15+3] * Filter[lf-1-3]
		         + Input[lx-15+4] * Filter[lf-1-4]
				 + Input[lx-15+5] * Filter[lf-1-5]
				 + Input[lx-15+6] * Filter[lf-1-6]
				 + Input[lx-15+7] * Filter[lf-1-7]
				 + Input[lx-15+8] * Filter[lf-1-8]
				 + Input[lx-15+9] * Filter[lf-1-9]
				 + Input[lx-15+10] * Filter[lf-1-10]
				 + Input[lx-15+11] * Filter[lf-1-11]
				 + Input[lx-15+12] * Filter[lf-1-12]
				 + Input[lx-15+13] * Filter[lf-1-13]
				 + Input[lx-15+14] * Filter[lf-1-14];
    // lx+3??
    Output[lx+3] = Input[lx-14+0] * Filter[lf-1-0]
		         + Input[lx-14+1] * Filter[lf-1-1]
		         + Input[lx-14+2] * Filter[lf-1-2]
		         + Input[lx-14+3] * Filter[lf-1-3]
				 + Input[lx-14+4] * Filter[lf-1-4]
				 + Input[lx-14+5] * Filter[lf-1-5]
				 + Input[lx-14+6] * Filter[lf-1-6]
				 + Input[lx-14+7] * Filter[lf-1-7]
				 + Input[lx-14+8] * Filter[lf-1-8]
				 + Input[lx-14+9] * Filter[lf-1-9]
				 + Input[lx-14+10] * Filter[lf-1-10]
				 + Input[lx-14+11] * Filter[lf-1-11]
				 + Input[lx-14+12] * Filter[lf-1-12]
				 + Input[lx-14+13] * Filter[lf-1-13];

    // lx+4??
    Output[lx+4] = Input[lx-13+0] * Filter[lf-1-0]
		         + Input[lx-13+1] * Filter[lf-1-1]
		         + Input[lx-13+2] * Filter[lf-1-2]
				 + Input[lx-13+3] * Filter[lf-1-3]
				 + Input[lx-13+4] * Filter[lf-1-4]
				 + Input[lx-13+5] * Filter[lf-1-5]
				 + Input[lx-13+6] * Filter[lf-1-6]
				 + Input[lx-13+7] * Filter[lf-1-7]
				 + Input[lx-13+8] * Filter[lf-1-8]
				 + Input[lx-13+9] * Filter[lf-1-9]
				 + Input[lx-13+10] * Filter[lf-1-10]
				 + Input[lx-13+11] * Filter[lf-1-11]
				 + Input[lx-13+12] * Filter[lf-1-12];

    // lx+5??
    Output[lx+5] = Input[lx-12+0] * Filter[lf-1-0]
		         + Input[lx-12+1] * Filter[lf-1-1]
				 + Input[lx-12+2] * Filter[lf-1-2]
				 + Input[lx-12+3] * Filter[lf-1-3]
				 + Input[lx-12+4] * Filter[lf-1-4]
				 + Input[lx-12+5] * Filter[lf-1-5]
				 + Input[lx-12+6] * Filter[lf-1-6]
				 + Input[lx-12+7] * Filter[lf-1-7]
				 + Input[lx-12+8] * Filter[lf-1-8]
				 + Input[lx-12+9] * Filter[lf-1-9]
				 + Input[lx-12+10] * Filter[lf-1-10]
				 + Input[lx-12+11] * Filter[lf-1-11];

    // lx+6??
    Output[lx+6] = Input[lx-11+0] * Filter[lf-1-0]
				 + Input[lx-11+1] * Filter[lf-1-1]
				 + Input[lx-11+2] * Filter[lf-1-2]
				 + Input[lx-11+3] * Filter[lf-1-3]
				 + Input[lx-11+4] * Filter[lf-1-4]
				 + Input[lx-11+5] * Filter[lf-1-5]
				 + Input[lx-11+6] * Filter[lf-1-6]
				 + Input[lx-11+7] * Filter[lf-1-7]
				 + Input[lx-11+8] * Filter[lf-1-8]
				 + Input[lx-11+9] * Filter[lf-1-9]
				 + Input[lx-11+10] * Filter[lf-1-10];
				 
    // lx+7??
    Output[lx+7] = Input[lx-10+0] * Filter[lf-1-0]
				 + Input[lx-10+1] * Filter[lf-1-1]
				 + Input[lx-10+2] * Filter[lf-1-2]
				 + Input[lx-10+3] * Filter[lf-1-3]
				 + Input[lx-10+4] * Filter[lf-1-4]
				 + Input[lx-10+5] * Filter[lf-1-5]
				 + Input[lx-10+6] * Filter[lf-1-6]
				 + Input[lx-10+7] * Filter[lf-1-7]
				 + Input[lx-10+8] * Filter[lf-1-8]
				 + Input[lx-10+9] * Filter[lf-1-9];
				 
    // lx+8??
    Output[lx+8] = Input[lx-9+0] * Filter[lf-1-0]
				 + Input[lx-9+1] * Filter[lf-1-1]
				 + Input[lx-9+2] * Filter[lf-1-2]
				 + Input[lx-9+3] * Filter[lf-1-3]
				 + Input[lx-9+4] * Filter[lf-1-4]
				 + Input[lx-9+5] * Filter[lf-1-5]
				 + Input[lx-9+6] * Filter[lf-1-6]
				 + Input[lx-9+7] * Filter[lf-1-7]
				 + Input[lx-9+8] * Filter[lf-1-8];
				 
    // lx+9??
    Output[lx+9] = Input[lx-8+0] * Filter[lf-1-0]
				 + Input[lx-8+1] * Filter[lf-1-1]
				 + Input[lx-8+2] * Filter[lf-1-2]
				 + Input[lx-8+3] * Filter[lf-1-3]
				 + Input[lx-8+4] * Filter[lf-1-4]
				 + Input[lx-8+5] * Filter[lf-1-5]
				 + Input[lx-8+6] * Filter[lf-1-6]
				 + Input[lx-8+7] * Filter[lf-1-7];
				 
    // lx+10??
    Output[lx+10] = Input[lx-7+0] * Filter[lf-1-0]
				  + Input[lx-7+1] * Filter[lf-1-1]
				  + Input[lx-7+2] * Filter[lf-1-2]
				  + Input[lx-7+3] * Filter[lf-1-3]
				  + Input[lx-7+4] * Filter[lf-1-4]
				  + Input[lx-7+5] * Filter[lf-1-5]
				  + Input[lx-7+6] * Filter[lf-1-6];

    // lx+11??
    Output[lx+11] = Input[lx-6+0] * Filter[lf-1-0]
				  + Input[lx-6+1] * Filter[lf-1-1]
				  + Input[lx-6+2] * Filter[lf-1-2]
				  + Input[lx-6+3] * Filter[lf-1-3]
				  + Input[lx-6+4] * Filter[lf-1-4]
				  + Input[lx-6+5] * Filter[lf-1-5];
				  
    // lx+12??
    Output[lx+12] = Input[lx-5+0] * Filter[lf-1-0]
			      + Input[lx-5+1] * Filter[lf-1-1]
				  + Input[lx-5+2] * Filter[lf-1-2]
				  + Input[lx-5+3] * Filter[lf-1-3]
				  + Input[lx-5+4] * Filter[lf-1-4];
				  
    // lx+13??
    Output[lx+13] = Input[lx-4+0] * Filter[lf-1-0]
				  + Input[lx-4+1] * Filter[lf-1-1]
				  + Input[lx-4+2] * Filter[lf-1-2]
				  + Input[lx-4+3] * Filter[lf-1-3];

    // lx+14??
    Output[lx+14] = Input[lx-3+0] * Filter[lf-1-0]	
			      + Input[lx-3+1] * Filter[lf-1-1]	
				  + Input[lx-3+2] * Filter[lf-1-2];
				  
    // lx+15??
    Output[lx+15] = Input[lx-2+0] * Filter[lf-1-0]	
				  + Input[lx-2+1] * Filter[lf-1-1];

    // lx+16??
    Output[lx+16] = Input[lx-1+0] * Filter[lf-1-0];	
}

float y[500];
float Tmp[400];
float a_1[320];
void zjw_dwt2(float *Input,float *Lp_Dec,float *Hp_Dec,float *output_ca,float *output_cd,int Length)
{
    int i;
    int lf = 18;                   // ????????
	int lx;                   // ???????
    int first;
	int last;
	
	int lx2;
    
	
	int p,q;
	

		
	// ??????(?????)
    // lf = length(Lo_D);
    // lx = length(x);
	lx = Length;

    // % first:????????
    // % last:????????
    first = 1;
    last = lx+lf-2;

	for(i=0;i<lf-1;i++)
	{
		y[i] = Input[lf-1-i];
	}
	// ?????????
	for(i=0;i<Length;i++)
	{
		y[lf-1+i] = Input[i];
		
	}
	// ?????????
	for(i=0;i<lf-1;i++)
	{
		y[Length+lf-1+i] = Input[Length-2-i];
	}
    // ???????
	lx2=lx+2*(lf-1);
	// ??????,??????,???Tmp?,?????lx2+lf-1
    zjw_conv(y,Lp_Dec,Tmp,lx2);

//z = Tmp(lf:lx2);
    // ?????????????:?????lf-1?lx2-1,?????lx2-lf+1=lx+2*(lf-1)-(lf-1)=lx+lf-1
	p = 0;
	
	for(i=lf-1;i<lx2;i++)
	{
		C[p] = Tmp[i];
		p++;
	}

//a = z(first:2:last);

    p = first;
	q = 0;
	do
	{
		a_1[q] = C[p];
		q++;
		p = p + 2;
	} while (p <= last);

    // ??????ca
	for(i=0;i<q;i++)
	{
		output_ca[i] = a_1[i];
	}

	// ??????,??????,???Tmp?,?????lx2+lf-1
    zjw_conv(y,Hp_Dec,Tmp,lx2);
	p = 0;
	for(i=lf-1;i<lx2;i++)
	{
		C[p] = Tmp[i];
		p++;
	}

//a = z(first:2:last);

    p = first;
	q = 0;
	do
	{
		a_1[q] = C[p];
		q++;
		p = p + 2;
	} while (p <= last);

    // ??????cd
	for(i=0;i<q;i++)
	{
		output_cd[i] = a_1[i];
	}
}

float output_ca[300];                       // ???????????
float output_cd[300];                       // ???????????
void zjw_wavedec2(float *Input,int Dec_level,float *Lp_Dec,float *Hp_Dec,float *C)
{
    int i,j;
	int p;



    for(i = 0;i<448;i++)
	{
		C[i] = 0;
	}

    // ??p????????
	p = 447;

	// ??,?????????
	for(i=0;i<Dec_level;i++)
	{
        // ?????,????????,??ca?cd
        zjw_dwt2(Input,Lp_Dec,Hp_Dec,output_ca,output_cd,L[Dec_level+1-i]);
		// ?cd?????C[]???
		for(j=0;j<L[Dec_level-i];j++)
		{
            C[p] = output_cd[ L[Dec_level-i] -1 -j];
			p--;
		}
		// ?ca?????input
		for(j=0;j<L[Dec_level-i];j++)
		{
            Input[j] = output_ca[j];
		}

	}

/*
	%?????????????????,????????Input??c??????
c = [Input c];
*/

    for(i=L[0]-1;i>=0;i--)
	{
		C[p] = output_ca[i];
		p--;
	}
}
 
float y_l[400];
float new_y[400];

void zjw_upsconv1(float *a,float *filter,int CurLength,int LastLength,float *a_out)
{
  int lx = 2*CurLength - 1;
//  int lf = 18;

  int i;
	
	int sx;  //Bernie.hou
	
	int first; //Bernie.hou
	
	int last;  //Bernie.hou
	
	int p;  //Bernie.hou


  for(i = 0; i<lx ;i++)
  {
	  y_l[i] = 0;
  }

  for(i = 0; i < CurLength;i++)
  {
	  y_l[2*i] = a[i];
  }

  // ??
  zjw_conv(y_l,filter,new_y,lx);

  //int sx = lx + 18 - 1;  //Bernie.hou
	sx = lx + 18 - 1;
  //int first = (sx-LastLength)/2;  //Bernie.hou
	first = (sx-LastLength)/2;
  //int last;

  if(((sx-LastLength) % 2) != 0)
  {
	  last = sx - ((sx-LastLength)/2 + 1);
  }
  else
  {
	  last = sx - (sx-LastLength)/2;
  }

  // ?new_y????first?last???????
  //int p =0;  //Bernie.hou
	p =0;

  do
  {
	  a_out[p] = new_y[first];
	  p++;
	  first++;
  } while (first < last);

}
void zjw_idwt(float *a,float *d,int CurLength,int LastLength,float *a_out)
{
   int i;

   zjw_upsconv1(a,Lp_R,CurLength,LastLength,Tmp);

   zjw_upsconv1(d,Hp_R,CurLength,LastLength,C);

   for(i = 0;i<LastLength;i++)
   {
	   a_out[i] = Tmp[i] + C[i];
   }
}
void WaveProcess(float *Input,float *Output)
{
    int i, n;
    int Start_Declevel; //Bernie.hou
    int Stop_Declevel;  //Bernie.hou
	    int Tmp;       //Bernie.hou
    int Sum_Dec[20];   //Bernie.hou
	
	    int d_p;  //Bernie.hou
    int j;   //Bernie.hou
  
    zjw_wavedec2(Input,DecLevel,Lp_D,Hp_D,C);    

    // ?2?:???CD6?CD5,?CA9,CD9,CD8,CD7,CD4,CD3,CD2,CD1??????0
	//        CA9+CD9+CD8+CD7?45???,
	//        CA9+CD9+CD8+CD7+CD6+CD5?114???
	//        ???45??,114??????0
   // int Start_Declevel = 71;  //Bernie.hou
   // int Stop_Declevel  = 117;//Bernie.hou
	Start_Declevel = 71;
	 Stop_Declevel  = 117;
   
    // ??
    for(i = 0;i < 448; i++)
    {
        if((i < Start_Declevel) || (i >= Stop_Declevel))
		{
		    C_rec[i] = 0;
		}
	    else
		{
		    C_rec[i] = C[i];
		}
    }

	// ?3?:???????????,?????Sum_Dec[]?,?????10
	//        Sum_Dec[] = {9,18,29,45,70,114,195,351,656,1259}

    Tmp=0;   //Bernie.hou
    //int Sum_Dec[20];   //Bernie.hou
   
    for(n=0;n<DecLevel+1;n++)
    {
	    Tmp = Tmp+L[n];
        Sum_Dec[n]=Tmp;
    }

	// ?4?:??



//    int d_p;  //Bernie.hou
//    int j;    //Bernie.hou

    // ????1?a
    for(i = 0 ; i< L[0];i++) 
	{
	    output_ca[i] = 0;
	}

    for(n = DecLevel;n>0;n--)
	{
        // ????d
		j = Sum_Dec[DecLevel-n];               // ???
		d_p = 0;                               // ??d????
		// ??,??????CDi?????d?
		while(j<Sum_Dec[DecLevel+1-n])
		{
			output_cd[d_p] = C_rec[j];
			d_p++;
			j++;
		}

		// ???a???d,??CA(i)?CD(i),????CA(i-1),?????a_out[]?
		// ?????,L[DecLevel+2-n-1]?CA(i)???,L[DecLevel+2-n]?CD(i)???,??CA(i-1)???
        zjw_idwt(output_ca,output_cd,L[DecLevel+2-n-1],L[DecLevel+2-n],new_y);

		// ????CA(i-1)?????a?
		// ?????????,??CA0?,?????,DecLevel+2-1=10,L[10]=1200
		// ??,???????1200???
		for(i = 0 ; i< L[DecLevel+2-n];i++)
		{
			output_ca[i] = new_y[i];
		}
	}
  
	for(i = 0;i < 300; i++)
	{
		Output[i] = output_ca[i];
	}
}
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
