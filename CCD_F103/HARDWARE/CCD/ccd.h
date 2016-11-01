#ifndef _CCD_H_
#define _CCD_H_

#include "sys.h"

#define SI_SetVal()   GPIO_SetBits(GPIOA,GPIO_Pin_2);//PTE4_OUT = 1;
#define SI_ClrVal()   GPIO_ResetBits(GPIOA,GPIO_Pin_2);//PTE4_OUT = 0;
#define CLK_ClrVal()  GPIO_ResetBits(GPIOA,GPIO_Pin_3);//PTE5_OUT = 0;
#define CLK_SetVal()  GPIO_SetBits(GPIOA,GPIO_Pin_3);//PTE5_OUT = 1;

#define left_size  10
#define right_size 118

void StartIntegration(void);   
void ImageCapture(unsigned char * ImageData);

void SamplingDelay(void);
void LandzoCCD_init(void);
void CalculateIntegrationTime(void) ;
uint8_t PixelAverage(uint8_t len, uint8_t *data) ;
uint8_t PixelAveragejici(uint8_t jicishu);
uint8_t MAX250NUM(void);
void ccdstart(void);

 void fasong(void);
  uint8_t jiyanse(int fengcha);

void zerotip(void);
void firsttip(void);
void secondtip(void);
void thirdtip(void);
void forthtip(void);
void fifthtip(void);
void sixtip(void);

	
int jueduizhi(int a);
void ccd(void);
void ccd_init(void);
void BEEP_Init(void);

u16 zuo(void);
u16 you(void);


void findboundary(void);

typedef enum{MIDDLE,LEFT,RIGHT} LOCAL;
typedef enum{yes,no} VIEW;

typedef struct {
	uint8_t leftnumber;
	uint8_t rightnumber;
	uint8_t baixian;
}SIDE_TypeDef;


u8 getwhiteStripes(void);
u8 getmiddle(void);
u8 getexposuretime(void);
#endif 



