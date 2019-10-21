#include "stm32f10x.h"
GPIO_InitTypeDef 					GPIO_InitStructure;

uint32_t Timingdelay;
#define shcp GPIO_Pin_13
#define stcp GPIO_Pin_14
#define ds GPIO_Pin_15

GPIO_InitTypeDef 					GPIO_InitStructure;
void GPIO_Configuration(void);
void Delay_ms(uint16_t time);
int n=1;
int main(void)
{
	SysTick_Config(SystemCoreClock/1000);
	// goi chuong trinh con da khai bao
	GPIO_Configuration();       
	//int n;
	
  while (1)
  {
		// dung ham xor  dao trang thai led
		//GPIO_WriteBit(GPIOC,GPIO_Pin_13,(BitAction)(1^GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)));	
		// goi ham delay tuong doi
		/*	
			//code gpio tro keo xuong
		if(n==2)
		{
			
			GPIO_WriteBit(GPIOC,shcp,1);
			Delay_ms(1000);				
			
		}
		else
		{
			GPIO_WriteBit(GPIOC,GPIO_Pin_13,0);
			Delay_ms(1000);
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==1)
		{
			while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==1);
			n++;
			
		}
		if(n==3)
		n=1;
		*/
		GPIO_WriteBit(GPIOC,ds,1);
		GPIO_WriteBit(GPIOC,shcp,1);
		GPIO_WriteBit(GPIOC,shcp,0);
		GPIO_WriteBit(GPIOC,stcp,1);
		GPIO_WriteBit(GPIOC,stcp,0);
		Delay_ms(1000);
		GPIO_WriteBit(GPIOC,ds,0);
		GPIO_WriteBit(GPIOC,shcp,1);
		GPIO_WriteBit(GPIOC,shcp,0);
		GPIO_WriteBit(GPIOC,stcp,1);
		GPIO_WriteBit(GPIOC,stcp,0);
		Delay_ms(1000);
		
  }
}
void GPIO_Configuration(void)
{
	// cap clock cho Port B, Port C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
	// cau hinh chan I/O su dung la PC13,toc do input,mode input la Input pull-up
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;						
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;						
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;						
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// cau hinh chan I/O su dung la PB9,toc do output,mode output la Output push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;						
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;			
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
// ham delay tuong doi dung vong lap while
void Delay_ms(uint16_t time)														
{
	// tang bien dem len 12000 lan
	uint32_t time_n=time*12000;		
	// cho den khi biem time_n giam =0 thi thoat	
	while(time_n!=0){time_n--;}													
}
void Delay_ms_sys(uint16_t time)
	{
	Timingdelay = time;						// gan bien dem bang tham so truyen vao
	while(Timingdelay!=0);				// ham cho doi bien dem bang 0.
	
	}
