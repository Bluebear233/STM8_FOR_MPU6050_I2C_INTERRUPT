#include "delay.h"
#include "stm8s.h"

#define DELAY_BEBUG 1

#if DELAY_BEBUG == 1
#include <stdio.h>
#elif DELAY_BEBUG == 0 && defined DELAY_BEBUG
#elif !defined DELAY_BEBUG
#error "please define DELAY_BEBUG in delay.c"
#else
#error "define DELAY_BEBUG error"
#endif



volatile unsigned char fac_us=0; //us延时倍乘数  

//延时函数初始化，用于HSI作为时钟源
void delay_init()
{
	unsigned char clk = CLK->CKDIVR&0x18;
	switch(clk){
		case 0x00:clk = 16;break;
		case 0x08:clk = 2;break;
		case 0X10:clk = 4;break;
		case 0X18:clk = 8;break;
	}
	if(clk>16)fac_us=(16-4)/4;//24Mhz时,stm8大概19个周期为1us
	else if(clk>4)fac_us=(clk-4)/4; 
	else fac_us=1;
}
//延时nus
//延时时间=(fac_us*4+4)*nus*(T)
//其中,T为CPU运行频率(Mhz)的倒数,单位为us.
//准确度:
//92%  @24Mhz
//98%  @16Mhz
//98%  @12Mhz
//86%  @8Mhz
void delay_us(unsigned short nus)
{  
	__asm(
		  "PUSH A          \n"  //1T,压栈
			  "DELAY_XUS:      \n"   
				  "LD A,fac_us     \n"   //1T,fac_us加载到累加器A
					  "DELAY_US_1:     \n"  
						  "NOP             \n"  //1T,nop延时
							  "DEC A           \n"  //1T,A--
								  "JRNE DELAY_US_1 \n"   //不等于0,则跳转(2T)到DELAY_US_1继续执行,若等于0,则不跳转(1T).
									  "NOP             \n"  //1T,nop延时
										  "DECW X          \n"  //1T,x--
											  "JRNE DELAY_XUS  \n"    //不等于0,则跳转(2T)到DELAY_XUS继续执行,若等于0,则不跳转(1T).
												  "POP A           \n"  //1T,出栈
													  ); 
} 
//延时nms  
//为保证准确度,nms不要大于16640.
void delay_ms(unsigned long nms)
{
	u8 t;
	if(nms>65)
	{
		t=nms/65;
		while(t--)delay_us(65000);
		nms=nms%65;
	}
	delay_us(nms*1000);
}