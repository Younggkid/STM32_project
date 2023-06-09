#include "stm32f10x.h"
#include "usart.h"
#include "oled.h"
#include "word.h"
#include "delay.h"
#include "max30102.h"
#include "30102_algorithm.h"
#include "IIC.h"
#include "./systick/bsp_SysTick.h"
#include "./dht11/bsp_dht11.h"
#include "./usart/bsp_usart1.h"
#include "./lcd/bsp_ili9341_lcd.h" 
#include "bsp_ldr.h"
#include "./led/bsp_led.h"
#include "Reg_RW.h"
#include "LDChip.h"
#include "bsp_esp8266.h"
#include "Common.h"
#include "bsp_beep.h"
#include "bsp_esp8266_test.h"
#include "core_delay.h"
#define MAX_BRIGHTNESS 255

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

void User_Modification(u8 dat);

u8 nAsrStatus=0;
u8 nAsrRes=0;
u8 flag=0;

int32_t hr_buf[16];
int32_t hrSum;
int32_t hrAvg;
int32_t spo2_buf[16];
int32_t spo2Sum;
int32_t spo2Avg;
int32_t spo2BuffFilled;
int32_t hrBuffFilled;
int32_t hrValidCnt = 0;
int32_t spo2ValidCnt = 0;
int32_t hrThrowOutSamp = 0;
int32_t spo2ThrowOutSamp = 0;
int32_t spo2Timeout = 0;
int32_t hrTimeout = 0;

u8 Auto = 1; //决定是否自动开关灯
u8 Blood = 0; //决定是否测量血氧
/*struct heart_and_spo
{
	int32_t heart_enable;
	int32_t heart_value;
	int32_t spo_enable;
	int32_t spo_value;
	
};*/


 int main(void)
 { 
	  char dispBuff[100];	 
		DHT11_Data_TypeDef DHT11_Data;
		int i;
		float f_temp;
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int32_t n_brightness;
	 	/* 初始化系统定时器 */
		SysTick_Init();
	 
	 /*外置LED灯的初始化*/
	  GPIO_init();
	 
	 	USARTx_Config();
		CPU_TS_TmrInit();
		ESP8266_Init ();                                                               //初始化WiFi模块使用的接口和外设
		DHT11_Init ();
		Beep_Init ();
		
		//LCD 初始化
		ILI9341_Init (); 
	 //其中0、3、5、6 模式适合从左至右显示文字，
	 //不推荐使用其它模式显示文字	其它模式显示文字会有镜像效果			
	 //其中 6 模式为大部分液晶例程的默认显示方向  
		ILI9341_GramScan ( 6 );

		ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* 清屏，显示全黑 */
		delay_init();

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
		delay_init();
		
		EXTIX_Init();
		LD_Reset();
		uart_init(9600);
		nAsrStatus = LD_ASR_NONE;		//	初始状态：没有在作ASR
		SCS=0;
		//Oled_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
		//delay_init();	    	 //延时函数初始化	  
		uart_init(115200);	 	//串口初始化为9600
		

		
		#ifndef BUILTAP_TEST
		ESP8266_StaTcpServer_ConfigTest();                                             //对ESP8266进行配置 STA模式
		#else
		ESP8266_ApTcpServer_ConfigTest();                                              //对ESP8266进行配置 AP模式
		#endif
		
		
		
		LD3320_Init();
    IIC_Init();

			
			
			

    if(!maxim_max30102_reset())//复位 MAX30102
        printf("max30102_reset failed!\r\n");
    if(!maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy))//读取并清除状态寄存器
        printf("read_reg REG_INTR_STATUS_1 failed!\r\n");
    if(!maxim_max30102_init())//初始化MAX30102
        printf("max30102_init failed!\r\n");
    
    n_brightness = 0;
    un_min = 0x3FFFF;
    un_max = 0;
    
    n_ir_buffer_length = 500; //缓冲区长度为100存储以100sps运行的5秒样本
    
    printf("采集500个样本\r\n");
    //读取前500个样本，并确定信号范围
    for(i = 0; i < n_ir_buffer_length; i++)
    {
        while(max30102_INTPin == 1);   //等待MAX30102中断引脚拉低

        maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));  //从MAX30102 FIFO读取
            
        if(un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];    //更新信号最小值
        if(un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];    //更新信号最大值
            
        //printf("red=");
        //printf("%i", aun_red_buffer[i]);
        //printf(", ir=");
        //printf("%i\r\n", aun_ir_buffer[i]);

    }
    un_prev_data = aun_red_buffer[i];
    
    //计算前500个样本后的心率和血氧饱和度（样本的前5秒）
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    //从MAX30102连续取样。每1秒计算一次心率和血氧饱和度
		
		printf ( "\r\n野火 WF-ESP8266 WiFi模块测试例程\r\n" );                          //打印测试例程提示信息
	
  
		
		
    while(1)
    {
        int i;
        un_min = 0x3FFFF;
        un_max = 0;
			  
			  
			
				/*调用DHT11_Read_TempAndHumidity读取温湿度，若成功则输出该信息*/
			if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)
			{
        ILI9341_DispStringLine_EN(LINE(0),"<<Qian Ru Shi Da Zuo Ye>>");
        
        /* 显示温度 */
        sprintf(dispBuff,"Temperature : %d.%d ",DHT11_Data.temp_int, DHT11_Data.temp_deci);
        LCD_ClearLine(LINE(1));	/* 清除单行文字 */
        ILI9341_DispStringLine_EN(LINE(1),dispBuff);
        
        /* 显示湿度 */
        sprintf(dispBuff,"Humidity : %d.%d%%",DHT11_Data.humi_int, DHT11_Data.humi_deci);
        LCD_ClearLine(LINE(2));	/* 清除单行文字 */
        ILI9341_DispStringLine_EN(LINE(2),dispBuff);
			}			
			else
			{
        LCD_ClearLine(LINE(1));	/* 清除单行文字 */
        LCD_ClearLine(LINE(2));	/* 清除单行文字 */
				ILI9341_DispStringLine_EN(LINE(1),"DHT11 Temperature ERROR");
        ILI9341_DispStringLine_EN(LINE(2),"DHT11 Humidity ERROR");
			}
			
			
			
			/*调用LDR模块检测光强，并自动开关灯*/
			if (LDR_Test(LDR_GPIO_PORT,LDR_GPIO_PIN) == LDR_ON)
			{
				//LED2_OFF;    // 有光关灯
				if(Auto)
					GPIO_off();
				LCD_ClearLine(LINE(4));	/* 清除单行文字 */
				ILI9341_DispStringLine_EN(LINE(4),"Light is Good!");
			}
			else
			{
				//LED2_ON;   // 无光开灯
				if(Auto)
					GPIO_on();
				LCD_ClearLine(LINE(4));	/* 清除单行文字 */
				ILI9341_DispStringLine_EN(LINE(4),"Light is Bad!");
			}
			
			
			
			//血氧和心率采集计算       
        for(i = 100; i < 500; i++) //在计算心率之前采集100组样本。
        {
            aun_red_buffer[i-100] = aun_red_buffer[i];
            aun_ir_buffer[i-100] = aun_ir_buffer[i];
            
            //update the signal min and max
            if(un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];
            if(un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];
        }
				for(i = 400; i < 500; i++)
        {   
            un_prev_data = aun_red_buffer[i-1]; 
			      while(max30102_INTPin == 1);   //等待MAX30102中断引脚拉低
            maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));                
				}		
				if(Blood)
				{
				  printf("red=");
					printf("%i", aun_red_buffer[i]);
					printf(", ir=");
					printf("%i", aun_ir_buffer[i]);
					printf(", HR=%i, ", n_heart_rate); 
					sprintf(dispBuff,"HR :%i", n_heart_rate);
					LCD_ClearLine(LINE(10));	
					ILI9341_DispStringLine_EN(LINE(10),dispBuff);
					printf("HRvalid=%i, ", ch_hr_valid);
					printf("SpO2=%i, ", n_sp02);
					sprintf(dispBuff,"SpO2=%i, ", n_sp02);
					LCD_ClearLine(LINE(11));	
					ILI9341_DispStringLine_EN(LINE(11),dispBuff);
					printf("SPO2Valid=%i\r\n", ch_spo2_valid);
					maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
				}
				else
				{
					printf("none");
					LCD_ClearLine(LINE(10));
					LCD_ClearLine(LINE(11));
				}
					delay_ms(200);
			
				

	    printf("HRvalid=%i, ", ch_hr_valid);
			printf("SpO2=%i, ", n_sp02);
			ESP8266_CheckRecv_SendDataTest(ch_hr_valid,n_heart_rate,ch_spo2_valid,n_sp02); // ESP8266 处理并发送数据
			/*语音模块，识别命令，做出响应*/
			switch(nAsrStatus)
			{
				case LD_ASR_RUNING:
				case LD_ASR_ERROR:	
						 break;
				case LD_ASR_NONE:
				{
					nAsrStatus=LD_ASR_RUNING;
					if (RunASR()==0)	/*	启动一次ASR识别流程：ASR初始化，ASR添加关键词语，启动ASR运算*/
					{
						nAsrStatus = LD_ASR_ERROR;
					}
					break;
				}

				case LD_ASR_FOUNDOK: /*	一次ASR识别流程结束，去取ASR识别结果*/
				{
					nAsrRes = LD_GetResult();		/*获取结果*/												
					User_Modification(nAsrRes);
					nAsrStatus = LD_ASR_NONE;
					break;
				}
				case LD_ASR_FOUNDZERO:
				default:
				{
					nAsrStatus = LD_ASR_NONE;
					break;
				}
			}  
	}
}
 /***********************************************************
* 名    称：用户执行函数 
* 功    能：识别成功后，执行动作可在此进行修改 
* 入口参数： 无 
* 出口参数：无
* 说    明： 					 
**********************************************************/
void User_Modification(u8 dat)
{
	printf("开始");
	if(dat ==0)
	{
		flag=1;
		printf("收到\r\n");
	}
	else if(flag)
	{
		flag=0;
		switch(nAsrRes)		   /*对结果执行相关操作,客户修改*/
		{		
			case CODE_1KL1:	 /*命令开灯�*/
			{
				Auto = 0;	
				GPIO_on();
				printf("\"开灯\"识别成功\r\n"); /*text.....*/
				break;
			}
			case CODE_1KL2:		/*命令关灯*/
			{ 
				Auto = 0;	
				GPIO_off();
				printf("\"关灯\"识别成功\r\n"); /*text.....*/
				break;
			}
			case CODE_1KL3:	 /*命令“自动控制*/
			{	
				Auto = 1;
				printf("\"自动控制\"识别成功\r\n"); /*text.....*/
												break;}
			case CODE_1KL4:		/*命令”测心率血氧“*/	
			{ 	Blood = 1;
					printf("\"测心率血氧\"识别成功\r\n"); /*text.....*/
												break;}
			case CODE_1KL5:		/*命令结束测量*/	
			{ 	Blood = 0;
					printf("\"结束测量\"识别成功\r\n"); /*text.....*/
												break;}
			
			default:break;
		}
	}
	else 	
	{
		printf("请说出一级口令\r\n"); /*text.....*/	
	}
	
}
