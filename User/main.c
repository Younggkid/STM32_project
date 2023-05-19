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

u8 Auto = 1; //¾ö¶¨ÊÇ·ñ×Ô¶¯¿ª¹ØµÆ
u8 Blood = 0; //¾ö¶¨ÊÇ·ñ²âÁ¿ÑªÑõ


 int main(void)
 { 
	  char dispBuff[100];	 
		DHT11_Data_TypeDef DHT11_Data;
		int i;
		float f_temp;
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int32_t n_brightness;
	 	/* ³õÊ¼»¯ÏµÍ³¶¨Ê±Æ÷ */
		SysTick_Init();
	 
	 /*ÍâÖÃLEDµÆµÄ³õÊ¼»¯*/
	  GPIO_init();
	 
	 	USARTx_Config();
		CPU_TS_TmrInit();
		ESP8266_Init ();                                                               //³õÊ¼»¯WiFiÄ£¿éÊ¹ÓÃµÄ½Ó¿ÚºÍÍâÉè
		DHT11_Init ();
		Beep_Init ();
		
		//LCD ³õÊ¼»¯
		ILI9341_Init (); 
	 //ÆäÖĞ0¡¢3¡¢5¡¢6 Ä£Ê½ÊÊºÏ´Ó×óÖÁÓÒÏÔÊ¾ÎÄ×Ö£¬
	 //²»ÍÆ¼öÊ¹ÓÃÆäËüÄ£Ê½ÏÔÊ¾ÎÄ×Ö	ÆäËüÄ£Ê½ÏÔÊ¾ÎÄ×Ö»áÓĞ¾µÏñĞ§¹û			
	 //ÆäÖĞ 6 Ä£Ê½Îª´ó²¿·ÖÒº¾§Àı³ÌµÄÄ¬ÈÏÏÔÊ¾·½Ïò  
		ILI9341_GramScan ( 6 );

		ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* ÇåÆÁ£¬ÏÔÊ¾È«ºÚ */
		delay_init();

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ÉèÖÃÖĞ¶ÏÓÅÏÈ¼¶·Ö×éÎª×é2£º2Î»ÇÀÕ¼ÓÅÏÈ¼¶£¬2Î»ÏìÓ¦ÓÅÏÈ¼¶
		delay_init();
		
		EXTIX_Init();
		LD_Reset();
		uart_init(9600);
		nAsrStatus = LD_ASR_NONE;		//	³õÊ¼×´Ì¬£ºÃ»ÓĞÔÚ×÷ASR
		SCS=0;
		//Oled_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// ÉèÖÃÖĞ¶ÏÓÅÏÈ¼¶·Ö×é2
		//delay_init();	    	 //ÑÓÊ±º¯Êı³õÊ¼»¯	  
		uart_init(115200);	 	//´®¿Ú³õÊ¼»¯Îª9600
		

		
		#ifndef BUILTAP_TEST
		ESP8266_StaTcpServer_ConfigTest();                                             //¶ÔESP8266½øĞĞÅäÖÃ STAÄ£Ê½
		#else
		ESP8266_ApTcpServer_ConfigTest();                                              //¶ÔESP8266½øĞĞÅäÖÃ APÄ£Ê½
		#endif
		
		
		
		LD3320_Init();
    IIC_Init();

			
			
			

    if(!maxim_max30102_reset())//¸´Î» MAX30102
        printf("max30102_reset failed!\r\n");
    if(!maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy))//¶ÁÈ¡²¢Çå³ı×´Ì¬¼Ä´æÆ÷
        printf("read_reg REG_INTR_STATUS_1 failed!\r\n");
    if(!maxim_max30102_init())//³õÊ¼»¯MAX30102
        printf("max30102_init failed!\r\n");
    
    n_brightness = 0;
    un_min = 0x3FFFF;
    un_max = 0;
    
    n_ir_buffer_length = 500; //»º³åÇø³¤¶ÈÎª100´æ´¢ÒÔ100spsÔËĞĞµÄ5ÃëÑù±¾
    
    printf("²É¼¯500¸öÑù±¾\r\n");
    //¶ÁÈ¡Ç°500¸öÑù±¾£¬²¢È·¶¨ĞÅºÅ·¶Î§
    for(i = 0; i < n_ir_buffer_length; i++)
    {
        while(max30102_INTPin == 1);   //µÈ´ıMAX30102ÖĞ¶ÏÒı½ÅÀ­µÍ

        maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));  //´ÓMAX30102 FIFO¶ÁÈ¡
            
        if(un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];    //¸üĞÂĞÅºÅ×îĞ¡Öµ
        if(un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];    //¸üĞÂĞÅºÅ×î´óÖµ
            
        //printf("red=");
        //printf("%i", aun_red_buffer[i]);
        //printf(", ir=");
        //printf("%i\r\n", aun_ir_buffer[i]);

    }
    un_prev_data = aun_red_buffer[i];
    
    //¼ÆËãÇ°500¸öÑù±¾ºóµÄĞÄÂÊºÍÑªÑõ±¥ºÍ¶È£¨Ñù±¾µÄÇ°5Ãë£©
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    //´ÓMAX30102Á¬ĞøÈ¡Ñù¡£Ã¿1Ãë¼ÆËãÒ»´ÎĞÄÂÊºÍÑªÑõ±¥ºÍ¶È
		
		printf ( "\r\nÒ°»ğ WF-ESP8266 WiFiÄ£¿é²âÊÔÀı³Ì\r\n" );                          //´òÓ¡²âÊÔÀı³ÌÌáÊ¾ĞÅÏ¢
	
  
		
		
    while(1)
    {
        int i;
        un_min = 0x3FFFF;
        un_max = 0;
			ESP8266_CheckRecv_SendDataTest(); // ESP8266 ´¦Àí²¢·¢ËÍÊı¾İ
			
				/*µ÷ÓÃDHT11_Read_TempAndHumidity¶ÁÈ¡ÎÂÊª¶È£¬Èô³É¹¦ÔòÊä³ö¸ÃĞÅÏ¢*/
			if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)
			{
        ILI9341_DispStringLine_EN(LINE(0),"<<Qian Ru Shi Da Zuo Ye>>");
        
        /* ÏÔÊ¾ÎÂ¶È */
        sprintf(dispBuff,"Temperature : %d.%d ",DHT11_Data.temp_int, DHT11_Data.temp_deci);
        LCD_ClearLine(LINE(1));	/* Çå³ıµ¥ĞĞÎÄ×Ö */
        ILI9341_DispStringLine_EN(LINE(1),dispBuff);
        
        /* ÏÔÊ¾Êª¶È */
        sprintf(dispBuff,"Humidity : %d.%d%%",DHT11_Data.humi_int, DHT11_Data.humi_deci);
        LCD_ClearLine(LINE(2));	/* Çå³ıµ¥ĞĞÎÄ×Ö */
        ILI9341_DispStringLine_EN(LINE(2),dispBuff);
			}			
			else
			{
        LCD_ClearLine(LINE(1));	/* Çå³ıµ¥ĞĞÎÄ×Ö */
        LCD_ClearLine(LINE(2));	/* Çå³ıµ¥ĞĞÎÄ×Ö */
				ILI9341_DispStringLine_EN(LINE(1),"DHT11 Temperature ERROR");
        ILI9341_DispStringLine_EN(LINE(2),"DHT11 Humidity ERROR");
			}
			
			
			
			/*µ÷ÓÃLDRÄ£¿é¼ì²â¹âÇ¿£¬²¢×Ô¶¯¿ª¹ØµÆ*/
			if (LDR_Test(LDR_GPIO_PORT,LDR_GPIO_PIN) == LDR_ON)
			{
				//LED2_OFF;    // ÓĞ¹â¹ØµÆ
				if(Auto)
					GPIO_off();
				LCD_ClearLine(LINE(4));	/* Çå³ıµ¥ĞĞÎÄ×Ö */
				ILI9341_DispStringLine_EN(LINE(4),"Light is Good!");
			}
			else
			{
				//LED2_ON;   // ÎŞ¹â¿ªµÆ
				if(Auto)
					GPIO_on();
				LCD_ClearLine(LINE(4));	/* Çå³ıµ¥ĞĞÎÄ×Ö */
				ILI9341_DispStringLine_EN(LINE(4),"Light is Bad!");
			}
			
			
			
			//ÑªÑõºÍĞÄÂÊ²É¼¯¼ÆËã       
        for(i = 100; i < 500; i++) //ÔÚ¼ÆËãĞÄÂÊÖ®Ç°²É¼¯100×éÑù±¾¡£
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
			      while(max30102_INTPin == 1);   //µÈ´ıMAX30102ÖĞ¶ÏÒı½ÅÀ­µÍ
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
			
			/*ÓïÒôÄ£¿é£¬Ê¶±ğÃüÁî£¬×ö³öÏìÓ¦*/
			switch(nAsrStatus)
			{
				case LD_ASR_RUNING:
				case LD_ASR_ERROR:	
						 break;
				case LD_ASR_NONE:
				{
					nAsrStatus=LD_ASR_RUNING;
					if (RunASR()==0)	/*	Æô¶¯Ò»´ÎASRÊ¶±ğÁ÷³Ì£ºASR³õÊ¼»¯£¬ASRÌí¼Ó¹Ø¼ü´ÊÓï£¬Æô¶¯ASRÔËËã*/
					{
						nAsrStatus = LD_ASR_ERROR;
					}
					break;
				}

				case LD_ASR_FOUNDOK: /*	Ò»´ÎASRÊ¶±ğÁ÷³Ì½áÊø£¬È¥È¡ASRÊ¶±ğ½á¹û*/
				{
					nAsrRes = LD_GetResult();		/*»ñÈ¡½á¹û*/												
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
* Ãû    ³Æ£ºÓÃ»§Ö´ĞĞº¯Êı 
* ¹¦    ÄÜ£ºÊ¶±ğ³É¹¦ºó£¬Ö´ĞĞ¶¯×÷¿ÉÔÚ´Ë½øĞĞĞŞ¸Ä 
* Èë¿Ú²ÎÊı£º ÎŞ 
* ³ö¿Ú²ÎÊı£ºÎŞ
* Ëµ    Ã÷£º 					 
**********************************************************/
void User_Modification(u8 dat)
{
	if(dat ==0)
	{
		flag=1;
		printf("ÊÕµ½\r\n");
	}
	else if(flag)
	{
		flag=0;
		switch(nAsrRes)		   /*¶Ô½á¹ûÖ´ĞĞÏà¹Ø²Ù×÷,¿Í»§ĞŞ¸Ä*/
		{		
			case CODE_1KL1:	 /*ÃüÁî¿ªµÆ±*/
			{
				Auto = 0;	
				GPIO_on();
				printf("\"¿ªµÆ\"Ê¶±ğ³É¹¦\r\n"); /*text.....*/
				break;
			}
			case CODE_1KL2:		/*ÃüÁî¹ØµÆ*/
			{ 
				Auto = 0;	
				GPIO_off();
				printf("\"¹ØµÆ\"Ê¶±ğ³É¹¦\r\n"); /*text.....*/
				break;
			}
			case CODE_1KL3:	 /*ÃüÁî¡°×Ô¶¯¿ØÖÆ*/
			{	
				Auto = 1;
				printf("\"×Ô¶¯¿ØÖÆ\"Ê¶±ğ³É¹¦\r\n"); /*text.....*/
												break;}
			case CODE_1KL4:		/*ÃüÁî¡±²âĞÄÂÊÑªÑõ¡°*/	
			{ 	Blood = 1;
					printf("\"²âĞÄÂÊÑªÑõ\"Ê¶±ğ³É¹¦\r\n"); /*text.....*/
												break;}
			case CODE_1KL5:		/*ÃüÁî½áÊø²âÁ¿*/	
			{ 	Blood = 0;
					printf("\"½áÊø²âÁ¿\"Ê¶±ğ³É¹¦\r\n"); /*text.....*/
												break;}
			
			default:break;
		}
	}
	else 	
	{
		printf("ÇëËµ³öÒ»¼¶¿ÚÁî\r\n"); /*text.....*/	
	}
	
}
