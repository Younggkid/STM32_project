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

u8 Auto = 1; //�����Ƿ��Զ����ص�
u8 Blood = 0; //�����Ƿ����Ѫ��


 int main(void)
 { 
	  char dispBuff[100];	 
		DHT11_Data_TypeDef DHT11_Data;
		int i;
		float f_temp;
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int32_t n_brightness;
	 	/* ��ʼ��ϵͳ��ʱ�� */
		SysTick_Init();
	 
	 /*����LED�Ƶĳ�ʼ��*/
	  GPIO_init();
	 
	 	USARTx_Config();
		CPU_TS_TmrInit();
		ESP8266_Init ();                                                               //��ʼ��WiFiģ��ʹ�õĽӿں�����
		DHT11_Init ();
		Beep_Init ();
		
		//LCD ��ʼ��
		ILI9341_Init (); 
	 //����0��3��5��6 ģʽ�ʺϴ���������ʾ���֣�
	 //���Ƽ�ʹ������ģʽ��ʾ����	����ģʽ��ʾ���ֻ��о���Ч��			
	 //���� 6 ģʽΪ�󲿷�Һ�����̵�Ĭ����ʾ����  
		ILI9341_GramScan ( 6 );

		ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* ��������ʾȫ�� */
		delay_init();

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
		delay_init();
		
		EXTIX_Init();
		LD_Reset();
		uart_init(9600);
		nAsrStatus = LD_ASR_NONE;		//	��ʼ״̬��û������ASR
		SCS=0;
		//Oled_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
		//delay_init();	    	 //��ʱ������ʼ��	  
		uart_init(115200);	 	//���ڳ�ʼ��Ϊ9600
		

		
		#ifndef BUILTAP_TEST
		ESP8266_StaTcpServer_ConfigTest();                                             //��ESP8266�������� STAģʽ
		#else
		ESP8266_ApTcpServer_ConfigTest();                                              //��ESP8266�������� APģʽ
		#endif
		
		
		
		LD3320_Init();
    IIC_Init();

			
			
			

    if(!maxim_max30102_reset())//��λ MAX30102
        printf("max30102_reset failed!\r\n");
    if(!maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy))//��ȡ�����״̬�Ĵ���
        printf("read_reg REG_INTR_STATUS_1 failed!\r\n");
    if(!maxim_max30102_init())//��ʼ��MAX30102
        printf("max30102_init failed!\r\n");
    
    n_brightness = 0;
    un_min = 0x3FFFF;
    un_max = 0;
    
    n_ir_buffer_length = 500; //����������Ϊ100�洢��100sps���е�5������
    
    printf("�ɼ�500������\r\n");
    //��ȡǰ500����������ȷ���źŷ�Χ
    for(i = 0; i < n_ir_buffer_length; i++)
    {
        while(max30102_INTPin == 1);   //�ȴ�MAX30102�ж���������

        maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));  //��MAX30102 FIFO��ȡ
            
        if(un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];    //�����ź���Сֵ
        if(un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];    //�����ź����ֵ
            
        //printf("red=");
        //printf("%i", aun_red_buffer[i]);
        //printf(", ir=");
        //printf("%i\r\n", aun_ir_buffer[i]);

    }
    un_prev_data = aun_red_buffer[i];
    
    //����ǰ500������������ʺ�Ѫ�����Ͷȣ�������ǰ5�룩
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    //��MAX30102����ȡ����ÿ1�����һ�����ʺ�Ѫ�����Ͷ�
		
		printf ( "\r\nҰ�� WF-ESP8266 WiFiģ���������\r\n" );                          //��ӡ����������ʾ��Ϣ
	
  
		
		
    while(1)
    {
        int i;
        un_min = 0x3FFFF;
        un_max = 0;
			ESP8266_CheckRecv_SendDataTest(); // ESP8266 ������������
			
				/*����DHT11_Read_TempAndHumidity��ȡ��ʪ�ȣ����ɹ����������Ϣ*/
			if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)
			{
        ILI9341_DispStringLine_EN(LINE(0),"<<Qian Ru Shi Da Zuo Ye>>");
        
        /* ��ʾ�¶� */
        sprintf(dispBuff,"Temperature : %d.%d ",DHT11_Data.temp_int, DHT11_Data.temp_deci);
        LCD_ClearLine(LINE(1));	/* ����������� */
        ILI9341_DispStringLine_EN(LINE(1),dispBuff);
        
        /* ��ʾʪ�� */
        sprintf(dispBuff,"Humidity : %d.%d%%",DHT11_Data.humi_int, DHT11_Data.humi_deci);
        LCD_ClearLine(LINE(2));	/* ����������� */
        ILI9341_DispStringLine_EN(LINE(2),dispBuff);
			}			
			else
			{
        LCD_ClearLine(LINE(1));	/* ����������� */
        LCD_ClearLine(LINE(2));	/* ����������� */
				ILI9341_DispStringLine_EN(LINE(1),"DHT11 Temperature ERROR");
        ILI9341_DispStringLine_EN(LINE(2),"DHT11 Humidity ERROR");
			}
			
			
			
			/*����LDRģ�����ǿ�����Զ����ص�*/
			if (LDR_Test(LDR_GPIO_PORT,LDR_GPIO_PIN) == LDR_ON)
			{
				//LED2_OFF;    // �й�ص�
				if(Auto)
					GPIO_off();
				LCD_ClearLine(LINE(4));	/* ����������� */
				ILI9341_DispStringLine_EN(LINE(4),"Light is Good!");
			}
			else
			{
				//LED2_ON;   // �޹⿪��
				if(Auto)
					GPIO_on();
				LCD_ClearLine(LINE(4));	/* ����������� */
				ILI9341_DispStringLine_EN(LINE(4),"Light is Bad!");
			}
			
			
			
			//Ѫ�������ʲɼ�����       
        for(i = 100; i < 500; i++) //�ڼ�������֮ǰ�ɼ�100��������
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
			      while(max30102_INTPin == 1);   //�ȴ�MAX30102�ж���������
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
			
			/*����ģ�飬ʶ�����������Ӧ*/
			switch(nAsrStatus)
			{
				case LD_ASR_RUNING:
				case LD_ASR_ERROR:	
						 break;
				case LD_ASR_NONE:
				{
					nAsrStatus=LD_ASR_RUNING;
					if (RunASR()==0)	/*	����һ��ASRʶ�����̣�ASR��ʼ����ASR��ӹؼ��������ASR����*/
					{
						nAsrStatus = LD_ASR_ERROR;
					}
					break;
				}

				case LD_ASR_FOUNDOK: /*	һ��ASRʶ�����̽�����ȥȡASRʶ����*/
				{
					nAsrRes = LD_GetResult();		/*��ȡ���*/												
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
* ��    �ƣ��û�ִ�к��� 
* ��    �ܣ�ʶ��ɹ���ִ�ж������ڴ˽����޸� 
* ��ڲ����� �� 
* ���ڲ�������
* ˵    ���� 					 
**********************************************************/
void User_Modification(u8 dat)
{
	if(dat ==0)
	{
		flag=1;
		printf("�յ�\r\n");
	}
	else if(flag)
	{
		flag=0;
		switch(nAsrRes)		   /*�Խ��ִ����ز���,�ͻ��޸�*/
		{		
			case CODE_1KL1:	 /*����Ʊ*/
			{
				Auto = 0;	
				GPIO_on();
				printf("\"����\"ʶ��ɹ�\r\n"); /*text.....*/
				break;
			}
			case CODE_1KL2:		/*����ص�*/
			{ 
				Auto = 0;	
				GPIO_off();
				printf("\"�ص�\"ʶ��ɹ�\r\n"); /*text.....*/
				break;
			}
			case CODE_1KL3:	 /*����Զ�����*/
			{	
				Auto = 1;
				printf("\"�Զ�����\"ʶ��ɹ�\r\n"); /*text.....*/
												break;}
			case CODE_1KL4:		/*���������Ѫ����*/	
			{ 	Blood = 1;
					printf("\"������Ѫ��\"ʶ��ɹ�\r\n"); /*text.....*/
												break;}
			case CODE_1KL5:		/*�����������*/	
			{ 	Blood = 0;
					printf("\"��������\"ʶ��ɹ�\r\n"); /*text.....*/
												break;}
			
			default:break;
		}
	}
	else 	
	{
		printf("��˵��һ������\r\n"); /*text.....*/	
	}
	
}
