

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "parameters.h"

extern unsigned char data_collection;
unsigned int start_frame;
extern unsigned char data_ok;

extern char KWH_Factor[]; //ADD
//***********************************
unsigned int kwh_counter=0;

unsigned long Watt_Hour_Cum = 0;
unsigned int Watt_Hour = 0;
unsigned long Watt_Sec = 0;

long PV_POWER=0;

extern char CUMKWH_FLASH[];


unsigned long pv_current;
unsigned long pv_voltage;
extern unsigned char start_counter;
//extern volatile unsigned char value_change_sign;
extern char flash_memory[400]; 
extern void strcat_flash(void);
extern void write_in_flash(char *);




//***********************************

#define      LED_OFF                  GPIO_SetBits(GPIOB,GPIO_Pin_12);
#define      LED_ON                   GPIO_ResetBits(GPIOB,GPIO_Pin_12);

#define      MESSAGE_OFF              GPIO_SetBits(GPIOB,GPIO_Pin_13);
#define      MESSAGE_ON     		 			GPIO_ResetBits(GPIOB,GPIO_Pin_13);

#define      GSM_MODE          0x00
#define      GPRS_MODE         0x01

typedef enum {AUTOMATIC = 0,GPRS_ONLY,GSM_ONLY,BLOCK_COMM}MODE;

volatile static unsigned int SUMadc;
volatile unsigned char dataindex = 0;
volatile unsigned char adc_in;
volatile unsigned char DMA_TC_FLAG,EXT_INT_FLAG;
volatile unsigned char cable_connection;
volatile unsigned char SMS_Received,IP_Received;
volatile unsigned char comm_error=1,one_flag=1;
volatile unsigned char comm_ok=0;
volatile unsigned int AvgSUMadc;
volatile unsigned int wtchdg_count;
volatile unsigned int _delay;
volatile unsigned long int msd_sec_count, csq_sec_count,gtry_sec_count,live_sec_count;
volatile unsigned long int RTC_sec_count,csq_sec_count,gtry_sec_count,live_sec_count,msd_sec_count;    
volatile char Enable_Soft_Int;
volatile  u16 adc;
volatile  u32 garbage_timeout;

struct BYTE1
{
	volatile unsigned char Rx0:1;
	volatile unsigned char Rx1:1;
	volatile unsigned char Rx2:1;
	volatile unsigned char Rx3:1;
	volatile unsigned char Rx4:1;
	volatile unsigned char Rx5:1;
	volatile unsigned char Rx6:1;
	volatile unsigned char Rx7:1;

	volatile unsigned char IP0:1;
	volatile unsigned char IP1:1;
	volatile unsigned char IP2:1;
	volatile unsigned char IP3:1;
	volatile unsigned char IP4:1;
	volatile unsigned char IP5:1;
	volatile unsigned char IP6:1;
	volatile unsigned char IP7:1;
};

struct Time4Alarm
{
	char Time[5];
	unsigned int time_event;
	unsigned int independent_counter;
	unsigned int count;
};
struct typeconverter
{
	char xcon[2];
	char No_of_Alerts[2];
	char LogTimeGprs[2];
	char RTU_OP_MODE[2];
	unsigned int xcoonh;
	int NoAlerts;
	unsigned int TimeGprs;
	unsigned int OPER_MODE;
};

union CumulativeKWH
{
   unsigned long CUM_KWH;
   char bytes[4];
};

extern union CumulativeKWH CurrentPosition;

struct BYTE1 buff;
oneP datareceived,finaldata;
extern struct typeconverter converter;			
extern struct Time4Alarm Event[32];
extern struct date_PCU before_RTC,after_RTC;
extern struct MODBUS_ENERGYMETER ThreePhase_EnergyMeter;

char *eSMS,*eIP;
char ReceBuffer[200];

//static unsigned char commands[13] = {21,20,11,10,13,16,15,18,14,12,9,23,22};                 // Query Command COMMENT1

u8 u8_TempData=0;
u16 u16_tempobcntr=0;
			 
unsigned int Alert0;
unsigned int Alert1;
unsigned int Alert2;
unsigned int Alert3;
unsigned int Alert4;
unsigned int Alert5;
unsigned int Alert6;
unsigned int Alert7;
unsigned int Alert8;
unsigned int Alert9;
unsigned int Alert10;
unsigned int Alert11;
unsigned int in;
unsigned int local = 0;
unsigned int configurable;
unsigned int disp_count;
unsigned int disp_count1;
unsigned int disp_count2;
unsigned char disp_once;
unsigned char Flag=0;
unsigned char disp_once1;
unsigned char disp_once2;
unsigned char SetAlarmIndication;
unsigned char ittr_count=0;
unsigned char TimeDisplay=0;
unsigned char comparison_flag=0;
//unsigned char SystemBatteryVoltage_LSB; //COMMENT2
//unsigned char SystemBatteryVoltage_MSB; //COMMENT3
unsigned char prev_conn=0;
unsigned char prev_conn_KWH=0;
unsigned char sync_flag=0;
unsigned char  RECEIVEDDATA[200];

extern char Rx0Buff[400];
extern char Rx1Buff[400];
extern char Rx2Buff[400];
extern char Rx3Buff[400];
extern char Rx4Buff[400];
extern char Rx5Buff[400];
extern char Rx6Buff[400];
extern char Rx7Buff[400];
extern char IP0Buff[400];
extern char IP1Buff[400];
extern char IP2Buff[400];
extern char IP3Buff[400];
extern char IP4Buff[400];
extern char IP5Buff[400];
extern char IP6Buff[400];
extern char IP7Buff[400];
extern char ConnMode;
extern char Rx1Buffer[400];
extern char RTU_MODE;
extern unsigned char AT_ack;
extern unsigned char __in;
extern unsigned char  RxBuffer[200];
extern unsigned char Rx2Buffer[200]; //ADD12
extern unsigned int No_of_bytes_to_read;
extern unsigned int time_out_ms;
extern unsigned int reconfig;
extern unsigned int LED_INDICATOR;
extern unsigned int Block_LED;
extern unsigned int SD_Flag;
extern volatile unsigned char checkedOK;
extern volatile unsigned char Counter;
extern volatile unsigned char Counter;
extern volatile unsigned char RESET_COUNT_FLAG;
extern volatile unsigned char RESET_FLAG;
extern volatile unsigned char Data_Log_Time, Sd_Write, CSQ_time_count; 	
extern volatile unsigned char GPRS_retry, Data_Log_Time, Sd_Write, CSQ_time_count, Send_Status;
extern volatile unsigned int delay_count;
extern volatile unsigned int time_count,CSQ_timeout;
extern volatile unsigned int RTC_time_capture;
extern volatile unsigned int delay_ADC;
extern volatile u32 NewAlarm;
extern volatile u32 NewCounter;
extern int at_int;

extern unsigned int modbusRTUcrc(unsigned char *crcbuffer,unsigned char length);
extern void start_DMA_access1(unsigned int );
extern void start_DMA_access1_receive(unsigned int,char *);
extern void SMS_control(void);
extern void put_date(struct date_PCU *,struct date_PCU *);
extern void start_DMA_access2(unsigned int );
extern void BuffMAPOnlineUPS(struct OnlineUPS *,unsigned char *);
extern void BuffMAP_MPPTSCC(struct OnlineUPS *,unsigned char *); //ADD15
extern void ManageBuffer(struct OnlineUPS *,unsigned int);
//extern void ManageBuffer(struct MPPTSCC *,struct MPPTSCC *,struct MPPTSCC *,unsigned int); //ADD14
//extern void reset_ups(struct OnlineUPS *);
extern void reset_SCC(struct OnlineUPS *);//ADD
extern unsigned char received_msgtype(char *);
extern char comparison(struct date_PCU *, struct date_PCU *);

extern void read_Data4mMPPTSCC(void); //ADD

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  LED_OFF
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if((start_counter)&&(data_ok))
	{kwh_counter++;
	if(kwh_counter==1000)
	{
//pv_current = 200;
//(unsigned long int)pv_voltage=1000;
         PV_POWER=(((unsigned long int)pv_current * (unsigned long int)pv_voltage));
         Watt_Sec  = PV_POWER;
         Watt_Hour  = Watt_Sec/36;

         Watt_Hour_Cum = Watt_Hour_Cum + Watt_Hour;
         if(Watt_Hour_Cum >= 1000000)  
						{
  	           Watt_Hour_Cum = Watt_Hour_Cum - 1000000;     
	             CurrentPosition.CUM_KWH = CurrentPosition.CUM_KWH + 1;
							 PV_POWER=0;
               //value_change_sign = 1;
							if(CurrentPosition.CUM_KWH)
							{
							    if(KWH_Factor[0] == 2)
							    {	
							       if((CurrentPosition.CUM_KWH % 2) == 0)
							       {prev_conn_KWH =1;}
						      }  
							    else if(KWH_Factor[0] == 5)
									{	
                     if((CurrentPosition.CUM_KWH % 5) == 0)
							       {prev_conn_KWH =1;}
									}
							    else if(KWH_Factor[0] == 10)
									{	
                     if((CurrentPosition.CUM_KWH % 10) == 0)
							       {prev_conn_KWH =1;}											 
									 }	 										 
									 	 
							}
							
							strcat_flash();
					    write_in_flash(flash_memory);
							 
					 }
						kwh_counter=0;
}
}
	NewCounter++;
	if(NewCounter==24*60*60*1000)
	{
		reconfig=1;
	}
	if(Counter)
	{
		if(disp_count++ > 1000)
		{
			disp_count = 0;
			if(disp_once == 0)
			{
				switch(converter.OPER_MODE)
				{
					case AUTOMATIC:
												LED_ON 
												break;

					case GPRS_ONLY:
												LED_ON
												break;

					case GSM_ONLY:
												Data_Log_Time = RESET;
												LED_ON 
												break;

					default:
									Data_Log_Time = RESET;
									LED_ON 
									MESSAGE_ON
									break;
				}
				disp_once = 1;	
			}
			else
			{
				disp_once = 0;	
				switch(converter.OPER_MODE)
				{
					case AUTOMATIC:
												if(RTU_MODE == GPRS_MODE)
												LED_OFF
												else if(RTU_MODE == GSM_MODE)
												LED_ON
												break;

					case GPRS_ONLY:
												if(RTU_MODE == GPRS_MODE)
												LED_OFF
												break;

					case GSM_ONLY:
												Data_Log_Time = RESET;
												if(RTU_MODE == GSM_MODE)
												LED_ON
												break;

					default:
									if(Block_LED)
									{
										LED_ON 
										MESSAGE_ON
									}
									else
									{
										LED_OFF
										MESSAGE_OFF
									}
									Data_Log_Time = RESET;
									break;
				}
			}
		}			
	}
	if(LED_INDICATOR == 1)
	{
		if(disp_count1++ > 200)
		{
			disp_count1 = 0;
			if(disp_once1 == 0)
			{
				MESSAGE_ON
				disp_once1 = 1;	
			}
			else
			{
				disp_once1 = 0;
				MESSAGE_OFF
			}
		}
	}

	if(csq_sec_count >= ((Max_CSQ_Interval*60)*1000))    
	{	
		csq_sec_count = 0;
		CSQ_time_count = 1;     
	}
	else 
		csq_sec_count++;

	if(gtry_sec_count >= ((Conn_Try_Interval*60)*1000))    
	{	
		gtry_sec_count = 0;
		GPRS_retry = 1; 
	}
	else 
		gtry_sec_count++;

	if(live_sec_count >= ((Live_Status_Interval*60)*1000))   
	{	
		live_sec_count = 0;
		Send_Status = 1; 
	}
	else 
		live_sec_count++;

	if(delay_count != 0)
	{
		--delay_count;
	}
	if(time_count != 0)
	{
		--time_count;
	}
	if(CSQ_timeout != 0)
	{
		--CSQ_timeout;
		if(CSQ_timeout == 1)
			at_int =  0;              
	}
	
	Decrement_TimingDelay();
}
/******************************************************/
void DMA1_Channel5_IRQHandler(void)
{
	char tfr_rx[10],g,f; 

	DMA_ClearFlag(DMA1_FLAG_TC5);
	DMA_TC_FLAG = 1;         
	DMA_ClearFlag(DMA1_FLAG_TC5);  
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(tfr_rx,'\0',10);
	
	for(f=2,g=0;g<5;g++,f++)
	{
		tfr_rx[g] = Rx1Buffer[f];	
	}
  
	garbage_timeout = 0xFFFFFF;
	if(received_msgtype(tfr_rx) != 99)
	{
		at_int = 0;  
			
		switch(IP_Received)
		{
			case 0:
							memset(IP0Buff,'\0',400);
							eIP=&IP0Buff[5]; 	
							strcat(IP0Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP0Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP0 = 1;
							break;
							
			case 1:
							memset(IP1Buff,'\0',400);
							eIP=&IP1Buff[5];   
							strcat(IP1Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP1Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP1 = 1;
							break;
							
			case 2:
							memset(IP2Buff,'\0',400);
							eIP=&IP2Buff[5];
							strcat(IP2Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP2Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP2 = 1;
							break;
							
			case 3:
							memset(IP3Buff,'\0',400);
							eIP=&IP3Buff[5]; 
							strcat(IP3Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP3Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP3 = 1;
							break;
							
			case 4:
							memset(IP4Buff,'\0',400);
							eIP=&IP4Buff[5];  
							strcat(IP4Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP4Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP4 = 1;
							break;
			
			case 5:
							memset(IP5Buff,'\0',400);
							eIP=&IP5Buff[5]; 
							strcat(IP5Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP5Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP5 = 1;
							break;
			
			case 6:
							memset(IP6Buff,'\0',400);
							eIP=&IP6Buff[5];  
							strcat(IP6Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP6Buff[strlen(Rx1Buffer)]);
							do
							{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP6 = 1;
							break;
			
			case 7:
							memset(IP7Buff,'\0',400);
							eIP=&IP7Buff[5];  
							strcat(IP7Buff,Rx1Buffer);
							start_DMA_access1_receive(400,&IP7Buff[strlen(Rx1Buffer)]);
							do
								{	
								while(*eIP == '\0' && (garbage_timeout-- > 10));
								if(*eIP++ == '\r')
								{
									while(*eIP != '\n' && (garbage_timeout-- > 10));
									break;
								}
							}while(garbage_timeout > 10);
							if((garbage_timeout > 10))
							buff.IP7 = 1;
							break;
							
			default:
							break;
		}
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);	

		if(IP_Received >= 7)
		{
			IP_Received = 0;
		}
		else
		{
			IP_Received++;
		}
	}
	else
	{
		eIP=&Rx1Buffer[7];
		start_DMA_access1_receive(400,&Rx1Buffer[8]);
		
		while(*eIP != '\0' && garbage_timeout-- > 0);
		
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);	
	}  
}
/******************************************************/
void DMA1_Channel6_IRQHandler(void)
{
	cable_connection=1;
	DMA_ClearFlag(DMA1_FLAG_TC6);
} 
/******************************************************/
void DMA1_Channel1_IRQHandler(void)
{ 
	DMA_ClearFlag(DMA1_FLAG_TC1);
}
/******************************************************/
void DMA1_Channel3_IRQHandler(void)
{
	unsigned int j;
	unsigned int receivedCRC,calculatedCRC;
	unsigned int shift_buffer;

	// Add instructions here
	// This calculates the CRC value of the received data
	for(in=0;in<(5+(2*No_of_bytes_to_read));in++)
	{
		RECEIVEDDATA[in]=RxBuffer[in];  // Only datas are copied into another buffer
	} 

	calculatedCRC=modbusRTUcrc(RECEIVEDDATA,in-2);
	j=(in-2);
	shift_buffer = (unsigned int)RxBuffer[j++];
	shift_buffer = shift_buffer << 8;
	receivedCRC = (shift_buffer + (unsigned int)RxBuffer[j]);
	if(receivedCRC == calculatedCRC)
	{
		checkedOK=1;
	}

	DMA_ClearFlag(DMA1_FLAG_TC3); 
}
/******************************************************/
void ADC1_2_IRQHandler(void)
{
	adc_in++;
	adc = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
	SUMadc = (unsigned int)adc + SUMadc;
	if(adc_in >= 16)
		{
			AvgSUMadc=SUMadc>>4;
			adc_in=0;
			SUMadc=0;      
		 }

	ADC_ClearFlag(ADC1,ADC_FLAG_JEOC);
	ADC1->SR = ADC1->SR&0xFFF7;
}
/******************************************************/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{

	}
	USART_ClearFlag(USART1,USART_IT_RXNE);
}
/******************************************************/
void TIM3_IRQHandler(void)                                      
{
	static unsigned char _step = 0;	

	local++;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) // if UIF flag is set
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);

		if(_step == 0)
		{
			cable_connection = 0;
			//DMA_Cmd(DMA1_Channel6, DISABLE); 
			USART_GetFlagStatus(USART2,USART_FLAG_ORE);
			USART_ReceiveData(USART2); 
			_step = 1;	
		}
		else if(_step == 1)
		{
			memset(Rx2Buffer,'\0',200);  
			start_DMA_access2(91);
			read_Data4mMPPTSCC(); //ADD16 //TOBECOMPLETED
			_step = 3;
		}
		else if(_step == 3)
		{
				 if(_delay++ >= 500)
				{
					_delay = 0;
	
					_step = 4;
				}
				else
				_step = 3;
				}
			
				else if(_step ==4)
				{		
				if(cable_connection == 1)
				{
					start_frame = Rx2Buffer[0]; 
					if (start_frame == 0)
					{
						ittr_count++;
						if(ittr_count>4)
						{ 
							ittr_count=0;
							if(data_collection == 0)
							{
								memset(Rx2Buffer,'\0',200);								
								data_collection =1;
								prev_conn=1;
								//comm_error =1;
							
								//one_flag=0;
								//reset_SCC(&finaldata);
							}
							reset_SCC(&datareceived);
							
							_step = 5; //7
						}
						else
						{
						
						_step = 5;  //7
						}
					}
					else
					{
						_step = 5;	
					}
				}
				else if( _delay++ >= 50)
				{
					_delay = 0;
					ittr_count++;
					if(ittr_count>4)
					{
						ittr_count=0;
						if(data_collection == 0)
						{	
							memset(Rx2Buffer,'\0',200);
							data_collection =1;
							prev_conn=1;
							//comm_error =1;
							
							//one_flag=0;
							//reset_SCC(&finaldata);
						}
						reset_SCC(&datareceived);
						
						_step = 5; //7
					}
					else
					{
						
						_step = 5; //7
					}
				}
	
		}
		else if(_step == 5)
		{
			DMA_Cmd(DMA1_Channel6, DISABLE); 
			BuffMAP_MPPTSCC(&datareceived,Rx2Buffer); //ADD
			finaldata	= datareceived;

			if(data_ok)
			{_step = 6;} 
else 
_step =7;	
		}
		else if(_step == 6)
		{
			//if(!data_collection) //ADD
			//{
				//if(comm_error == 1)
				//{	
				//	comm_error =0;
					//prev_conn=1;
					//one_flag=0;
				//}
				
				ManageBuffer(&finaldata,configurable);
				_step = 7;

		}
		else if(_step == 7)
		{
			if(local>=1000)
			{
			local=0;

			_step = 0;		  
			}
			else
				_step = 7;
		}
	}
}
/******************************************************/
void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0);
	at_int = 0;  
	if(RTU_MODE==GPRS_MODE && ConnMode == 1)
	{
		Enable_Soft_Int = 1;
	}		
	EXT_INT_FLAG++;
	if(EXT_INT_FLAG >= 2)
	{
		EXT_INT_FLAG = 1; garbage_timeout=0xFFFFFF;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		
		switch(SMS_Received)
		{
			case 0:
							memset(Rx0Buff,'\0',400);
							eSMS=&Rx0Buff[0];
							start_DMA_access1_receive(400,&Rx0Buff[0]);
							if(*eSMS=='\0')
							{
								while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx0 = 1;
							break;
			
			case 1:
							memset(Rx1Buff,'\0',400);
							eSMS=&Rx1Buff[0];
							start_DMA_access1_receive(400,&Rx1Buff[0]);
							if(*eSMS=='\0')
							{
							while(*eSMS != '\r' && (garbage_timeout-- != 0));							}
							if((garbage_timeout != 0))
							buff.Rx1 = 1;
							break;
			
			case 2:
							memset(Rx2Buff,'\0',400);
							eSMS=&Rx2Buff[0];
							start_DMA_access1_receive(400,&Rx2Buff[0]);
							if(*eSMS=='\0')
							{
								while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx2 = 1;
							break;
			
			case 3:
							memset(Rx3Buff,'\0',400);
							eSMS=&Rx3Buff[0];
							start_DMA_access1_receive(400,&Rx3Buff[0]);
							if(*eSMS=='\0')
							{
								while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx3 = 1;
							break;
			
			case 4:
							memset(Rx4Buff,'\0',400);
							eSMS=&Rx4Buff[0];
							start_DMA_access1_receive(400,&Rx4Buff[0]);
							if(*eSMS=='\0')
							{
							while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx4 = 1;
							break;
			
			case 5:
							memset(Rx5Buff,'\0',400);
							eSMS=&Rx5Buff[0];
							start_DMA_access1_receive(400,&Rx5Buff[0]);
							if(*eSMS=='\0')
							{
							while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx5 = 1;
							break;
			
			case 6:
							memset(Rx6Buff,'\0',400);
							eSMS=&Rx6Buff[0];
							start_DMA_access1_receive(400,&Rx6Buff[0]);
							if(*eSMS=='\0')
							{
								while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx6 = 1;
							break;
			
			case 7:
							memset(Rx7Buff,'\0',400);
							eSMS=&Rx7Buff[0];
							start_DMA_access1_receive(400,&Rx7Buff[0]);
							if(*eSMS=='\0')
							{
								while(*eSMS != '\r' && (garbage_timeout-- != 0));
							}
							if((garbage_timeout != 0))
							buff.Rx7 = 1;
							break;
							
			default:
							break;
		}
		if(SMS_Received >= 7)
		{
			SMS_Received = 0;
		}
		else
		{
			SMS_Received++;
		}
	}
	at_int = 0;
}
/******************************************************/
void EXTI9_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line9);
}
/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_IRQHandler(void)
{
	unsigned int i;

	if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
	{
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_SEC);
		
		/* Enable time update */
		TimeDisplay = 1;

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		
		/*****Check for the number of times the alarm is to be generated*********/
		for(i=0;i<16;i++) //EVENTSS
		{
			if(Event[i].independent_counter >= (60*Event[i].time_event))
			{
				Event[i].independent_counter = 0;
				Event[i].count = 0;
			}
			else
			{
				Event[i].independent_counter++;
			}
		}

		/* Reset RTC Counter when Time is 11:59:59 */
		if (RTC_GetCounter() > 0x00015180)
		{
			RESET_COUNT_FLAG = 1;
			/* Wait until last write operation on RTC registers has finished */
			RTC_WaitForLastTask();
		}
	}
	if(RTC_sec_count >= (converter.TimeGprs*60))
	{
		RTC_sec_count = 0;
		Data_Log_Time = 1;
		RTC_WaitForLastTask();
		RTC_time_capture = RTC_GetCounter();
		RTC_WaitForLastTask();
	}
	else if(converter.TimeGprs != 1)
		RTC_sec_count++;

	#ifdef MSD_Log_Interval
		if(msd_sec_count >= (MSD_Log_Interval*60))   
		{	
			msd_sec_count = 0;
			Sd_Write = 1; 
			RTC_WaitForLastTask();
			RTC_time_capture = RTC_GetCounter();
			RTC_WaitForLastTask();	
		}
		else 
			msd_sec_count++;
	#endif

	if((RTC_GetITStatus(RTC_IT_ALR) != RESET) || (RESET_COUNT_FLAG == 1))
	{
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_ALR);
		RTC_WaitForLastTask();
		SetAlarmIndication = 1; 
		if(RESET_COUNT_FLAG == 1)
		RESET_FLAG = 1;
		RTC_time_capture = RTC_GetCounter();
		RTC_WaitForLastTask();
	}
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}



  */ 
/*****END OF FILE****/
