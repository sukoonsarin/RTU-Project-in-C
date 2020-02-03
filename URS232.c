#include "stm32f10x.h"
#include "main.h"
#include "ATCGSM.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "parameters.h"

unsigned char data_collection = 0;
extern oneP datareceived,finaldata;
unsigned char itt_count = 0;
unsigned char data_ok = 0;
extern char Reset_Error[]; //sukoonerror
//**********************************
unsigned int CUMKWH_M, CUMKWH_L;
extern unsigned  long pv_current;
extern unsigned long pv_voltage;
//**********************************

struct Time4Alarm
{
	char Time[5];
	unsigned int time_event;
	unsigned int independent_counter;
	unsigned int count;
};

struct date_PCU
{
	unsigned char date;
	unsigned char month;
	unsigned char year;
};

struct rtc
{
	u32 Hour;
	u32 Minute;
	u32 Second;
};

struct LE 
{
	unsigned char MSB;
	unsigned char LSB;
};
		
struct networktime
{
	unsigned char Hour;
	unsigned char Minute;
	unsigned char Second;
};

struct timeformat
{
	unsigned char dd;
	unsigned char mm;
	unsigned char yy;
	unsigned char HH;
	unsigned char MM;
	unsigned char SS;
}SP1;

struct Time4NetworkSync
{
	unsigned char Date[2];
	unsigned char Month[2];
	unsigned char Year[2];
	unsigned char Hour[2];
	unsigned char Minute[2];
	unsigned char Second[2];
};

struct verifytime
{
	unsigned char verifyHour;
	unsigned char verifyMinute;
	unsigned char verifySecond;
};

struct RTC_PCU
{
	unsigned char dd_PCU;
	unsigned char mm_PCU;
	unsigned char yy_PCU;
	unsigned char HH_PCU;
	unsigned char MM_PCU;
	unsigned char SS_PCU;
}Verify;

struct Mobile
{
	char No1[15];
	char No2[15]; 
	char No3[15]; 
	char No4[15];
	char No5[15];
	char No6[15]; 
};

union CumulativeKWH
{
   unsigned long CUM_KWH;
   char bytes[4];
};

extern union CumulativeKWH CurrentPosition;

extern struct rtc GSM;
extern struct Time4Alarm Event[32];
extern struct networktime time;
extern struct verifytime verify;
extern struct Time4NetworkSync Sync;
struct date_PCU before_RTC,after_RTC;	 

unsigned char flag0,flag1,flag2,flag3,flag4,flag5,flag6,flag7,flag8,flag9,flag10,flag11,flag12,flag13,flag14,flag15,flag16,flag17;
unsigned char result_final1=0;
unsigned char Hr,Min,Sec;
unsigned char chk_Solar=0;
unsigned char result_count=0;
unsigned char DataNoGarbage;  
unsigned char errorSolar;
unsigned int real_event;

volatile unsigned char Hour,Minute,Second;
volatile unsigned char value_change_sign;
volatile unsigned char FlagforRTC=0;
volatile unsigned int RTC_time_capture;

extern unsigned char RTC_corrupt_flag;
extern unsigned char SetAlarmIndication;
extern unsigned char prev_conn;
extern char Fver[5]; 
extern volatile unsigned int temp_register;
extern volatile unsigned char Data_Log_Time, Sd_Write;
extern volatile unsigned char cable_connection;
extern volatile unsigned char Signal_Strength;
extern unsigned char Send_Msg_To_Server_Once; //ADD
	  
//void reset_ups(struct OnlineUPS *);
void reset_SCC(struct OnlineUPS *); //ADD
void write_RTCvalue(struct timeformat *);
void ASCII_alignation_OnlineUPS(char *storage,struct OnlineUPS *UPSDATA,struct Time4NetworkSync *,char *id,char *Fversion,unsigned char type,char cmode,char tower,char xph);
void ASCII_alignation_UPS_mobile_reader(char *,struct OnlineUPS *,unsigned char);
void send_MobileNo(struct Mobile *,char *);
void ASCII_hex_BCD_conversion(struct timeformat *,char *);
void getRTCvalue(char *,struct rtc *);
void autoTIMEset(char *,struct rtc *);
void ReplyRESET_Hardware(char *,char *);
void getRTCfromnetwork(struct rtc *,struct networktime *);	
void bcd2ASCII_time(unsigned char *x,char *data,char *id); 
void ExtractTimefromRTC(struct rtc *,unsigned char *);
void Reply_xdflt(char *tra,char rec,char cmode);
void reply_Rtime(char *time,char *id,struct Time4NetworkSync *st,char cmode);
void put_date(struct date_PCU *,struct date_PCU *);
void BuffMAPOnlineUPS(struct OnlineUPS *,unsigned char *);
void BuffMAP_MPPTSCC(struct OnlineUPS *,unsigned char *); //ADD
void ManageBuffer(struct OnlineUPS *,unsigned int);
unsigned char ASCII_TO_BCD(unsigned char ascii_text[2]);
unsigned int verify_RTCvalue(unsigned char *clock,struct timeformat *new);
unsigned int ASCII_2_decimal(char *);
//unsigned int sensitivity_STATUS(unsigned int );
unsigned int sensitivity_STATUS(struct OnlineUPS *); //ADD 
char comparison(struct date_PCU *, struct date_PCU *);	

extern void decimal_ASCII_conversion(unsigned int,char *,unsigned char);
extern void decimal_ASCII_conversion_4byte(unsigned long,char *);
extern void decimal_ASCII_conversion_4byte_CSTAT(unsigned long,char *);
extern void place_decimal(char *string , char pos);
extern unsigned char HEX_2_BCD(unsigned char );
extern unsigned int hex_decimal_conversion(unsigned char ,unsigned char );
extern unsigned long hex_decimal_conversion2bytes(unsigned int ,unsigned int );

void read_Data4mMPPTSCC(void);

/******************************************************/
void autoTIMEset(char *rt,struct rtc *time_value)
{
	time_value->Hour=(u32)*rt++;
	time_value->Hour=time_value->Hour<<8;
	time_value->Hour=time_value->Hour + (u32)*rt++;
	time_value->Minute=(u32)*rt++;
	time_value->Minute=time_value->Minute<<8;
	time_value->Minute=time_value->Minute + (u32)*rt++;
	time_value->Second=0x3030;
}
/******************************************************/
void getRTCvalue(char *rt,struct rtc *time_value)
{
	*rt++;
	*rt++;
	*rt++;
	*rt++;
	*rt++;
	*rt++;
	time_value->Hour=(u32)*rt++;
	time_value->Hour=time_value->Hour<<8;
	time_value->Hour = time_value->Hour + (u32)*rt++;
	time_value->Minute=(u32)*rt++;
	time_value->Minute=time_value->Minute<<8;
	time_value->Minute = time_value->Minute + (u32)*rt++;
	time_value->Second=(u32)*rt++;
	time_value->Second=time_value->Second<<8;
	time_value->Second = time_value->Second + (u32)*rt++;
}
/******************************************************/
void ExtractTimefromRTC(struct rtc *time_value,unsigned char *RTC_Time)
{
	u32 temp,temp_Hi,temp_Lo;

	temp=*RTC_Time++;
	temp_Hi=0x30+(temp>>4);
	temp_Lo=0x30+(temp&0x0F);

	temp_Hi=temp_Hi<<8;
	time_value->Second=(temp_Hi+temp_Lo);

	temp=*RTC_Time++;
	temp_Hi=0x30+(temp>>4);
	temp_Lo=0x30+(temp&0x0F);

	temp_Hi=temp_Hi<<8;
	time_value->Minute=(temp_Hi+temp_Lo);

	temp=*RTC_Time++ & 0x7F;
	temp_Hi=0x30+(temp>>4);
	temp_Lo=0x30+(temp&0x0F);

	temp_Hi=temp_Hi<<8;
	time_value->Hour=(temp_Hi+temp_Lo); 
}
/******************************************************/
unsigned char hex2bcd (unsigned char x)
{
	unsigned char y;
	
	y = (x / 10) << 4;
	y = y | (x % 10);
	return (y);
}
/******************************************************/
void ASCII_hex_BCD_conversion(struct timeformat *tt,char *ddmmyy)
{
	static unsigned char Temp[2];
	unsigned char decimal;

	Temp[0] = (unsigned char)*ddmmyy++;
	Temp[0] = (Temp[0] & 0x0F);
	Temp[1] = (unsigned char)*ddmmyy++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);
	tt->dd = hex2bcd(decimal);

	Temp[0] = (unsigned char)*ddmmyy++;
	Temp[0] = (Temp[0] & 0x0F); 
	Temp[1] = (unsigned char)*ddmmyy++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);
	tt->mm = hex2bcd(decimal);

	Temp[0] = (unsigned char)*ddmmyy++;
	Temp[0] = (Temp[0] & 0x0F); 
	Temp[1] = (unsigned char)*ddmmyy++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);
	tt->yy = hex2bcd(decimal);

	Temp[0] = (unsigned char)*ddmmyy++;
	Temp[0] = (Temp[0] & 0x0F); 
	Temp[1] = (unsigned char)*ddmmyy++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);

	tt->HH = hex2bcd(decimal);
	tt->HH = tt->HH + 128;

	Temp[0] = (unsigned char)*ddmmyy++;
	Temp[0] = (Temp[0] & 0x0F); 
	Temp[1] = (unsigned char)*ddmmyy++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);

	tt->MM = hex2bcd(decimal);

	Temp[0] = (unsigned char)*ddmmyy++;
	Temp[0] = (Temp[0] & 0x0F); 
	Temp[1] = (unsigned char)*ddmmyy++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);

	tt->SS = hex2bcd(decimal);
}
/******************************************************/
unsigned int ASCII_2_decimal(char *ASCII)
{
	unsigned int SUM=0;
	unsigned int Tem[15];
	unsigned char i=0,j=0,k=0;

	Tem[0]=Tem[1]=Tem[2]=Tem[3]=Tem[4]=Tem[5]=Tem[6]=Tem[7]=Tem[8]=Tem[9]=Tem[10]=Tem[11]=Tem[12]=Tem[13]=Tem[14]=0;
	while(*ASCII != '\0')
	{ 
		Tem[i] = *ASCII++ & 0x0F;
		i++;
	}
	j=0;
	while(i)
	{
		i--;
		k=j;
		while(k--)
		{
			Tem[i] = Tem[i] * 10; 
		}
		SUM = SUM + Tem[i];
		j++;
	}

	return SUM; 
}
/******************************************************/
void write_RTCvalue(struct timeformat *tt)
{
	USART_SendData(USART2,0x51);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,0x32);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,tt->SS);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);  
	USART_SendData(USART2,tt->MM);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,tt->HH);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,tt->dd);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,tt->mm);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,tt->yy);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,0x00);  // data login time
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,0x00);  // data login time
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,0x0D);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);  
}
/******************************************************/
unsigned int sensitivity_STATUS(struct OnlineUPS *datacollect)
{   
	//*******************************
		if((datacollect->StatusInverter.parameter & 512)>0) //High Temp Shutdown             
	{ 
		if(flag12>=5)
		{
			real_event = real_event|0x8000;
			flag12=0;
		} 
		else
		{
			flag12++;
		}
	}
	else
	{
		real_event = real_event&0x7FFF;
		flag12=0;
	}
	//***********************************
		if((datacollect->StatusInverter.parameter & 4)>0) //Short circuit shutdown      
	{
		if(flag2>=5)
		{
			real_event = real_event|0x4000;
			flag2=0;
		} 
		else
		{
			flag2++;
		}
	}
	else
	{
		real_event = real_event&0xBFFF;
		flag2=0;
	}
	//***********************************
		if((datacollect->StatusInverter.parameter & 64)>0) //Battery Low Shutdown       
	{
		if(flag6>=8)    
		{
			real_event = real_event|0x2000;
			flag6=0;
		} 
		else
		{
			flag6++;
		}
	}
	else
	{
		real_event = real_event&0xDFFF;
		flag6=0;
	}
	//***********************************
		if((datacollect->StatusInverter.parameter & 16)>0) //Overload Shutdown             
	{ 
		if(flag1>=8)
		{
			real_event = real_event|0x1000;
			flag1=0;
		} 
		else
		{
			flag1++;
		}
	}
	else
	{
		real_event = real_event&0xEFFF;
		flag1=0;
	}
	
	//***********************************
	if((datacollect->FlagCharging.parameter&8192)>0) //Battery High Shutdown                  
	{ 
		if(flag7>=6)
		{
			real_event = real_event|0x0800;
			flag7=0;
		} 
		else
		{
			flag7++;
		}
	}
	else
	{
		real_event = real_event&0xF7FF;
		flag7=0;
	}
	//***********************************
		if((datacollect->StatusInverter.parameter & 4096)>0) //High Temp Warning              
	{
		if(flag15>8)
		{
			real_event = real_event|0x0400;
			flag15=0;
		} 
		else
		{
			flag15++;
		}
	}
	else
	{
		real_event = real_event&0xFBFF;
		flag15=0;
	}
	//***********************************
	if((datacollect->StatusInverter.parameter & 128)>0) //Battery Low Warning               
	{
		if(flag3>8)
		{
			real_event = real_event|0x0200;
			flag3=0;
		} 
		else
		{
			flag3++;
		}
	}
	else
	{
		real_event = real_event&0xFDFF;
		flag3=0;
	}
	//***********************************
	if((datacollect->StatusInverter.parameter & 32)>0) //Overload Warning
	{
		if(flag0>=8)
		{
			real_event = real_event|0x0100;
			flag0=0;
		} 
		else
		{
			flag0++;
		}
	}
	else
	{
		real_event = real_event&0xFEFF;
		flag0=0;
	}
	//***********************************
	if((datacollect->SolarStatus.parameter&4)>0) //PV Current High
	{ 
		if(flag13>=6)
		{
      real_event = real_event|0x0080;
			flag13=0;
		} 
		else
		{
			flag13++;
		}
	}
	else
	{
		real_event = real_event&0xFF7F;
		flag13=0;
	}
	//***********************************
			if((datacollect->SolarStatus.parameter&2)>0) //PV Voltage High
	{ 
		if(flag14>=6)
		{
			real_event = real_event|0x0040;
			flag14=0;
		} 
		else
		{
			flag14++;
		}
	}
	else
	{
		real_event = real_event&0xFFBF;
		flag14=0;
	}
	//***********************************
		//if((datacollect->StatusInput.parameter & 128)<1) //Mains Fail
		  if((datacollect->OperatingMode.parameter != 4) && (datacollect->OperatingMode.parameter != 6)) //Mains Fail
	{
		if(flag9>=6)
		{
			real_event = real_event|0x0020;
			flag9=0;
		} 
		else
		{
			flag9++;
		}
	}
	else
	{
			real_event = real_event&0xFFDF;
		flag9=0;
	}

	//***********************************
		//	if((datacollect->StatusInput.parameter & 128)>0)//Mains OK   
        if((datacollect->OperatingMode.parameter == 4) || (datacollect->OperatingMode.parameter == 6))//Mains OK         	
	{ 
		if(flag8>=12)
		{
			real_event = real_event|0x0010;
			flag8=0;
		} 
		else
		{
			flag8++;
		}
	}
	else
	{
			real_event = real_event&0xFFEF;
		flag8=0;
	}

	//***********************************
			if((datacollect->StatusInput.parameter & 1024)>0)//Mains High            
	{
		if(flag11>=6)
		{
			
			real_event = real_event|0x0008;
			flag11 = 0;
		} 
		else
		{
			flag11++;
		}
	}
	else
	{ 

		real_event = real_event&0xFFF7;
		
		flag11 = 0; //ADD
	}
	
	//***********************************
		if((datacollect->StatusInput.parameter & 2048)>0) //Mains Low              
	{ 
		if(flag10>=8)
		{

			real_event = real_event|0x0004;
			flag10=0;
		} 
		else
		{
			flag10++;
		}
	}
	else
	{

		real_event = real_event&0xFFFB;
		flag10=0;
	}

	//***********************************

		if(datacollect->OperatingMode.parameter == 2) //UPS Status
	{
		if(flag4>=12)   //UPS ON
		{
			real_event = real_event&0xFFFE;
			real_event = real_event|0x0002;
			flag4=0;
		} 
		else
		{
			flag4++;
		}
	}
	else if(datacollect->OperatingMode.parameter == 0)
	{
		if(flag5>=12)   //UPS OFF
		{
			real_event = real_event&0xFFFD;
			real_event = real_event|0x0001;
			flag5=0;
		} 
		else
		{
			flag5++;
		}
	}
	
	//***********************************

	return(real_event);
}

/*******************************************************/
void read_Data4mMPPTSCC(void)
{
	
	USART_SendData(USART2,0x51);                                                   // 1
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,0x31);                                                   // 2
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	USART_SendData(USART2,0x0D);                                                   // 3
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET); 

	/*
	dd_mm_yy_hh_mm_ss = Sync.Date[0];	    // Date_LSB                              // 4
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	dd_mm_yy_hh_mm_ss = Sync.Date[1];	    // Date_MSB                              // 5
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	dd_mm_yy_hh_mm_ss = Sync.Month[0];	   // Month_LSB                            // 6
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	dd_mm_yy_hh_mm_ss = Sync.Month[1];	   // Month_MSB                             // 7
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	dd_mm_yy_hh_mm_ss = Sync.Year[0];	     // Year_LSB                              // 8
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	dd_mm_yy_hh_mm_ss = Sync.Year[1];	      // Year_MSB                             // 9
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	HH_MM_SS = RTC_time_capture;

	dd_mm_yy_hh_mm_ss = HH_MM_SS/3600;                        //HR                  // 10
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	HH_MM_SS = (HH_MM_SS - (dd_mm_yy_hh_mm_ss*3600));     //MIN                     // 11
	dd_mm_yy_hh_mm_ss = HH_MM_SS/60;
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
	
	HH_MM_SS = (HH_MM_SS - (dd_mm_yy_hh_mm_ss*60));       //SEC                      // 12
	dd_mm_yy_hh_mm_ss = HH_MM_SS;	
	USART_SendData(USART2,dd_mm_yy_hh_mm_ss);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET);
*/
	
}
/******************************************************/
unsigned int verify_RTCvalue(unsigned char *clock, struct timeformat *new)
{
	unsigned int result=0;
	
	if(*clock++ == 0x41)
	{
		*clock++;
		if(*clock++ == new->SS)
		{
			*clock++;
			if(*clock++ == new->MM)
			{
				*clock++;
				if(*clock++ == new->HH)
				{
					*clock++;
					if(*clock++ == new->dd)
					{
						*clock++;
						if(*clock++ == new->mm)
						{
							*clock++;
							if(*clock++ == new->yy)
							{
								*clock++;
								if(*clock++ == 0x0D)
								{
									result=1;
								}
							}
						}
					}
				}
			}
		}
	}
	else
	{
		result=0;
	}

	return result;  
}
/*******************************************************************************************/
void ASCII_alignation_OnlineUPS(char *storage,struct OnlineUPS *UPSDATA,struct Time4NetworkSync *datatime,char *id,char *Fversion,unsigned char type,char cmode,char tower,char xph)
{
	char PL[6];
char FOURBYTEPlace[11];
	unsigned char i;
	unsigned int time_capture;
	unsigned char temp,temp_MSB,temp_LSB;

	if(SetAlarmIndication)            
		time_capture = RTC_time_capture; 

	else
		time_capture = RTC_GetCounter();

	Hr = time_capture/3600;
	time_capture = (time_capture - (Hr*3600));
	Min = time_capture/60;
	time_capture = (time_capture - (Min*60));
	Sec = time_capture;	

	if(cmode)                     // header
	{
		*storage++ = 0x0D;          // \r
		*storage++ = 0x0A;          // \n
	}		
	switch(type)
	{
		case 0:
			*storage++ = 0x50;   // send P 
			break;
		case 1:
			*storage++ = 0x47;   // send G 
			break;
		case 2:
			*storage++ = 0x45;   // send E
			break;
		case 3:
			*storage++ = 0x52;   // send R
			break;
		case 4:
			*storage++ = 0x41;   // send A
			break;		
		case 5:
			if(Reset_Error[0] == 1)
			{*storage++ = 0x50;}   // send P //sukoonerror
			else
			{*storage++ = 0x49;}   // send I
			break;					 
		default:
			break;
	}
	*storage++ = 0x2F;   // send /*/
	
//**********************************
	
	while(*id != '\0' )                         // Site_ID
	{ 
		*storage++ = *id;
		*id++;
	}
	*storage++ = 0x2F;   // send /
	
//**********************************
	
	memset(PL,'\0',6);                            
	decimal_ASCII_conversion(UPSDATA->ControlFversn.parameter,PL,0);      // Firmware Version Grid_Tie
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}
	*storage++ = 0x2F;   // send /
	
//**********************************

	while(*Fversion != '\0' )                                                 // Firmware Version RMU
	{ 
		*storage++ = *Fversion;
		*Fversion++;
	}
	*storage++ = 0x2F;   // send /
	
//********************************** 
	memset(PL,'\0',6);                                                          // System_capacity
	UPSDATA->SystemCapacity.parameter = UPSDATA->SystemCapacity.parameter/100;
	decimal_ASCII_conversion(UPSDATA->SystemCapacity.parameter,PL,0);
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}
	
	*storage++ = 0x2F; 
	
//**********************************
	
	memset(PL,'\0',6);  
	decimal_ASCII_conversion(UPSDATA->PV_V.parameter,PL,0);                 // PV_Voltage
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
//**********************************
	
	memset(PL,'\0',6);  
	decimal_ASCII_conversion(UPSDATA->PV_I.parameter,PL,0);                  // PV_Current
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);  
	decimal_ASCII_conversion(UPSDATA->DCBus.parameter,PL,0);                  // PV_Current
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 

//**********************************
	
	memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->Batt_V.parameter,PL,0);                     // PV_Power
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
//**********************************
	
	memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->Batt_I.parameter,PL,0);                 // Grid_Voltage
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
//**********************************	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->Input_V.parameter,PL,0);                  // HeatSinkTemp
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      

	*storage++ = 0x2F;    
	
//**********************************	

	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->Charging_I.parameter,PL,0);                  // Cum_KWH_Day1
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      

	*storage++ = 0x2F; 
	//**********************************	
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->Input_Freq.parameter,PL,0);                   // Cum_KWH_Day2
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      

	*storage++ = 0x2F; 
	
	//**********************************	
	memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->Output_V.parameter,PL,0);                 // Grid_Voltage
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
	//**********************************	
	
  memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->Output_Freq.parameter,PL,0);                 // Grid_Current
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F;
	
//**********************************	
	
	memset(PL,'\0',6);                                                                                        
	decimal_ASCII_conversion(UPSDATA->Load_Per.parameter,PL,0);                // Grid_Frequency 
	i=0;                                                                                                      
	while(PL[i] != '\0')                                                                                      
	{                                                                                                         
		*storage++ = PL[i++];                                                                                   
	}               
	
	*storage++ = 0x2F;     
	
//**********************************

	memset(PL,'\0',6);                                                                                        
	decimal_ASCII_conversion(UPSDATA->Batt_per.parameter,PL,0);                // Grid_Frequency 
	i=0;                                                                                                      
	while(PL[i] != '\0')                                                                                      
	{                                                                                                         
		*storage++ = PL[i++];                                                                                   
	}               
	
	*storage++ = 0x2F;     
	
//**********************************

	memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->Boost_V.parameter ,PL,0);                 // Grid_Voltage
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
	//**********************************	

	memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->Amb_Temp.parameter,PL,0);                 // Grid_Voltage
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}

	*storage++ = 0x2F; 
	
	//**********************************
	memset(FOURBYTEPlace,'\0',11);                                                     // Cum_KWH
	decimal_ASCII_conversion_4byte_CSTAT(CurrentPosition.CUM_KWH,FOURBYTEPlace);
	i=0;
  while(FOURBYTEPlace[i] != '\0' )
	{
		*storage++ = FOURBYTEPlace[i++];
	}
	
	*storage++ = 0x2F;

	//**********************************
	*storage++ = 0x30;
	*storage++ = 0x2F;
	*storage++ = 0x30;
	*storage++ = 0x2F;
	*storage++ = 0x30;
	*storage++ = 0x2F;
	//**********************************		

	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->StatusWord.parameter,PL,0);                   // Cum_KWH_Day2
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      

	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->StatusCommand.parameter,PL,0);                   // Cum_KWH_Day2
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      
		
	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->StatusInverter.parameter,PL,0);                   // Cum_KWH_Day3
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}          
	
	
	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->StatusInput.parameter,PL,0);                   // Cum_KWH_Day4
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      
	
	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->FlagCharging.parameter,PL,0);                     // Cum_KWH_Day5
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                     
		
	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->StatusSwitch.parameter,PL,0);                     // Cum_KWH_Day5
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                     
		
	*storage++ = 0x2F; 
	
	//**********************************
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->SolarStatus.parameter,PL,0);                     // Cum_KWH_Day6
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      
	
	*storage++ = 0x2F; 
	
	//**********************************

	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->OperatingMode.parameter,PL,0);                     // Cum_KWH_Day6
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                      
		 
	*storage++ = 0x2F; 
	
	//**********************************	
	
	memset(PL,'\0',6);     
	decimal_ASCII_conversion(UPSDATA->SystemID.parameter,PL,0);                     // Cum_KWH_Day5
	i=0;                   
	while(PL[i] != '\0')   
	{                      
		*storage++ = PL[i++];
	}                     
	
	*storage++ = 0x2F; 
	
	//**********************************
	
	*storage++ = datatime->Date[0];                                                          // Date_Time
	*storage++ = datatime->Date[1];

	*storage++ = datatime->Month[0];
	*storage++ = datatime->Month[1];

	*storage++ = datatime->Year[0];
	*storage++ = datatime->Year[1];

	temp = hex2bcd(Hr);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*storage++ = temp_MSB;
	*storage++ = temp_LSB;   

	temp = hex2bcd(Min);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*storage++ = temp_MSB;
	*storage++ = temp_LSB;

	temp = hex2bcd(Sec);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	//*storage++ = temp_MSB;
	//*storage++ = temp_LSB;
	 		if(cmode)                     // header
	{
		*storage++ = 0x0D;          // \r
		*storage++ = 0x0A;          // \n
	} 	  
	 
//**********************************
}
/******************************************************/
void ASCII_alignation_UPS_mobile_reader(char *storage,struct OnlineUPS *st,unsigned char type)
{
	unsigned char i;
	char PL[6];
	char FOURBYTEPlace[11];
		unsigned int time_capture;
	unsigned char temp,temp_MSB,temp_LSB;
//*******************************************
	if(SetAlarmIndication)
		time_capture = RTC_time_capture;

	else
		time_capture = RTC_GetCounter();
	
	Hr = time_capture/3600;
	time_capture = (time_capture - (Hr*3600));
	Min = time_capture/60;
	time_capture = (time_capture - (Min*60));
	Sec = time_capture;
//*********************************************
	*storage++ = 'G'; *storage++ = 'T'; *storage++ = 'I'; *storage++ = ' '; *storage++ = 'H'; *storage++ = 'Y'; *storage++ = 'B'; *storage++ = 'R'; *storage++ = 'I'; *storage++ = 'D'; *storage++ = ' '; 
	memset(PL,'\0',6); 
  st->SystemCapacity.parameter = st->SystemCapacity.parameter/100;
	decimal_ASCII_conversion(st->SystemCapacity.parameter,PL,1);                  // System_capacity
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}
	*storage++ = ' '; *storage++ = 'k'; *storage++ = 'W'; *storage++ = 0x0A;
	
    if((type == 5)&&(Reset_Error[0] != 1))
	{*storage++ = 'S'; *storage++ = 'Y'; *storage++ = 'S'; *storage++ = 'T'; *storage++ = 'E'; *storage++ = 'M'; *storage++ = ' '; *storage++ = 'H';
	 *storage++ = 'A'; *storage++ = 'S'; *storage++ = ' '; *storage++ = 'S'; *storage++ = 'T'; *storage++ = 'A'; *storage++ = 'R'; *storage++ = 'T'; *storage++ = 'E'; *storage++ = 'D'; *storage++ = 0x0A;
	}
	else if(type == 4)
	{*storage++ = 'A'; *storage++ = 'U'; *storage++ = 'T'; *storage++ = 'O'; *storage++ = ' '; *storage++ = 'R';*storage++ = 'E'; *storage++ = 'P'; *storage++ = 'O'; *storage++ = 'R'; *storage++ = 'T'; *storage++ = 0X0A;
	}
	else if(type == 2)
	{*storage++ = 'A'; *storage++ = 'L'; *storage++ = 'E'; *storage++ = 'R'; *storage++ = 'T'; *storage++ = '!'; *storage++ = 0X0A;}
			else if(type == 6)
	{*storage++ = 'U'; *storage++ = 'N'; *storage++ = 'I'; *storage++ = 'T'; *storage++ = ' '; *storage++ = 'G'; *storage++ = 'E'; *storage++ = 'N'; *storage++ = 'E'; *storage++ = 'R'; *storage++ = 'A';*storage++ = 'T'; *storage++ = 'E'; *storage++ = 'D'; *storage++ = 0X0A;}
	/*******************************************************************************************************************/
/*if((st->OperatingMode.parameter == 4) || (st->OperatingMode.parameter == 0))//Grid charging mode //Commented on 15June because batt% to be worked on
{
	*storage++ = 0x42;	// send B
	*storage++ = 0x61;	// send a
	*storage++ = 0x74;  // send t
	*storage++ = 0x74;	// send t
	*storage++ = 0x65;	// send e
	*storage++ = 0x72;	// send r
	*storage++ = 0x79;	// send y
	*storage++ = 0x20;	// send " "
	*storage++ = 0x25;	// send %
	*storage++ = 0x3D;	// send =
	*storage++ = 0x20;     // send " "
	memset(PL,'\0',6);                                                                                        
	decimal_ASCII_conversion(st->Batt_per.parameter,PL,0);                // Battery_Percentage 
	i=0;                                                                                                      
	while(PL[i] != '\0')                                                                                      
	{                                                                                                         
		*storage++ = PL[i++];                                                                                   
	}               
	*storage++ = 0x0A;	// send ENTER
}
else if(st->OperatingMode.parameter == 2) //backup mode
{
	*storage++ = 0x4C;	// send L
	*storage++ = 0x6F;	// send o
	*storage++ = 0x61;  // send a
	*storage++ = 0x64;	// send d
	*storage++ = 0x20;	// send " "
	*storage++ = 0x25;	// send %
	*storage++ = 0x3D;	// send =
	*storage++ = 0x20;     // send " "
	memset(PL,'\0',6);                                                                                        
	decimal_ASCII_conversion(st->Load_Per.parameter,PL,0);                // Load_Percentage 
	i=0;                                                                                                      
	while(PL[i] != '\0')                                                                                      
	{                                                                                                         
		*storage++ = PL[i++];                                                                                   
	}               
	*storage++ = 0x0A;	// send ENTER
	
	
	*storage++ = 0x42;	// send B
	*storage++ = 0x61;	// send a
	*storage++ = 0x74;  // send t
	*storage++ = 0x74;	// send t
	*storage++ = 0x65;	// send e
	*storage++ = 0x72;	// send r
	*storage++ = 0x79;	// send y
	*storage++ = 0x20;	// send " "
	*storage++ = 0x25;	// send %
	*storage++ = 0x3D;	// send =
	*storage++ = 0x20;     // send " "
	memset(PL,'\0',6);                                                                                        
	decimal_ASCII_conversion(st->Batt_per.parameter,PL,0);                // Battery_Percentage 
	i=0;                                                                                                      
	while(PL[i] != '\0')                                                                                      
	{                                                                                                         
		*storage++ = PL[i++];                                                                                   
	}               
	*storage++ = 0x0A;	// send ENTER
}


else	
{*/
	*storage++ = 0x50;	// send P
	*storage++ = 0x56;	// send V
	*storage++ = 0x20;     // send " "
	*storage++ = 0x50;	// send P
	*storage++ = 0x6F;	// send o
	*storage++ = 0x77;	// send w
	*storage++ = 0x65;	// send e
	*storage++ = 0x72;	// send r
	*storage++ = 0x3D;	// send =
	*storage++ = 0x20;     // send " "
	memset(PL,'\0',6);
	decimal_ASCII_conversion(st->PVPower.parameter,PL,1);                     // PV_Power
	i=0;
	while(PL[i] != '\0')
	{
		*storage++ = PL[i++];
	}
	*storage++ = 0x20;  // send " "
	*storage++ = 0x57;	// send W
	*storage++ = 0x61;	// send a
	*storage++ = 0x74;  // send t
	*storage++ = 0x74;	// send t
	*storage++ = 0x73;	// send s
	*storage++ = 0x0A;	// send ENTER
//}
	
	/*******************************************************************************************************************/
  *storage++ = 0x43;	// send C
	*storage++ = 0x75;	// send u
	*storage++ = 0x6D;  // send m
	*storage++ = 0x75;	// send u
	*storage++ = 0x6C;	// send l
	*storage++ = 0x61;	// send a
	*storage++ = 0x74;	// send t
	*storage++ = 0x69;	// send i
	*storage++ = 0x76;	// send v
	*storage++ = 0x65;	// send e
	*storage++ = 0x20;  // send " "
	*storage++ = 0x6B;	// send k
	*storage++ = 0x57;	// send W
	*storage++ = 0x68;	// send h
	*storage++ = 0x3D;	// send =
	*storage++ = 0x20;     // send " "
	memset(FOURBYTEPlace,'\0',11);                                                     // Cum_KWH
	decimal_ASCII_conversion_4byte(CurrentPosition.CUM_KWH,FOURBYTEPlace);
	i=0;
  while(FOURBYTEPlace[i] != '\0' )
	{
		*storage++ = FOURBYTEPlace[i++];
	}
	*storage++ = 0x20;     // send " "        	   
	*storage++ = 0x6B;	// send k
	*storage++ = 0x57;	// send W
	*storage++ = 0x68;	// send h
	*storage++ = 0x0A;	// send ENTER	
	/***************************************OPERATING MODE*********************************/
	
if(data_collection)
{
			*storage++ = 0x0A;	// send ENTER	
			*storage++ = 'C'; *storage++ = 'o'; *storage++ = 'm'; *storage++ = 'm'; *storage++ = ' '; *storage++ = 'E'; *storage++ = 'r'; *storage++ = 'r';
			*storage++ = 'o'; *storage++ = 'r';  *storage++ = 0x0A;
}
	else
	{
if(st->OperatingMode.parameter == 0)
		{
			*storage++ = 0x0A;	// send ENTER	
			*storage++ = 'S'; *storage++ = 'y'; *storage++ = 's'; *storage++ = 't'; *storage++ = 'e'; *storage++ = 'm'; *storage++ = ' '; *storage++ = 'O';
			*storage++ = 'f'; *storage++ = 'f';  *storage++ = 0x0A;
		}
else if(st->OperatingMode.parameter == 2)
		{
			*storage++ = 0x0A;	// send ENTER	
			*storage++ = 'B'; *storage++ = 'a'; *storage++ = 'c'; *storage++ = 'k'; *storage++ = ' '; *storage++ = 'u'; *storage++ = 'p'; *storage++ = ' ';
	    *storage++ = 'M'; *storage++ = 'o'; *storage++ = 'd'; *storage++ = 'e'; *storage++ = 0x0A; 
		}
else if(st->OperatingMode.parameter == 4)
		{
			*storage++ = 0x0A;	// send ENTER	
			*storage++ = 'G'; *storage++ = 'r'; *storage++ = 'i'; *storage++ = 'd'; *storage++ = ' '; *storage++ = 'C'; *storage++ = 'h'; *storage++ = 'a';
	    *storage++ = 'r'; *storage++ = 'g'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 'g'; *storage++ = 0x0A;
		}
 
else if(st->OperatingMode.parameter == 6)
		{
			*storage++ = 0x0A;	// send ENTER	
			*storage++ = 'G'; *storage++ = 'r'; *storage++ = 'i'; *storage++ = 'd'; *storage++ = ' '; *storage++ = 'F'; *storage++ = 'e'; *storage++ = 'e';
	    *storage++ = 'd'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 'g'; *storage++ = 0x0A;
		}
		
else if(st->OperatingMode.parameter == 7) //Fault Mode
		{
			*storage++ = 0x0A;	// send ENTER	
		}
	}

/**************************************************/
	if((st->StatusInverter.parameter & 512)>0)     // High Temp Shutdown 
	{
		*storage++ = 'H'; *storage++ = 'i'; *storage++ = 'g'; *storage++ = 'h'; *storage++ = ' '; 	
		*storage++ = 'T'; *storage++ = 'e'; *storage++ = 'm'; *storage++ = 'p' ; *storage++ = ' ' ;
		*storage++ = 'S'; *storage++ = 'h'; *storage++ = 'u'; *storage++ = 't'; *storage++ = 'd'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = 'n'; *storage++ = 0x0A;
	}
		else if((st->StatusInverter.parameter & 4)>0)   // Short Circuit Shutdown
	{
		*storage++ = 'S'; *storage++ = 'h'; *storage++ = 'o'; *storage++ = 'r';  *storage++ = 't'; *storage++ = ' '; *storage++ = 'C'; *storage++ = 'k'; *storage++ = 't'; *storage++ = ' ';
		*storage++ = 'S'; *storage++ = 'h'; *storage++ = 'u'; *storage++ = 't'; *storage++ = 'd'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = 'n'; *storage++ = 0x0A;
	}
		else if((st->StatusInverter.parameter & 64)>0)   // Battery Low Shutdown
	{
		*storage++ = 'B'; *storage++ = 'a'; *storage++ = 't'; *storage++ = 't'; *storage++ = ' '; 	
		*storage++ = 'L'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = ' ' ; 
		*storage++ = 'S'; *storage++ = 'h'; *storage++ = 'u'; *storage++ = 't'; *storage++ = 'd'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = 'n'; *storage++ = 0x0A;  
	}
		else if((st->StatusInverter.parameter & 16)>0)   // Overload Shutdown
	{
		*storage++ = 'O'; *storage++ = 'v'; *storage++ = 'e'; *storage++ = 'r'; *storage++ = 'l'; *storage++ = 'o'; *storage++ = 'a'; *storage++ = 'd'; *storage++ = ' '; 	
		*storage++ = 'S'; *storage++ = 'h'; *storage++ = 'u'; *storage++ = 't'; *storage++ = 'd'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = 'n'; *storage++ = 0x0A;  
	}
	else if((st->FlagCharging.parameter&8192)>0)      // Battery High Shutdown
	{
		*storage++ = 'B'; *storage++ = 'a'; *storage++ = 't'; *storage++ = 't'; *storage++ = ' '; 	
		*storage++ = 'H'; *storage++ = 'i'; *storage++ = 'g'; *storage++ = 'h' ; *storage++ = ' ' ;
		*storage++ = 'S'; *storage++ = 'h'; *storage++ = 'u'; *storage++ = 't'; *storage++ = 'd'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = 'n'; *storage++ = 0x0A;  
	}

else
	{
		 if((st->StatusInverter.parameter & 4096)>0)    // High Temperature Warning 
		{
		  *storage++ = 'H'; *storage++ = 'i'; *storage++ = 'g'; *storage++ = 'h'; *storage++ = ' '; 	
		  *storage++ = 'T'; *storage++ = 'e'; *storage++ = 'm'; *storage++ = 'p' ; *storage++ = ' ' ;
		  *storage++ = 'W'; *storage++ = 'a'; *storage++ = 'r'; *storage++ = 'n'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 'g'; *storage++ = 0x0A; 
		}
		else if((st->StatusInverter.parameter & 128)>0)       // Battery Low Warning
		{
			*storage++ = 'B'; *storage++ = 'a'; *storage++ = 't'; *storage++ = 't'; *storage++ = ' '; 	
			*storage++ = 'L'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = ' '; 
		  *storage++ = 'W'; *storage++ = 'a'; *storage++ = 'r'; *storage++ = 'n'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 'g'; *storage++ = 0x0A;   
		}
		else if((st->StatusInverter.parameter & 32)>0)   // Overload Warning
		{
		*storage++ = 'O'; *storage++ = 'v'; *storage++ = 'e'; *storage++ = 'r'; *storage++ = 'l'; *storage++ = 'o'; *storage++ = 'a'; *storage++ = 'd'; *storage++ = ' '; 
		  *storage++ = 'W'; *storage++ = 'a'; *storage++ = 'r'; *storage++ = 'n'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 'g'; *storage++ = 0x0A; 
		}
		if((st->SolarStatus.parameter&4)>0)      // PV Current High
		{
			*storage++ = 'P'; *storage++ = 'V'; *storage++ = ' '; *storage++ = 'C'; *storage++ = 'u'; *storage++ = 'r'; *storage++ = 'r'; *storage++ = 'e'; *storage++ = 'n' ; *storage++ = 't';  *storage++ = ' ' ; 		
			*storage++ = 'H'; *storage++ = 'i'; *storage++ = 'g'; *storage++ = 'h'; *storage++ = 0x0A;
		}
		else if((st->SolarStatus.parameter&2)>0)   // PV Voltage High
		{
			*storage++ = 'P'; *storage++ = 'V'; *storage++ = ' '; *storage++ = 'V'; *storage++ = 'o'; *storage++ = 'l'; *storage++ = 't'; *storage++ = 'a'; *storage++ = 'g'; *storage++ = 'e';  *storage++ = ' ' ; 		
			*storage++ = 'H'; *storage++ = 'i'; *storage++ = 'g'; *storage++ = 'h'; *storage++ = 0x0A;
		}
		//if((st->StatusInput.parameter & 128)<1)   // Mains Fail
		if((st->OperatingMode.parameter != 4) && (st->OperatingMode.parameter != 6))//Mains Fail 
		{
			*storage++ = 'M'; *storage++ = 'a'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 's'; *storage++ = ' ';	
			*storage++ = 'F'; *storage++ = 'a'; *storage++ = 'i'; *storage++ = 'l'; *storage++ = 0x0A; 		
		}
		else if((st->StatusInput.parameter & 2048)>0)   // mains low
		{
	    *storage++ = 'M'; *storage++ = 'a'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 's'; *storage++ = ' ';	
			*storage++ = 'L'; *storage++ = 'o'; *storage++ = 'w'; *storage++ = 0x0A;  
		}
	  else if((st->StatusInput.parameter & 1024)>0)   // mains high
		{
			*storage++ = 'M'; *storage++ = 'a'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 's'; *storage++ = ' ';	
			*storage++ = 'H'; *storage++ = 'i'; *storage++ = 'g'; *storage++ = 'h'; *storage++ = 0x0A; 	
		}
		//else if((st->StatusInput.parameter & 128)>0) //Main OK
		else if((st->OperatingMode.parameter == 4) || (st->OperatingMode.parameter == 6))//Mains OK 
		{
			*storage++ = 'M'; *storage++ = 'a'; *storage++ = 'i'; *storage++ = 'n'; *storage++ = 's'; *storage++ = ' ';	
			*storage++ = 'O'; *storage++ = 'K'; *storage++ = 0x0A;  
		}



	}


/***********************TIME***********************/
	
	//*storage++ = 0x0A; 
	*storage++ = 'T'; *storage++ = 'I'; *storage++ = 'M'; *storage++ = 'E'; *storage++ = ':'; *storage++ = ' ';
	
	temp = hex2bcd(Hr);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*storage++ = temp_MSB;
	*storage++ = temp_LSB;   
	*storage++ = ':'; 

	temp = hex2bcd(Min);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*storage++ = temp_MSB;
	*storage++ = temp_LSB;
	//*storage++ = ':'; 

	temp = hex2bcd(Sec);

	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	//*storage++ = temp_MSB;
	//*storage++ = temp_LSB;	
}
/******************************************************/
void send_MobileNo(struct Mobile *ext,char *dt)
{
	unsigned char i;

	*dt++ = 0x50;
	*dt++ = 0x68;
	*dt++ = 0x31;
	*dt++ = 0x20;	
	i=0;
	while(ext->No1[i] != '\0')
	{
		*dt++ = ext->No1[i++];
	}
	*dt++ = 0x0A;	// send ENTER
	*dt++ = 0x50;
	*dt++ = 0x68;
	*dt++ = 0x32;
	*dt++ = 0x20;	// send SPACE
	i=0;
	while(ext->No2[i] != '\0')
	{
		*dt++ = ext->No2[i++];
	}      
}
/******************************************************/
void ReplyRESET_Hardware(char *id,char *tra)
{
	*tra++ = 0x52;   // Reply R
	*tra++ = 0x2F;   // slash /
	while(*id != '\0')
	{
		*tra++ = *id;
		*id++;
	}
	*tra++ = 0x2F;   // slash
	*tra++ = 0x48;   // H
	*tra++ = 0x61;	 // a
	*tra++ = 0x72;	 // r
	*tra++ = 0x64;	 // d
	*tra++ = 0x77;   //	w
	*tra++ = 0x61;   // a
	*tra++ = 0x72;	 // r
	*tra++ = 0x65;	 // e
	*tra++ = 0x52;	 // R
	*tra++ = 0x65;	 // e
	*tra++ = 0x73;	 // s
	*tra++ = 0x65;	 // e
	*tra++ = 0x74;	 // t
}
/******************************************************/

void Reply_xdflt(char *tra,char rec,char cmode)
{
	if(cmode)                     // header
	{
		*tra++ = 0x0D;          // \r
		*tra++ = 0x0A;          // \n
	}
	*tra++ = 0x52;   // Reply R
	*tra++ = 0x2F;   // slash /
	*tra++ = 'D';   // H
	*tra++ = 'e';	 // a
	*tra++ = 'f';	 // r
	*tra++ = 'a';	 // d
	*tra++ = 'u';   //	w
	*tra++ = 'l';   // a
	*tra++ = 't';	 // r
	if(rec)
	{
		*tra++ = 'N';	 // R
		*tra++ = 'o';	 // e
		*tra++ = 't';	 // s	
	}
	*tra++ = 'S';	 // R
	*tra++ = 'e';	 // e
	*tra++ = 't';	 // s
	if(cmode)                     // header
	{
		*tra++ = 0x0D;          // \r
		*tra++ = 0x0A;          // \n
	}
}

/******************************************************/
void reply_Rtime(char *time,char *id,struct Time4NetworkSync *st,char cmode)
{
	unsigned char temp,temp_MSB,temp_LSB;
	unsigned int time_capture;
	unsigned char Hr,Min,Sec;

	time_capture = RTC_GetCounter();
	Hr = time_capture/3600;
	time_capture = (time_capture - (Hr*3600));
	Min = time_capture/60;
	time_capture = (time_capture - (Min*60));
	Sec = time_capture;	

	if(cmode)                     // header
	{
		*time++ = 0x0D;          // \r
		*time++ = 0x0A;          // \n
	}

	*time++ = 0x52;   // Reply R
	*time++ = 0x2F;   // slash /
	while(*id != '\0')
	{
		*time++ = *id;
		*id++;
	}
	*time++ = 0x2F;   // slash /
	*time++ = st->Date[0];
	*time++ = st->Date[1];		
	*time++ = st->Month[0];
	*time++ = st->Month[1];
	*time++ = st->Year[0];
	*time++ = st->Year[1];

	/********* Internal RTC Time  ***************/		
	temp = hex2bcd(Hr);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*time++ = temp_MSB;
	*time++ = temp_LSB;   

	temp = hex2bcd(Min);
	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*time++ = temp_MSB;
	*time++ = temp_LSB;

	temp = hex2bcd(Sec);

	temp_MSB = 0x30 + (temp >> 4);
	temp_LSB = 0x30 + (temp & 0x0F);

	*time++ = temp_MSB;
	*time++ = temp_LSB;

	if(cmode)                     // footer
	{
		*time++ = 0x0D;          // \r
		*time++ = 0x0A;          // \n
	}
}
/******************************************************/
void bcd2ASCII_time(unsigned char *x,char *data,char *id)
{
	unsigned char temp;
	unsigned char HH_l,MM_l,SS_l,dd_l,mm_l,yy_l;

	*x++;
	*x++;
	SS_l = *x++;
	*x++;
	MM_l = *x++;
	*x++;
	HH_l = *x++;
	*x++;
	dd_l = *x++;
	*x++;
	mm_l = *x++;
	*x++;
	yy_l = *x++;
	*x++;
	*x++;
	*data++ = 0x52;
	*data++ = 0x2F;   
	while(*id != '\0')
	{
		*data++ = *id;
		*id++;
	}
	*data++ = 0x2F;   // send /
	temp = dd_l; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = mm_l; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = yy_l; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = HH_l;
	temp = temp&0x7F; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = MM_l; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = SS_l; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F);

	*data++ = 0x2F;   // send /
	if(result_final1)
	{
		*data++ = 'R';
		*data++ = 'T';
		*data++ = 'C';
		*data++ = ' ';
		*data++ = 'S';
		*data++ = 'E'; 
		*data++ = 'T'; 
	}
	else
	{
		*data++ = 'R';
		*data++ = 'T';
		*data++ = 'C';
		*data++ = ' ';
		*data++ = 'N';
		*data++ = 'O'; 		
		*data++ = 'T';
		*data++ = ' ';
		*data++ = 'S';
		*data++ = 'E'; 
		*data++ = 'T'; 			
	}
}
/******************************************************/
char comparison(struct date_PCU *before_final, struct date_PCU *after_final)
{
	unsigned char result_RTC = 0;
	if(before_final->date == after_final->date)
	{
		if(before_final->month == after_final->month)
		{
			if(before_final->year == after_final->year)
			{
				result_RTC = 1;
			}
		}
	}
	else
	{
		result_RTC = 0;
	}
	
	return result_RTC;
}
/******************************************************/
void put_date(struct date_PCU *before_date,struct date_PCU *after_date)
{
	before_date->date = after_date->date;
	before_date->month = after_date->month;
	before_date->year = after_date->year;
}
/******************************************************/
void ManageBuffer(struct OnlineUPS *values,unsigned int data)
{
	unsigned int FL;
	unsigned char shifts;
	unsigned int select;

	FL=sensitivity_STATUS(&finaldata);
	select = temp_register ^ FL;
	if(select!=0)
	{ 
		shifts = 0;
		while(select!=0)
		{
			shifts++;
			select = select>>1;       
		} 
		switch(shifts)
		{
				case 16://High Temp Shutdown													
						if((values->StatusInverter.parameter & 512)>0) 				 
						{
							if(data&0x00001000 && Event[12].count==0)
							
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x8000;
							Event[12].count++;
							
						}
						else
						{
							temp_register = temp_register&0x7FFF;
						}
						break;
						
			case 15://Short Circuit shutdown											  
						if((values->StatusInverter.parameter & 4)>0)					  
						{
							if(data&0x00000004 && Event[2].count==0)
							
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x4000;
							Event[2].count++;
							
						}
						else
						{
							temp_register = temp_register&0xBFFF;
						} 
						break;			
			
			case 14://battery low shutdown                        
						if((values->StatusInverter.parameter & 64)>0)					 
						{
							if(data&0x00000040 && Event[6].count==0)
						
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x2000;
							Event[6].count++;
							
						}
						else
						{
							temp_register = temp_register&0xDFFF;
						}
						break;
			
      case 13://Overload Shutdown                         
						if((values->StatusInverter.parameter & 16)>0)				 
						{
							if(data&0x00000002 && Event[1].count==0)
							
							{
								prev_conn = 1;
							}
							
							temp_register = temp_register|0x1000;
							Event[1].count++;
							
						}
						else
						{
							temp_register = temp_register&0xEFFF;
						}
						break;						
			
			case 12://Battery High Shutdown														
						if((values->FlagCharging.parameter&8192)>0)             
						{
							if(data&0x00000080 && Event[7].count==0)
							
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x0800;
							Event[7].count++;
							
						}
						else
						{
							temp_register = temp_register&0xF7FF;
						}
						break;		
			
			case 11://High Temp Warning                      
						if((values->StatusInverter.parameter & 4096)>0)           
						{
							if(data&0x00008000 && Event[15].count==0)
							
							{
							prev_conn = 1;
							}
							
							temp_register = temp_register|0x0400;
							Event[15].count++;
							
						}
						else
						{
							temp_register = temp_register&0xFBFF;
						}
						break;

			case 10://Battery Low Warning                          
						if((values->StatusInverter.parameter & 128)>0)           
						{
							if(data&0x00000008 && Event[3].count==0)
							
							{
							prev_conn = 1;
							}
							
							temp_register = temp_register|0x0200;
							Event[3].count++;
							
						}
						else
						{
							temp_register = temp_register&0xFDFF;
						}
						break;
						
			case 9://Overload Warning                              
						if((values->StatusInverter.parameter & 32)>0)           
						{
							if(data&0x00000001 && Event[0].count==0)
							
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x0100;
							Event[0].count++;
							
						}
						else
						{
							temp_register = temp_register&0xFEFF;
						}
						break;
						
			case 8: //PV Current High
					                                      
						if((values->SolarStatus.parameter&4)>0)						
						{
							if(data&0x00002000 && Event[13].count==0)
							{
								prev_conn = 1;
							}	
              temp_register = temp_register|0x0080;
							Event[13].count++;
						}
						else
						{
              temp_register = temp_register&0xFF7F;
						}
						break;
			case 7: //PV Voltage High

						if((values->SolarStatus.parameter&2)>0)						
						{
							if(data&0x00004000 && Event[14].count==0)
							{
								prev_conn = 1;
							}
		
							temp_register = temp_register|0x0040;
							Event[14].count++;
						}
						else
						{
							temp_register = temp_register&0xFFBF;
						}
						break;
						
						
			case 6://Mains Fail																	 
						//if((values->StatusInput.parameter & 128)<1)   
						if((values->OperatingMode.parameter != 4) && (values->OperatingMode.parameter != 6))//Mains Fail
						{
							if(data&0x00000200 && Event[9].count==0)
							
							{
							prev_conn = 1;
							}
								
							temp_register = temp_register|0x0020;							
							Event[9].count++;
							
						}
						else
						{
							
							temp_register = temp_register&0xFFDF;
						}
						break;
						
			case 5://Mains OK																	     
						//if((values->StatusInput.parameter & 128)>0)          
						if((values->OperatingMode.parameter == 4) || (values->OperatingMode.parameter == 6))//Mains OK 
						{
							if(data&0x00000100 && Event[8].count==0)
							
							{
								prev_conn = 1;
							}
							
							
							temp_register = temp_register|0x0010;
							Event[8].count++;
						
						}
						else
						{
						
							temp_register = temp_register&0xFFEF;
						}
						break;						
						
			case 4://Mains High                             
						if((values->StatusInput.parameter & 1024)>0)       
						{
							if(data&0x00000800 && Event[11].count==0)
		
							{
								prev_conn = 1;
							}
						
							temp_register = temp_register|0x0008;	
							Event[11].count++;
			
						}
						else
						{
						
							temp_register = temp_register&0xFFF7;
						}
						break;

			case 3://Mains Low                                      
						if((values->StatusInput.parameter & 2048)>0)						 
						{
							if(data&0x00000400 && Event[10].count==0)
				
							{
								prev_conn = 1;
							}
							
				
							temp_register = temp_register|0x0004;
							Event[10].count++;
					
						}
						else
						{
	
							temp_register = temp_register&0xFFFB;
						}
						break;
												
			case 2://UPS Status ON														      
						if(values->OperatingMode.parameter == 2)           
						{
							if(data&0x00000010 && Event[4].count == 0)
						
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x0002;
							
							Event[4].count++;
						
						}
						else
						{
							temp_register = temp_register&0xFFFD;
						}
						break;
						
			case 1://UPS Status OFF															
						if(values->OperatingMode.parameter == 0)         
						{
							if(data&0x00000020 && Event[5].count == 0)
							
							{
								prev_conn = 1;
							}
							temp_register = temp_register|0x0001;
							Event[5].count++;
						
						}
						else
						{
							temp_register = temp_register&0xFFFE;
						}
						break;

			default:
						break;
		}
	}							 
}
/******************************************************/

/******************************************************/
unsigned char ASCII_TO_BCD(unsigned char ascii_text[2])
{
	unsigned char bcd_value;

	if(ascii_text[0] >= '0' && ascii_text[0] <= '9')  // 0-9 range
	{
		bcd_value = ( ascii_text[0] - 48)  << 4 ; // 48 for '0' ASCII offset
	}
	else if (ascii_text[0] >= 'A' && ascii_text[0] <= 'F') // A-F range
	{
		bcd_value = ( 10 + ascii_text[0] - 65 )  << 4 ; // 65 for 'A' ASCII offset
	}
	else if (ascii_text[0] >= 'a' && ascii_text[0] <= 'f') // a-f range
	{
		bcd_value = ( 10 + ascii_text[0] - 97)  << 4 ; // 97 for 'a'  ASCII offset
	}

	if(ascii_text[1] >= '0' && ascii_text[1] <= '9')  // 0-9 range
	{
		bcd_value |= ( ascii_text[1] - 48); // 48 for '0' ASCII offset
	}
	else if (ascii_text[1] >= 'A' && ascii_text[1] <= 'F') // A-F range
	{
		bcd_value |= ( 10 + ascii_text[1] - 65)   ; // 65 for 'A' ASCII offset
	}
	else if (ascii_text[1] >= 'a' && ascii_text[1] <= 'f') // a-f range
	{
		bcd_value |= ( 10 + ascii_text[1] - 97 ) ; // 97 for 'a' ASCII offset
	}

	return bcd_value;
}
/******************************************************/
void getRTCfromnetwork(struct rtc *time_value,struct networktime *sp)
{
	u32 temp,temp_Hi,temp_Lo;

	temp=sp->Hour&0x7F;
	temp_Hi=0x30+(temp>>4);
	temp_Lo=0x30+(temp&0x0F);

	temp_Hi=temp_Hi<<8;
	time_value->Hour=(temp_Hi+temp_Lo);

	temp=sp->Minute;
	temp_Hi=0x30+(temp>>4);
	temp_Lo=0x30+(temp&0x0F);

	temp_Hi=temp_Hi<<8;
	time_value->Minute=(temp_Hi+temp_Lo);

	temp=sp->Second;
	temp_Hi=0x30+(temp>>4);
	temp_Lo=0x30+(temp&0x0F);

	temp_Hi=temp_Hi<<8;
	time_value->Second=(temp_Hi+temp_Lo);
}
/******************************************************/
/*******************************************************************/
void reset_SCC(struct OnlineUPS *status)
{
	status->StatusWord.parameter = 0;
	status->StatusCommand.parameter = 0;
	status->StatusInverter.parameter = 0;
	status->StatusInput.parameter = 0;
	status->FlagCharging.parameter = 0;
	status->StatusSwitch.parameter = 0;
	status->SystemID.parameter = 0;
	status->SolarStatus.parameter = 0;
  status->OperatingMode.parameter = 0;
	status->Input_Freq.parameter = 0;
	status->Input_V.parameter = 0;
	status->Charging_I.parameter = 0;
	status->Batt_V.parameter = 0;
	status->Output_Freq.parameter = 0;
	status->Output_V.parameter = 0;
	status->Output_I.parameter = 0;
	status->Batt_I.parameter = 0;
	status->Amb_Temp.parameter = 0;
	status->Boost_V.parameter = 0;
	status->PV_V.parameter = 0;
	status->PV_I.parameter = 0;
	status->PVInverterState.parameter = 0;
	status->ControlFversn.parameter = 0;
	status->DisplayFversn.parameter = 0; 
	status->SystemVersn.parameter = 0;
	status->SystemCapacity.parameter = 0;
	status->DCBus.parameter = 0;
	status->BattBackupHr.parameter = 0; 
	status->BattBackupMin.parameter = 0;
	status->Batt_per.parameter = 0;
	status->Load_Per.parameter = 0;
	status->ScalingFin.parameter = 0;
	status->ScalingVin.parameter = 0;
	status->ScalingIchg.parameter = 0;
	status->ScalingVb.parameter = 0;
	status->ScalingFout.parameter = 0;
	status->ScalingVout.parameter = 0;
	status->ScalingIout.parameter = 0;
	status->ScalingIdc.parameter = 0;
	status->ScalingTamb.parameter = 0;
	status->ScalingBattBackupHr.parameter = 0;
	status->ScalingBattBackupMin.parameter = 0;
	status->ScalingBattPer.parameter = 0; 
	status->ScalingLoadPer.parameter = 0;
	status->PVPower.parameter = 0;
}

/******************************************************/
void BuffMAP_MPPTSCC(struct OnlineUPS *datacollect,unsigned char *transfer)
{
	static volatile unsigned char SolarParameterOK;
  
	SolarParameterOK = 0;
	data_ok=0;
	//reset_SCC(&datareceived);

	//switch(cable_connection)
	//{
		//case 1:
			//data_collection = 0;
			if(*transfer++ == '#')		//1
			{
					SolarParameterOK = 1;								// Mapping is OK
			}
			if(SolarParameterOK)
	       		{ 	   
				datacollect->StatusWord.values.MSB 						= *transfer++; //2                        // Status Fault
				datacollect->StatusWord.values.LSB 						= *transfer++; //3                       
				
				datacollect->StatusCommand.values.MSB 				= *transfer++; //4                       // Status System
				datacollect->StatusCommand.values.LSB 				= *transfer++; //5                         

				datacollect->StatusInverter.values.MSB 				= *transfer++; //6
				datacollect->StatusInverter.values.LSB 				= *transfer++; //7
							
        datacollect->StatusInput.values.MSB 					= *transfer++; //8
				datacollect->StatusInput.values.LSB 					= *transfer++; //9
							
				datacollect->FlagCharging.values.MSB 					= *transfer++; //10
				datacollect->FlagCharging.values.LSB 					= *transfer++; //11
				
				datacollect->StatusSwitch.values.MSB 					= *transfer++; //12                         // CumKWH_MSB
				datacollect->StatusSwitch.values.LSB 					= *transfer++; //13      

				datacollect->SystemID.values.MSB 							= *transfer++; //14                         // CumKWH_LSB
				datacollect->SystemID.values.LSB 							= *transfer++; //15
				
				datacollect->SolarStatus.values.MSB    				= *transfer++; //16                  // System Capacity
				datacollect->SolarStatus.values.LSB    				= *transfer++; //17
				
			  datacollect->OperatingMode.values.MSB    			= *transfer++; //18                  // Firmware Version
				datacollect->OperatingMode.values.LSB    			= *transfer++; //19
				
				datacollect->Input_Freq.values.MSB 						= *transfer++; //20                      // PVVoltage_MSB
				datacollect->Input_Freq.values.LSB 						= *transfer++; //21   
				
				datacollect->Input_V.values.MSB 							= *transfer++; //22                                                              // PVVoltage_LSB
				datacollect->Input_V.values.LSB 							= *transfer++; //23
				
				//datacollect->PVVoltage_LSB.values.MSB 			= *transfer++; //22                    // PVVoltage_LSB
				//datacollect->PVVoltage_LSB.values.LSB 			= *transfer++; //23                     

				datacollect->Charging_I.values.MSB 						= *transfer++; //24                      // PVCurrent_MSB
				datacollect->Charging_I.values.LSB 						= *transfer++; //25      

				datacollect->Batt_V.values.MSB 								= *transfer++; //26                                                              // PVCurrent_LSB
				datacollect->Batt_V.values.LSB 								= *transfer++; //27
				
				//datacollect->PVCurrent_LSB.values.MSB 			= *transfer++; //26                    // PVCurrent_LSB
				//datacollect->PVCurrent_LSB.values.LSB 			= *transfer++; //27

				datacollect->Output_Freq.values.MSB   				= *transfer++; //28                           // PVPower
				datacollect->Output_Freq.values.LSB   				= *transfer++; //29

				datacollect->Output_V.values.MSB 							= *transfer++; //30                         // GridVoltage
				datacollect->Output_V.values.LSB 							= *transfer++; //31                        
				
				datacollect->Output_I.values.MSB 							= *transfer++; //32                         // GridCurrent
				datacollect->Output_I.values.LSB 							= *transfer++; //33             
				
				datacollect->Batt_I.values.MSB 								= *transfer++; //34                       // GridFrequency
				datacollect->Batt_I.values.LSB 								= *transfer++; //35  				
				
				datacollect->Amb_Temp.values.MSB 							= *transfer++; //36
				datacollect->Amb_Temp.values.LSB 							= *transfer++; //37                        // HeatSinkTemp
				
				datacollect->Boost_V.values.MSB 							= *transfer++; //38
				datacollect->Boost_V.values.LSB 							= *transfer++; //39                        // Cum_KWH_Day1
				
				datacollect->PV_V.values.MSB 									= *transfer++; //40
				datacollect->PV_V.values.LSB 									= *transfer++; //41                        // Cum_KWH_Day2
				
				datacollect->PV_I.values.MSB			 						= *transfer++; //42
				datacollect->PV_I.values.LSB 									= *transfer++; //43                        // Cum_KWH_Day3
				
				datacollect->PVInverterState.values.MSB 			= *transfer++; //44
				datacollect->PVInverterState.values.MSB 			= *transfer++; //45                        // Cum_KWH_Day4
				
				datacollect->ControlFversn.values.MSB 				= *transfer++; //46
				datacollect->ControlFversn.values.LSB 				= *transfer++; //47                        // Cum_KWH_Day5
				
				datacollect->DisplayFversn.values.MSB 				= *transfer++; //48
				datacollect->DisplayFversn.values.LSB 				= *transfer++; //49                        // Cum_KWH_Day6
				
				datacollect->SystemVersn.values.MSB 					= *transfer++; //50
				datacollect->SystemVersn.values.LSB 					= *transfer++; //51                        // Cum_KWH_Day7
				
				datacollect->SystemCapacity.values.MSB        = *transfer++; //52 
				datacollect->SystemCapacity.values.LSB        = *transfer++; //53
				
				datacollect->DCBus.values.MSB   							= *transfer++; //54
				datacollect->DCBus.values.LSB   							= *transfer++; //55
				
				datacollect->BattBackupHr.values.MSB   				= *transfer++; //56
				datacollect->BattBackupHr.values.LSB   				= *transfer++; //57
				
				datacollect->BattBackupMin.values.MSB   			= *transfer++; //58
				datacollect->BattBackupMin.values.LSB   			= *transfer++; //59
				
			  datacollect->Batt_per.values.MSB   						= *transfer++; //60
			  datacollect->Batt_per.values.LSB   						= *transfer++; //61
				
				datacollect->Load_Per.values.MSB   						= *transfer++; //62
				datacollect->Load_Per.values.LSB   						= *transfer++; //63
				
				datacollect->ScalingFin.values.MSB   					= *transfer++; //64
				datacollect->ScalingFin.values.LSB   					= *transfer++; //65
				
				datacollect->ScalingVin.values.MSB   					= *transfer++; //66
				datacollect->ScalingVin.values.LSB   					= *transfer++; //67
				
				datacollect->ScalingIchg.values.MSB   				= *transfer++; //68
				datacollect->ScalingIchg.values.LSB   				= *transfer++; //69
				
				datacollect->ScalingVb.values.MSB   					= *transfer++; //70
				datacollect->ScalingVb.values.LSB   					= *transfer++; //71
				
				datacollect->ScalingFout.values.MSB  				 	= *transfer++; //72
				datacollect->ScalingFout.values.LSB   				= *transfer++; //73
				
				datacollect->ScalingVout.values.MSB   				= *transfer++; //74
				datacollect->ScalingVout.values.LSB   				= *transfer++; //75
				
				datacollect->ScalingIout.values.MSB   				= *transfer++; //76
				datacollect->ScalingIout.values.LSB   				= *transfer++; //77
				
				datacollect->ScalingIdc.values.MSB   					= *transfer++; //78
				datacollect->ScalingIdc.values.LSB   					= *transfer++; //79
				
				datacollect->ScalingTamb.values.MSB   				= *transfer++; //80
				datacollect->ScalingTamb.values.LSB   				= *transfer++; //81
				
				datacollect->ScalingBattBackupHr.values.MSB   = *transfer++; //82
				datacollect->ScalingBattBackupHr.values.LSB   = *transfer++; //83
				
				datacollect->ScalingBattBackupMin.values.MSB  = *transfer++; //84
				datacollect->ScalingBattBackupMin.values.LSB  = *transfer++; //85
				
				datacollect->ScalingBattPer.values.MSB   			= *transfer++; //86
				datacollect->ScalingBattPer.values.LSB   			= *transfer++; //87
				
				datacollect->ScalingLoadPer.values.MSB   			= *transfer++; //88
				datacollect->ScalingLoadPer.values.LSB   			= *transfer++; //89
				
				
			*transfer++; //90
				
			
				if(*transfer++ == '.') //91
				{
					DataNoGarbage = 1;
				}
				else
				{
					DataNoGarbage = 0;
				}

				if(DataNoGarbage)
				{
					if(chk_Solar == 0)
					{
						chk_Solar = 1;
						FlagforRTC = 1;
					}
					
					data_collection = 0;
					
					datacollect->StatusWord.parameter         	= hex_decimal_conversion(datacollect->StatusWord.values.MSB,datacollect->StatusWord.values.LSB);
					datacollect->StatusCommand.parameter      	= hex_decimal_conversion(datacollect->StatusCommand.values.MSB,datacollect->StatusCommand.values.LSB);
					datacollect->StatusInverter.parameter     	= hex_decimal_conversion(datacollect->StatusInverter.values.MSB,datacollect->StatusInverter.values.LSB);
					datacollect->StatusInput.parameter        	= hex_decimal_conversion(datacollect->StatusInput.values.MSB,datacollect->StatusInput.values.LSB);
					//datacollect->PVVoltage_MSB.parameter    	= hex_decimal_conversion(datacollect->PVVoltage_MSB.values.MSB,datacollect->PVVoltage_MSB.values.LSB);
					//datacollect->PVVoltage_LSB.parameter    	= hex_decimal_conversion(datacollect->PVVoltage_LSB.values.MSB,datacollect->PVVoltage_LSB.values.LSB);
					//datacollect->PVVoltage.parameter        	= hex_decimal_conversion2bytes(datacollect->PVVoltage_MSB.parameter,datacollect->PVVoltage_LSB.parameter);
					datacollect->FlagCharging.parameter       	= hex_decimal_conversion(datacollect->FlagCharging.values.MSB,datacollect->FlagCharging.values.LSB);
					//datacollect->PVCurrent_MSB.parameter    	= hex_decimal_conversion(datacollect->PVCurrent_MSB.values.MSB,datacollect->PVCurrent_MSB.values.LSB);
					//datacollect->PVCurrent_LSB.parameter    	= hex_decimal_conversion(datacollect->PVCurrent_LSB.values.MSB,datacollect->PVCurrent_LSB.values.LSB);	
					//datacollect->PVCurrent.parameter        	= hex_decimal_conversion2bytes(datacollect->PVCurrent_MSB.parameter,datacollect->PVCurrent_LSB.parameter);
					datacollect->StatusSwitch.parameter       	= hex_decimal_conversion(datacollect->StatusSwitch.values.MSB,datacollect->StatusSwitch.values.LSB);
				  datacollect->SystemID.parameter           	= hex_decimal_conversion(datacollect->SystemID.values.MSB,datacollect->SystemID.values.LSB);					
					datacollect->SolarStatus.parameter        	= hex_decimal_conversion(datacollect->SolarStatus.values.MSB,datacollect->SolarStatus.values.LSB);	
				  datacollect->OperatingMode.parameter      	= hex_decimal_conversion(datacollect->OperatingMode.values.MSB,datacollect->OperatingMode.values.LSB);	
					datacollect->Input_Freq.parameter         	= hex_decimal_conversion(datacollect->Input_Freq.values.MSB,datacollect->Input_Freq.values.LSB);	
					datacollect->Input_V.parameter            	= hex_decimal_conversion(datacollect->Input_V.values.MSB,datacollect->Input_V.values.LSB);
					datacollect->Charging_I.parameter         	= hex_decimal_conversion(datacollect->Charging_I.values.MSB,datacollect->Charging_I.values.LSB);
					datacollect->Batt_V.parameter             	= hex_decimal_conversion(datacollect->Batt_V.values.MSB,datacollect->Batt_V.values.LSB);
					datacollect->Output_Freq.parameter    			= hex_decimal_conversion(datacollect->Output_Freq.values.MSB,datacollect->Output_Freq.values.LSB);	
          datacollect->Output_V.parameter  						= hex_decimal_conversion(datacollect->Output_V.values.MSB,datacollect->Output_V.values.LSB);
          datacollect->Output_I.parameter   					= hex_decimal_conversion(datacollect->Output_I.values.MSB,datacollect->Output_I.values.LSB);	
          datacollect->Batt_I.parameter    						= hex_decimal_conversion(datacollect->Batt_I.values.MSB,datacollect->Batt_I.values.LSB);
          datacollect->Amb_Temp.parameter    					= hex_decimal_conversion(datacollect->Amb_Temp.values.MSB,datacollect->Amb_Temp.values.LSB);
          datacollect->Boost_V.parameter    					= hex_decimal_conversion(datacollect->Boost_V.values.MSB,datacollect->Boost_V.values.LSB);
					datacollect->PV_V.parameter    							= hex_decimal_conversion(datacollect->PV_V.values.MSB,datacollect->PV_V.values.LSB);
					datacollect->PV_I.parameter    							= hex_decimal_conversion(datacollect->PV_I.values.MSB,datacollect->PV_I.values.LSB);
			    datacollect->PVInverterState.parameter    	= hex_decimal_conversion(datacollect->PVInverterState.values.MSB,datacollect->PVInverterState.values.LSB);
					datacollect->ControlFversn.parameter    		= hex_decimal_conversion(datacollect->ControlFversn.values.MSB,datacollect->ControlFversn.values.LSB);
					datacollect->DisplayFversn.parameter    		= hex_decimal_conversion(datacollect->DisplayFversn.values.MSB,datacollect->DisplayFversn.values.LSB);
					datacollect->SystemVersn.parameter      		= hex_decimal_conversion(datacollect->SystemVersn.values.MSB,datacollect->SystemVersn.values.LSB);
					datacollect->SystemCapacity.parameter  			= hex_decimal_conversion(datacollect->SystemCapacity.values.MSB,datacollect->SystemCapacity.values.LSB);
					datacollect->DCBus.parameter  							= hex_decimal_conversion(datacollect->DCBus.values.MSB,datacollect->DCBus.values.LSB);
				  datacollect->BattBackupHr.parameter  				= hex_decimal_conversion(datacollect->BattBackupHr.values.MSB,datacollect->BattBackupHr.values.LSB);
          datacollect->BattBackupMin.parameter 		  	= hex_decimal_conversion(datacollect->BattBackupMin.values.MSB,datacollect->BattBackupMin.values.LSB);	
          datacollect->Batt_per.parameter   			 		= hex_decimal_conversion(datacollect->Batt_per.values.MSB,datacollect->Batt_per.values.LSB);
          datacollect->Load_Per.parameter    					= hex_decimal_conversion(datacollect->Load_Per.values.MSB,datacollect->Load_Per.values.LSB);
          //datacollect->Boost_V.parameter    				= hex_decimal_conversion(datacollect->Boost_V.values.MSB,datacollect->Boost_V.values.LSB);
					datacollect->ScalingFin.parameter    				= hex_decimal_conversion(datacollect->ScalingFin.values.MSB,datacollect->ScalingFin.values.LSB);
					datacollect->ScalingVin.parameter    				= hex_decimal_conversion(datacollect->ScalingVin.values.MSB,datacollect->ScalingVin.values.LSB);
					datacollect->ScalingIchg.parameter    			= hex_decimal_conversion(datacollect->ScalingIchg.values.MSB,datacollect->ScalingIchg.values.LSB);
					datacollect->ScalingVb.parameter    				= hex_decimal_conversion(datacollect->ScalingVb.values.MSB,datacollect->ScalingVb.values.LSB);
					datacollect->ScalingFout.parameter      		= hex_decimal_conversion(datacollect->ScalingFout.values.MSB,datacollect->ScalingFout.values.LSB);
					datacollect->ScalingVout.parameter  				= hex_decimal_conversion(datacollect->ScalingVout.values.MSB,datacollect->ScalingVout.values.LSB);
					
					 datacollect->ScalingIout.parameter  				= hex_decimal_conversion(datacollect->ScalingIout.values.MSB,datacollect->ScalingIout.values.LSB);
          datacollect->ScalingIdc.parameter   				= hex_decimal_conversion(datacollect->ScalingIdc.values.MSB,datacollect->ScalingIdc.values.LSB);	
          datacollect->ScalingTamb.parameter    			= hex_decimal_conversion(datacollect->ScalingTamb.values.MSB,datacollect->ScalingTamb.values.LSB);
          datacollect->ScalingBattBackupHr.parameter 	= hex_decimal_conversion(datacollect->ScalingBattBackupHr.values.MSB,datacollect->ScalingBattBackupHr.values.LSB);
          datacollect->ScalingBattBackupMin.parameter = hex_decimal_conversion(datacollect->ScalingBattBackupMin.values.MSB,datacollect->ScalingBattBackupMin.values.LSB);
					datacollect->ScalingBattPer.parameter       = hex_decimal_conversion(datacollect->ScalingBattPer.values.MSB,datacollect->ScalingBattPer.values.LSB);
					datacollect->ScalingLoadPer.parameter    		= hex_decimal_conversion(datacollect->ScalingLoadPer.values.MSB,datacollect->ScalingLoadPer.values.LSB);
					
					
				/*
				  datacollect->StatusFault.parameter    = hex_decimal_conversion(datacollect->StatusFault.values.MSB,datacollect->StatusFault.values.LSB);
					datacollect->StatusSystem.parameter   = hex_decimal_conversion(datacollect->StatusSystem.values.MSB,datacollect->StatusSystem.values.LSB);
					datacollect->SystemCapacity.parameter = hex_decimal_conversion(datacollect->SystemCapacity.values.MSB,datacollect->SystemCapacity.values.LSB);
					datacollect->FirmwareVersion.parameter= hex_decimal_conversion(datacollect->FirmwareVersion.values.MSB,datacollect->FirmwareVersion.values.LSB);
					//datacollect->PVVoltage_MSB.parameter  = hex_decimal_conversion(datacollect->PVVoltage_MSB.values.MSB,datacollect->PVVoltage_MSB.values.LSB);
					//datacollect->PVVoltage_LSB.parameter  = hex_decimal_conversion(datacollect->PVVoltage_LSB.values.MSB,datacollect->PVVoltage_LSB.values.LSB);
					//datacollect->PVVoltage.parameter      = hex_decimal_conversion2bytes(datacollect->PVVoltage_MSB.parameter,datacollect->PVVoltage_LSB.parameter);
					datacollect->PVVoltage.parameter      = hex_decimal_conversion(datacollect->PVVoltage_MSB.values.MSB,datacollect->PVVoltage_MSB.values.LSB);
					//datacollect->PVCurrent_MSB.parameter  = hex_decimal_conversion(datacollect->PVCurrent_MSB.values.MSB,datacollect->PVCurrent_MSB.values.LSB);
					//datacollect->PVCurrent_LSB.parameter  = hex_decimal_conversion(datacollect->PVCurrent_LSB.values.MSB,datacollect->PVCurrent_LSB.values.LSB);	
					//datacollect->PVCurrent.parameter      = hex_decimal_conversion2bytes(datacollect->PVCurrent_MSB.parameter,datacollect->PVCurrent_LSB.parameter);
					datacollect->PVCurrent.parameter      = hex_decimal_conversion(datacollect->PVCurrent_MSB.values.MSB,datacollect->PVCurrent_MSB.values.LSB);
				  datacollect->CumKWH_MSB.parameter     = hex_decimal_conversion(datacollect->CumKWH_MSB.values.MSB,datacollect->CumKWH_MSB.values.LSB);
					datacollect->CumKWH_LSB.parameter     = hex_decimal_conversion(datacollect->CumKWH_LSB.values.MSB,datacollect->CumKWH_LSB.values.LSB);	
					datacollect->CumKWH.parameter         = hex_decimal_conversion2bytes(datacollect->CumKWH_MSB.parameter,datacollect->CumKWH_LSB.parameter);
					datacollect->PVPower.parameter        = hex_decimal_conversion(datacollect->PVPower.values.MSB,datacollect->PVPower.values.LSB);
					datacollect->GridVoltage.parameter    = hex_decimal_conversion(datacollect->GridVoltage.values.MSB,datacollect->GridVoltage.values.LSB);
					datacollect->GridCurrent.parameter    = hex_decimal_conversion(datacollect->GridCurrent.values.MSB,datacollect->GridCurrent.values.LSB);	
          datacollect->GridFrequency.parameter  = hex_decimal_conversion(datacollect->GridFrequency.values.MSB,datacollect->GridFrequency.values.LSB);
          datacollect->HeatSinkTemp.parameter   = hex_decimal_conversion(datacollect->HeatSinkTemp.values.MSB,datacollect->HeatSinkTemp.values.LSB);	
          datacollect->CumKwh_Day1.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day1.values.MSB,datacollect->CumKwh_Day1.values.LSB);
          datacollect->CumKwh_Day2.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day2.values.MSB,datacollect->CumKwh_Day2.values.LSB);
          datacollect->CumKwh_Day3.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day3.values.MSB,datacollect->CumKwh_Day3.values.LSB);
					datacollect->CumKwh_Day4.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day4.values.MSB,datacollect->CumKwh_Day4.values.LSB);
					datacollect->CumKwh_Day5.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day5.values.MSB,datacollect->CumKwh_Day5.values.LSB);
					datacollect->CumKwh_Day6.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day6.values.MSB,datacollect->CumKwh_Day6.values.LSB);
					datacollect->CumKwh_Day7.parameter    = hex_decimal_conversion(datacollect->CumKwh_Day7.values.MSB,datacollect->CumKwh_Day7.values.LSB);
					datacollect->last_date.parameter      = hex_decimal_conversion(datacollect->last_date.values.MSB,datacollect->last_date.values.LSB);
					datacollect->Last_cum_kwh1.parameter  = hex_decimal_conversion(datacollect->Last_cum_kwh1.values.MSB,datacollect->Last_cum_kwh1.values.LSB);
					datacollect->Last_cum_kwh2.parameter  = hex_decimal_conversion(datacollect->Last_cum_kwh2.values.MSB,datacollect->Last_cum_kwh2.values.LSB);	
*/
/***********************************/
         datacollect->PVPower.parameter = (datacollect->PV_I.parameter)*(datacollect->PV_V.parameter);
         datacollect->PVPower.parameter = (datacollect->PVPower.parameter)/10;
pv_current = datacollect->PV_I.parameter;
pv_voltage = datacollect->PV_V.parameter;
/*********************************** KwH Calculation **************************************************************/

					CUMKWH_M   = hex_decimal_conversion(CurrentPosition.bytes[3],CurrentPosition.bytes[2]);
					CUMKWH_L   = hex_decimal_conversion(CurrentPosition.bytes[1],CurrentPosition.bytes[0]);	
					CurrentPosition.CUM_KWH     = hex_decimal_conversion2bytes(CUMKWH_M,CUMKWH_L);
data_ok=1;

						
/**********************************************************************/
}
			}
			
		/*	break;
		case 0:
			itt_count++;
			if(itt_count > 2)
			{
				itt_count = 0;
				if(data_collection == 0)
				{
					data_collection = 1;
					//value_change_sign = 1;
					chk_Solar = 0;
				}
				else
				{
					value_change_sign = 0;
				}
				reset_SCC(&datareceived);	
			}	 
			break;
	}*/
					if(Send_Msg_To_Server_Once)
			{
				Send_Msg_To_Server_Once = 0;
				prev_conn = 1;
			}
}


