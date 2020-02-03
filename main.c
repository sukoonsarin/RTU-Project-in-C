/**
******************************************************************************
 @brief   : Main program body
 @name    : Hybrid GTI Solar Inverter GSM/GPRS RTU(Remote Terminal Unit)
 @version : V1.20
 @author  : Sukam Power Systems Ltd., Research and Development Centre
 @date    : June 2018
******************************************************************************


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "main.h"
#include "ATCGSM.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "sdcard_drivers.h"
#include "sdcard.h"
#include "math.h"
#include "parameters.h"

unsigned char Server_No_New[] = {"9999105839"}; //ADD1 //Sukoon Testing Number
unsigned char *pntr; //ADD2
//*******************************
extern unsigned long Watt_Hour_Cum;
extern unsigned int Watt_Hour;
extern unsigned long Watt_Sec;
char Reset_Error[1]; //sukoonerror
char KWH_Factor[1];
extern long PV_POWER;
char CUMKWH_FLASH[1];
unsigned char start_counter=0;

//*******************************

#define      INPUT_COIL               0x00 //NOT USED
#define      HOLDING_REGISTER         0x01 //NOT USED

#define      LED_OFF                  GPIO_SetBits(GPIOB,GPIO_Pin_12);
#define      LED_ON                   GPIO_ResetBits(GPIOB,GPIO_Pin_12);

#define      MESSAGE_OFF              GPIO_SetBits(GPIOB,GPIO_Pin_13);
#define      MESSAGE_ON     		 			GPIO_ResetBits(GPIOB,GPIO_Pin_13);

#define      PWRKey_ON                GPIO_SetBits(GPIOB,GPIO_Pin_1);
#define      PWRKey_OFF     		  		GPIO_ResetBits(GPIOB,GPIO_Pin_1);

#define      SIM800_ON                GPIO_SetBits(GPIOB,GPIO_Pin_15);
#define      SIM800_OFF     		  		GPIO_ResetBits(GPIOB,GPIO_Pin_15);

#define      DTR_PULLDOWN             GPIO_ResetBits(GPIOA,GPIO_Pin_0);
#define      DTR_PULLUP               GPIO_SetBits(GPIOA,GPIO_Pin_0);

#define      CTS_LOW                  GPIO_ResetBits(GPIOA,GPIO_Pin_12);

#define      DCD                      GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);

#define      StartAddr        ((u32)0x0801F800)
#define      EndAddr          ((u32)0x0801FBFF)
#define      FLASH_PAGE_SIZE  ((u16)0x400)

#define      LsiFreq           40000

#define      GSM_MODE          0x00
#define      GPRS_MODE         0x01

#define        OK              1
#define      Configured        0
#define      Not_Configured    1

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
typedef enum {ERR_OK=1,ERR_APN,ERR_PDP,ERR_SERVER,ERR_OTHER}MSG;
typedef enum {AUTOMATIC = 0,GPRS_ONLY,GSM_ONLY,BLOCK_COMM}MODE;
typedef enum {CMDM = 0, DATM = !CMDM} ConStatus;

volatile u32 NewAlarm;
volatile FLASH_Status FLASHStatus;
volatile TestStatus MemoryProgramStatus;
volatile u32 NewCounter=0;
volatile char *point;
volatile char SMS_FLAG;
volatile char *ptr_one;
volatile char Site_Status; //NOT USED
volatile unsigned char page_write; //NOT USED
volatile unsigned char checkedOK=0;
volatile unsigned char SDcard_chk;
volatile unsigned char Sd_Write,CSQ_time_count;
volatile unsigned char compare,SIM_chk,GSM_mode,PDU_TEXT_mode,PDU_mode,status_OK,set_serviceAD,dial_number_response,set_response,hard_flow,validity_set,gprs_en,IPhead,gprs_mux;
volatile unsigned char Counter;
volatile unsigned char Signal_Strength;
volatile unsigned char GPRS_retry, Data_Log_Time, Sd_Write, CSQ_time_count, Send_Status,reset_status=0,message_status;
volatile unsigned char RESET_COUNT_FLAG=0;
volatile unsigned char RESET_FLAG=0;
volatile unsigned int Count=0;
volatile unsigned int time_count;
volatile unsigned int CSQ_timeout;
volatile unsigned int temp_register=0;
volatile unsigned int delay_ADC=0; //NOT USED
volatile unsigned int delay_count;
volatile unsigned char Enable_RTC,RTC_without_update,Disable_RF,Resetting_RF;
volatile unsigned char debug_RTC=0;
volatile unsigned int Config_count=0; //NOT USED

extern char FILEN2[10];
extern char FILEN4[10]; //NOT USED
extern char IPAdd[20];
extern char Autoreport0[8]; //NOT USED
extern char Autoreport1[8]; //NOT USED
extern char TFRAME[15];
extern char SiteName[15];
extern char SIM_slot[5];
extern char SiteId[15];
extern char KWH_update[15]; //14JUNE
extern char Alarm_config[15];
extern char Rx1Buffer[400];	
extern char CSQBuffer[100];
extern volatile unsigned char comm_ok,prev_conn,prev_conn_KWH;
extern volatile unsigned char comm_error, one_flag;
extern volatile unsigned char SMS_Received;
extern volatile unsigned char FlagforRTC;
extern volatile unsigned char cable_connection;
extern volatile unsigned char DMA_TC_FLAG,EXT_INT_FLAG;
extern volatile unsigned char value_change_sign; //NOT USED
extern unsigned int Alert0; //NOT USED
extern unsigned int Alert1; //NOT USED
extern unsigned int Alert2; //NOT USED
extern unsigned int Alert3; //NOT USED
extern unsigned int Alert4; //NOT USED
extern unsigned int Alert5; //NOT USED
extern unsigned int Alert6; //NOT USED
extern unsigned int Alert7; //NOT USED
extern unsigned int Alert8; //NOT USED
extern unsigned int Alert9; //NOT USED
extern unsigned int Alert10; //NOT USED
extern unsigned int Alert11; //NOT USED
extern unsigned int dial_num_test;
extern unsigned int configurable;
extern unsigned char result_final1;
extern unsigned char RxBuffer[200];
extern unsigned char Rx2Buffer[200]; //ADD3
extern unsigned char sync_flag; //NOT USED
extern unsigned char SetAlarmIndication;
extern unsigned char DataNoGarbage; //NOT USED
extern unsigned char validSMS;
extern unsigned char result_count; //NOT USED
extern unsigned char Receive_SMS; //NOT USED
extern volatile unsigned int AvgSUMadc; 
extern volatile unsigned int RTC_sec_count;
extern volatile char SMS_FLAG;	

struct Time4Alarm
{
	char Time[5];
	unsigned int time_event;
	unsigned int independent_counter;
	unsigned int count;
};
		
struct Time4NetworkSync
{
	unsigned char Date[2];
	unsigned char Month[2];
	unsigned char Year[2];
	unsigned char Hour[2];
	unsigned char Minute[2];
	unsigned char Second[2];
};
		
struct rtc
{
	u32 Hour;
	u32 Minute;
	u32 Second;
}GSM,AutoGen;

struct verifytime
{
	unsigned char verifyHour;
	unsigned char verifyMinute;
	unsigned char verifySecond;
};
	
struct GPRS
{
	char ServerIP[20];
	char ServerPort[10];
	char mode[5];
}Gparam;

struct GprsSet
{
	char APN[50];
	char log_time_gprs[10];
}Gpset;
	
struct MSSG
{
	char DIAL_NUM[15];
	char MESSAGE[15];
};

struct Mobile
{
	char No1[15];
	char No2[15];
}Ph;
		
struct Reply
{
	char PhNo1[15];
	char PhNo2[15];
	char Site[15];
};
		
struct Auto
{
	char AutoReport[8];
};
		  
struct timeformat
{
	unsigned char dd;
	unsigned char mm;
	unsigned char yy;
	unsigned char HH;
	unsigned char MM;
	unsigned char SS;
};
	  
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

struct networktime
{
	unsigned char Hour;
	unsigned char Minute;
	unsigned char Second;
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

struct typeconverter converter;

union CumulativeKWH
{
   unsigned long CUM_KWH;
		char bytes[4];
};

union CumulativeKWH CurrentPosition;
extern struct Time4Alarm Event[32];     
extern struct Reply PHvalue; 
extern struct Auto daily[12];
extern struct BYTE1 buff;
extern struct RTC_PCU Verify;
extern struct date_PCU before_RTC,after_RTC;
extern struct timeformat SP1;	
extern oneP datareceived,finaldata;
struct Time4NetworkSync Sync;	 
struct networktime time;
struct verifytime verify;
struct typeconverter converter;
struct MSSG Msg[10];

char Fver[5] = "302";
char *local_port = "5000";
char RTU_State;
char *RTC_value;	 
char Rx0Buff[400];
char Rx1Buff[400];
char Rx2Buff[400];
char Rx3Buff[400];
char Rx4Buff[400];
char Rx5Buff[400];
char Rx6Buff[400];
char Rx7Buff[400];
char IP0Buff[400];
char IP1Buff[400];
char IP2Buff[400];
char IP3Buff[400];
char IP4Buff[400];
char IP5Buff[400];
char IP6Buff[400];
char IP7Buff[400];
char ConnMode;
char RTU_MODE,DataStatus;
char FIRM_Version[20];
char *AT1;
char *strcp;
char *strcp1;
char *smt;
char *AT_BUFFER;
char separator[]=";";
char len;
char Service_Ad[15];
char ARRAY_TRANS[200];
char PCU_RTC[30];
char ALARM_TR[200];
char LoggData[200];
char Chk_state[10];
char flash_memory[400];
char *AT_ECHO;
char AT_ECHO_SAVE[200];
char LoggData[200];
char tx_bff[100];
char *AT_COMMAND[] = {"AT","AT+CPIN?","AT+CMGF=?","AT+CMGF=1","AT+CMGF=0","AT+CPMS?","AT+CSCA?","AT+CSMP?","AT+CPMS=\"SM\"","AT+CNMI=2,2,0,0,0","AT+CMGL=\"REC UNREAD\",0","ATE0","ATE1","AT+CMGDA=\"DEL ALL\"","AT+CMGD=1","AT+CSCLK=2","AT+CSAS=0","AT+CSAS?","AT+IPR=9600"};
static char *AT_RESPONSE[] = {"AT\r\r\nOK\r\n","AT+CPIN?\r\r\n+CPIN: READY\r\n\r\nOK\r\n","AT+CPIN?\r\r\nERROR\r\n","AT+CSCS=\"GSM\"\r\r\nOK\r\n","AT+CMGF=?\r\r\n+CMGF: (0,1)\r\n\r\nOK\r\n","AT+CMGF=1\r\r\nOK\r\n","\r\nOK\r\n","\r\r\n> ","OK","AT+IPR=9600\r\n\r\nOK\r\n","RDY","\0"};

/* Variables defined for flash */
uint8_t TU[512]={0};
uint8_t RU[512]={0};

uint32_t i=0,j;
vu32 vu32_TimingDelay=0;
u16 u16_Tempcounter=0x00;
ErrorStatus HSEStartUpStatus;
u32 NewRTCvalue;
u16 NewRTCvalue_Hi,NewRTCvalue_Lo;
u16 NewAlarm_Hi,NewAlarm_Lo;

unsigned int Alert[24];
unsigned int lg;
unsigned int SD_Flag=0;
unsigned int LED_INDICATOR=0;
unsigned int Block_LED=0;
unsigned int No_of_bytes_to_read,No_of_bytes_to_write;
unsigned int time_capture = 0;
unsigned int Alert[24];
unsigned int time_out_ms;
unsigned int result;
unsigned int increment,__configure;
unsigned int Page_StAddress,PageNo;
unsigned int increment,__configure;
unsigned int BLOCKsize,test;
unsigned int reconfig = 0;
unsigned char all_message_delete;
unsigned char  RXBufferSize;
unsigned char TRANSMIT_BUFFER[50];
unsigned char changeRTCvalue;
unsigned char AT_ack = 0,auto_baud=0;
unsigned char result_final=0;
unsigned char datawrite_success;
unsigned char msg_slot,tag;
unsigned char ACEnergy,DCEnergy,SolarP;
unsigned char compare_response;
unsigned char tempRece[50];
unsigned char *tpr=tempRece;
unsigned char __in;
unsigned char RTC_Check_Count=0;
unsigned char TIME_FRAME[15];
unsigned char RTC_corrupt_flag = 0;
unsigned char debug=0,chk_alarm;
unsigned long EraseCounter = 0x00, Address = 0x00;
unsigned long Data;
unsigned long NbrOfPage = 0x00;
unsigned char Send_Msg_To_Server_Once=0; //ADD

extern void start_DMA_access1_receive(unsigned int,char *);
extern void reply_xconp(char *,char,char *,char);
extern void reply_xname(char *id,char *snam, char *drag,char cmode);
extern void reply_fname(char *id,char *snam,char *fname,struct OnlineUPS *, char *drag,char cmode);
extern void start_DMA_accessCSQ(unsigned int);
extern void RTCinterrupt(void);
extern void getRTCfromnetwork(struct rtc *,struct networktime *);
extern void getRTCvalue(char *,struct rtc *);
extern void autoTIMEset(char *,struct rtc *);
extern void ExtractTimefromRTC(struct rtc *,unsigned char *);
extern void start_DMA_access2(unsigned int);
extern void set_alert(unsigned int *,unsigned char ,unsigned char );
extern void sort(unsigned int *,int );
extern void ASCII_alignation_OnlineUPS(char *storage,struct OnlineUPS *UPSDATA,struct Time4NetworkSync *,char *id,char *Fversion,unsigned char type,char cmode,char tower,char xph);
extern void RTC_on(void);
extern void initializeRTC(struct timeformat *);
extern void Read_multiple_register(unsigned char ,unsigned int ,unsigned int ,unsigned char ,unsigned char ,unsigned char *);
extern void Write_multiple_register(unsigned char ,unsigned int ,unsigned int ,unsigned char ,unsigned char *,unsigned char *);
extern void read_SMS_command( unsigned char ,unsigned char *,char * );
extern void ASCII_hex_BCD_conversion(struct timeformat *,char *);
extern void write_RTCvalue(struct timeformat *);
extern void decimal_ASCII_conversion(unsigned int,char *);
extern void ASCII_alignation_UPS_mobile_reader(char *,struct OnlineUPS *,unsigned char);
extern void send_MobileNo(struct Mobile *,char *);
extern void reply_reset(char *, char *id);
extern void reply_extract_phoneNo(char *id,char *drag,char cmode);
extern void BuffMAPOnlineUPS(struct OnlineUPS *,unsigned char *);
extern void BuffMAP_MPPTSCC(struct OnlineUPS *,unsigned char *); //ADD5
extern void ManageBuffer(struct OnlineUPS *,unsigned int);
//extern void ManageBuffer(struct MPPTSCC *,struct MPPTSCC *,struct MPPTSCC *,unsigned int); //ADD4
extern void reply_MobileNo(struct Mobile *ext,char *dt,char *id,char cmode);
extern void delete_SMS(char *);
extern void start_DMA_access(unsigned int);
extern void start_DMA_access1(unsigned int);
extern void DMA_RX2Interrupt(void);
extern void DMA_RxInterrupt(void);
extern void DMA_Rx1Interrupt(void);
extern void USART1_initialisation(void);
extern void UART5_initialisation(void);
extern void USART2_initialisation(void);
extern void USART3_initialisation(void);
extern void RI_interrupt_modeminit(void);
extern void RI_interrupt_modemDeinit(void);
extern void Analog_Initialisation(void);
extern void initialisationIOpins(void);
extern void RTC_Configuration(void);
extern void Timer3_Initialize(void);
extern void reply_Rtime(char *time,char *id,struct Time4NetworkSync *st,char cmode);
extern void LDByteReadI2C( unsigned char , unsigned char , unsigned char , unsigned char * );
extern void Time_Adjust(u32);
extern void Write_BKP_register(unsigned char , char );
extern void reply_ltime(char *,struct GprsSet *,char *,char);
extern void reply_xmode(char *,char *,char);
extern void reply_xgsip(char *,struct GPRS *,char *,char);
extern void reply_ipadd(char *,char *,char *,char *,char );
extern void reply_IPadd(char *dt,char *id,char *IP,char cmode);
extern void reply_xgprt(char *,struct GPRS *,char *,char);
extern void reply_xgapn(char *,struct GprsSet *,char *,char);
extern void dial_number(char *);
extern void reply_extract_phoneNo_config(char *,char *,char *,struct Time4Alarm *,char);
extern void bcd2ASCII_time(unsigned char *x,char *data,char *id);
extern void Reply_xdflt(char *,char,char);
extern void Send_ConfigAll(char *sname,char *id,struct GPRS *gpdt,struct GprsSet *gset,struct Mobile *ext,char *dt,char cmode);
extern void reply_rconp(char *id,char xcn, char *drag,char cmode);
//**********************
extern void reply_KWHXX(char *, char *res_factor,char cmode); //ADD
extern void reply_XCKWH(char *, char); //14JUNE
//**********************
extern unsigned int hex_decimal_conversion(unsigned char ,unsigned char );
extern unsigned int hex_decimal_conversion2bytes(unsigned int ,unsigned int );
extern unsigned char received_msgtype(char *);
extern unsigned char ASCII_TO_BCD(unsigned char ascii_text[2]);
extern unsigned int ASCII_2_decimal(char *);
extern unsigned int verify_RTCvalue(unsigned char *clock,struct timeformat *new);
extern u8 Enter_Value(u32 value);
extern char SMS_control(char *,char);

void SysTick_Configuration(void);
void version(char *firm_version);
void Check_SMS_Buffer(void);
void Check_Task_LiveStatus(void);
void Reset_flash(void);
void strcat_flash(void); //sukoonerror
void write_in_flash(char *);
void Initialize_Date(struct Time4NetworkSync *dmy, char *real);
void Set_Default(void);
void datasend_GSM(void);
void Send_SMS(char *);
void bcd2ASCII_RTIME(struct networktime *,struct Time4NetworkSync *,char *,char *);
void phone_numbers_map(void);
void clear_ATbuffer(void);
void Initialize_Time(struct networktime *hhmmss, struct timeformat *tt);
void datasend_Alarm(void);
void delete_buffer(void);
void haultCall(void);
void Check_GSM_Network(void);
void Send_Data_SMS(void);
void Send_Data_SMS1(void); //ADD
void Send_Data_SMS2(void); //ADD
void Call_CSQ_Module(void);
void Check_GPRS_Buffer(void);
void ReplyRESET_Hardware(char *,char *);
void ASCII_2_HEX(char *,struct verifytime *);

char GSM_Initialize(void);
char read_4m_flash(char *flash);
char check_GSM_MODE(void);
char sync_RTC(void);

unsigned int delay_ms(volatile unsigned int);

/***************** TCPIP  *************************/
extern void Switch2CommandMode(void);
extern void Switch2DataMode(void);
extern void CloseConnection(void);
extern void ShutConn(void);
extern void Send_Data_GPRS(void);
extern void GetClientIP(void);
extern char Connect2Server(void);
extern char Connection(void);
extern char GetGprsState(void);
extern char Check_IPStatus(void);

u32 Time_Regulate(struct rtc*);



/*******************************************************************/
unsigned int delay_ms(volatile unsigned int delay_temp)
{
   delay_count = delay_temp;
   while(delay_count);
   return(delay_count);
 }
/*******************************************************************/
void Set_Default(void)
{
	 int k,kk; char *def[] = {"0","SU-KAM","1200","30","65279","120"}; //DOUBT1
   char table[6];

   memset(Ph.No1,'\0',15);
   memset(Ph.No2,'\0',15);
   memset(SiteName,'\0',15);
	 memset(SiteId,'\0',15);
	 memset(Alarm_config,'\0',15);
	 memset(Gparam.ServerIP,'\0',15);
	 memset(Gpset.APN,'\0',50);
	 memset(Gparam.ServerPort,'\0',15);
	 memset(Gparam.mode,'\0',5);
	 memset(Gpset.log_time_gprs,'\0',10);
   memset(converter.xcon,'\0',2);
	 memset(converter.No_of_Alerts,'\0',2);
   memset(Reset_Error,'\0',1); //sukoonerror
   memset(KWH_Factor,'\0',1); //ADD
	 memset(CUMKWH_FLASH,'\0',1);

	for(k=0;k<12;k++)
	{
		memset(daily[k].AutoReport,'\0',8);
	}

	for(k=0;k<25;k++)
	{
		memset(Event[k].Time,'\0',5);
	}

	converter.No_of_Alerts[0] = 1;
	converter.NoAlerts = 1;
	converter.xcon[0] = 2;
	converter.xcoonh = 2;
    Reset_Error[0]=0; //sukoonerror
	KWH_Factor[0]=0;//ADD
	CurrentPosition.bytes[0]=0;//SUKI
	CurrentPosition.bytes[1]=0;//SUKI
	CurrentPosition.bytes[2]=0;//SUKI
	CurrentPosition.bytes[3]=0;//SUKI

	
	strcpy(SiteId,def[0]);
	strcpy(SiteName,def[1]);
	strcpy(daily[0].AutoReport,def[2]);
	strcpy(Gparam.mode,def[0]);
	
	converter.RTU_OP_MODE[0] = ASCII_2_decimal(Gparam.mode);
	converter.OPER_MODE = ASCII_2_decimal(Gparam.mode);
	
	for(i=0;i<converter.NoAlerts;i++)
	{
		autoTIMEset(daily[i].AutoReport,&AutoGen);
		Alert[i]=Time_Regulate(&AutoGen);
	}

	sort(Alert,converter.NoAlerts);
	
	strcpy(Gpset.log_time_gprs,def[3]);
	
	converter.LogTimeGprs[0] = ASCII_2_decimal(Gpset.log_time_gprs);
	converter.TimeGprs = ASCII_2_decimal(Gpset.log_time_gprs);
	
	strcpy(Alarm_config,def[4]);

	for(k=0;k<16;k++) //EVENTSS
	{
		strcpy(Event[k].Time,def[5]);
	}

	configurable = ASCII_2_decimal(Alarm_config);

	for(k=0;k<16;k++) //EVENTSS
	{
		memset(table,'\0',6);
		kk = 0;
		while(Event[k].Time[kk] != '\0')
		{
			table[kk] = Event[k].Time[kk];
			kk++;
		}
		Event[k].time_event = ASCII_2_decimal(table);
	}
}
/*******************************************************************/
void Reset_flash(void)
{
  FLASHStatus = FLASH_COMPLETE;
  MemoryProgramStatus = PASSED;
  Data = 0xFFFFFFFF;

  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();

  /* Define the number of page to be erased */
  NbrOfPage = 1;
  /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  /* Erase the FLASH pages */
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(StartAddr + (FLASH_PAGE_SIZE * EraseCounter));
  }

  /*  FLASH Word program of data flash-memory at addresses defined by StartAddr and EndAddr*/
  Address = StartAddr;

  while((Address < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
  {
    
		FLASHStatus = FLASH_ProgramWord(Address,Data);
		Address = Address + 4;
  }
  /* Unlock the Flash Program Erase controller */
  FLASH_Lock();
}
/*******************************************************************/
void write_in_flash(char *flash)
{
  FLASHStatus = FLASH_COMPLETE;
  MemoryProgramStatus = PASSED;

  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();

  /* Define the number of page to be erased */
  NbrOfPage = 1;
	
  /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  /* Erase the FLASH pages */
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(StartAddr + (FLASH_PAGE_SIZE * EraseCounter));
  }

  /*  FLASH Word program of data flash-memory at addresses defined by StartAddr and EndAddr*/
  Address = StartAddr;

  while((Address < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
  {
    Data = (unsigned long)*flash;
		FLASHStatus = FLASH_ProgramWord(Address,Data);
		*flash++;
    Address = Address + 4;
  }
  /* Unlock the Flash Program Erase controller */
  FLASH_Lock();
}
/*******************************************************************/
char read_4m_flash(char *flash)
{
 	char return_var = 0;
	int d;
	
  FLASH_Unlock();
	
  /* Check the corectness of written data */
  Address = StartAddr;

  while((Address < EndAddr))
  {
    Data = (*(vu32*)Address);
		if(Data != 0xFFFFFFFF)
	  {
	    *flash++ = Data;
	  }
		else
	  {
	    *flash++ = '\0';
	  }
    Address = Address + 4;
  }
  FLASH_Lock();
	
	for(d=0;d<500;d++)
	{
		if(*strcp == '\0')
    *strcp++;
    else
		{
      return_var = 1;
      break;
		}
	}
	return return_var;
}
/*******************************************************************/
void phone_numbers_map(void)
{
   memset(PHvalue.PhNo1,'\0',15);
   memset(PHvalue.PhNo2,'\0',15);
   memset(PHvalue.Site,'\0',15);

   strcp = Ph.No1;
   strcp1 = PHvalue.PhNo1;
   strcpy(strcp1,strcp);

   strcp = Ph.No2;
   strcp1 = PHvalue.PhNo2;
   strcpy(strcp1,strcp);

   strcp = SiteId;
   strcp1 = PHvalue.Site;
   strcpy(strcp1,strcp);
}
/*******************************************************************/
void clear_ATbuffer(void)
{
	 memset(Rx1Buffer,'\0',400);
   memset(AT_ECHO_SAVE,'\0',100);
   AT_BUFFER = &Rx1Buffer[0];
	 AT_ECHO = &AT_ECHO_SAVE[0];
   start_DMA_access1(400);
}
/*******************************************************************/
 void Check_Task_CSQ(void)
{
	if(CSQ_time_count)
	{
		CSQ_time_count = RESET;
		Call_CSQ_Module();
	}
}
/*******************************************************************/
void Check_RTU_Mode_DCD(void)
{
		char dcd_state;

		dcd_state = DCD;         // DCD high NO gprs
		if(dcd_state == SET)
		{
			#ifdef _GSM_MODE_
			RTU_MODE = GSM_MODE;
			EXT_INT_FLAG = SET;
			#endif
		}
		else
		{
			#ifdef _GPRS_MODE_			// DCD low Connect OK
			RTU_MODE = GPRS_MODE;
			#endif
		}

		#ifdef _GSM_MODE_
		if(Signal_Strength <= 5)
			check_GSM_MODE();
		#endif
}
/*****************************************************************************/
void Check_ExtIntFlag(void)
{
	 if(EXT_INT_FLAG == SET &&  RTU_MODE == GPRS_MODE && ConnMode == 1)
	 {
			IWDG_ReloadCounter();
			delay_ms(2000);
			IWDG_ReloadCounter();
			Switch2CommandMode();
			delay_ms(200);
			IWDG_ReloadCounter();
			if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0))       // Check RI=0 for Call
			{
				haultCall(); 
				Switch2DataMode();
				DMA_Cmd(DMA1_Channel5, DISABLE);
				IWDG_ReloadCounter();
				EXT_INT_FLAG = RESET;
// 				memset(Rx1Buffer,'\0',400);              
// 				start_DMA_access1(7);
			}
	 }
}
/*****************************************************************************/
void Check_Task_GPRSDataLog(void)
{
		if(Data_Log_Time)
		{
			Data_Log_Time = RESET;
			if(RTU_MODE == GPRS_MODE)
			{
				memset(ARRAY_TRANS,'\0',200);
				ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,4,RTU_MODE,Signal_Strength,converter.OPER_MODE);
				Send_Data_GPRS();
				delay_ms(100);
				IWDG_ReloadCounter();
			}
		}
}
/*********************************************************************/
void Check_Task_GPRSConnRetry(void)
{
	if(GPRS_retry && converter.OPER_MODE <= 1)
	{			
		GPRS_retry = RESET;
		if(RTU_MODE == GSM_MODE)
		{
			 if(Gpset.APN[0] != '\0' && Gparam.ServerIP[0] != '\0' && Gparam.ServerPort[0] != '\0')
			 {
					if(Connection() == ERR_OK)
					{
						 RTC_sec_count = RESET; Data_Log_Time = RESET;
				 		 memset(ARRAY_TRANS,'\0',200);
						 reply_IPadd(ARRAY_TRANS,SiteId,IPAdd,GPRS_MODE);
						 IWDG_ReloadCounter();
						 Send_Data_GPRS();
						 IWDG_ReloadCounter();
						 delay_ms(100);
						 IWDG_ReloadCounter();
						 RTU_MODE = GPRS_MODE;
						 EXT_INT_FLAG =RESET;
// 						 memset(Rx1Buffer,'\0',400);              
// 						 start_DMA_access1(7);	
					}
					else
					{
						 RTU_MODE = GSM_MODE;
			    }
		    }
		}
	}
}
/********************************************************************/
void Send_Alarm2Server()
{
	switch(converter.OPER_MODE)
  {
		#ifdef _GPRS_MODE_
		case AUTOMATIC:
				 if(RTU_MODE == GPRS_MODE)
			   {
						memset(ARRAY_TRANS,'\0',200);
						ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						Send_Data_GPRS();
						IWDG_ReloadCounter();
						delay_ms(100);
						IWDG_ReloadCounter();		
					}
				 else if(RTU_MODE == GSM_MODE)
					{
						if(Reset_Error[0] != 1) //sukoonerror
						{
						  memset(ARRAY_TRANS,'\0',200);
							ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
							Send_Data_SMS1();
							IWDG_ReloadCounter();
							delay_ms(500); //12april // changed from 1000
							IWDG_ReloadCounter();		
						}
						
	        	memset(ARRAY_TRANS,'\0',200);
            ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,message_status);		
						Send_Data_SMS2();
						IWDG_ReloadCounter();
						delay_ms(1000);
						IWDG_ReloadCounter();								
				   }
					break;
		#endif
	 
		case GPRS_ONLY:
    		 if(RTU_MODE == GPRS_MODE)
			   {
						memset(ARRAY_TRANS,'\0',200);
						ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						Send_Data_GPRS();
						IWDG_ReloadCounter();
						delay_ms(100);
						IWDG_ReloadCounter();			 				
          }
				  break;

		case GSM_ONLY:
				 if(RTU_MODE == GSM_MODE)
				 {
						if(Reset_Error[0] != 1) //sukoonerror
					  {
						memset(ARRAY_TRANS,'\0',200);
						ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						Send_Data_SMS1();
						IWDG_ReloadCounter();
						delay_ms(500); //12april // changed from 1000
						IWDG_ReloadCounter();	
						}
						
	        	memset(ARRAY_TRANS,'\0',200);
            ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,message_status);		
						Send_Data_SMS2();
						IWDG_ReloadCounter();
						delay_ms(1000);
						IWDG_ReloadCounter();											
					}
					break;
		
		 default:  
          break;
	 }
}
/******************************************************************/
void Send_Alarm2Server_2()
{
	switch(converter.OPER_MODE)
  {
		#ifdef _GPRS_MODE_
		case AUTOMATIC:
				 if(RTU_MODE == GPRS_MODE)
			   {
						memset(ARRAY_TRANS,'\0',200);
						ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						Send_Data_GPRS();
						IWDG_ReloadCounter();
						delay_ms(100);
						IWDG_ReloadCounter();		
					}
				 else if(RTU_MODE == GSM_MODE)
					{
						memset(ARRAY_TRANS,'\0',200);
						//ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,6); //11APRIL //ADD 0 //ADD 6
						Send_Data_SMS2();
						IWDG_ReloadCounter();
						delay_ms(1000);
						IWDG_ReloadCounter();			
				   }
					break;
		#endif
	 
		case GPRS_ONLY:
    		 if(RTU_MODE == GPRS_MODE)
			   {
						memset(ARRAY_TRANS,'\0',200);
						ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						Send_Data_GPRS();
						IWDG_ReloadCounter();
						delay_ms(100);
						IWDG_ReloadCounter();			 				
          }
				  break;

		case GSM_ONLY:
				 if(RTU_MODE == GSM_MODE)
				 {
						memset(ARRAY_TRANS,'\0',200);
						//ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,message_status,RTU_MODE,Signal_Strength,converter.OPER_MODE);
						ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,6); //11APRIL //ADD 0 //ADD 6
					 Send_Data_SMS2();
						IWDG_ReloadCounter();
						delay_ms(1000);
						IWDG_ReloadCounter();									
					}
					break;
		
		 default:  
          break;
	 }
}

/******************************************************************/
void Check_Task_LiveStatus(void)
{
		if(Send_Status)
		{
			Send_Status = RESET;
			if(RTU_MODE == GPRS_MODE)
			{		
				memset(ARRAY_TRANS,'\0',200);    
				reply_IPadd(ARRAY_TRANS,SiteId,IPAdd,GPRS_MODE);
				Send_Data_GPRS();
			}
		}	  
}
/*******************************************************************/
void datasend_GSM(void)
{
	char echo_len;
	char chkOK[7]= "OK\r\n",loop,cmgs[8]="+CMGS:",*cmg="\r\n+CMGS:",emp[8],*cc,*cc1,loop1;
	
	IWDG_ReloadCounter();
	delay_ms(100); 
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE);
	delay_ms(5);   
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',200); 			
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	send_DATA(140,ARRAY_TRANS);
	AT_ECHO = &AT_ECHO_SAVE[0];
	delay_ms(100);	
	time_count = 5000;                
	delay_ms(1000);
	
	while(*AT_BUFFER == '\0'&& time_count != 0);                            // wait 4 buffer to fill     
		                            
  do
	{
			while(*AT_BUFFER == '\0' && time_count != 0);		                        
			if(*AT_ECHO == *AT_BUFFER)
			{					
				*AT_ECHO++; 	
				*AT_BUFFER++;
			}
	 }while((--echo_len > 1) && time_count != 0);
		
	delay_ms(10);			

	 do
	 {
			while(*AT_BUFFER != *cmg && time_count != 0);
			if(time_count != 0)
			{ 
         *AT_BUFFER++; *cmg++; 
      }
			else
		      break;
	 }while(*cmg != '\0' && time_count != 0);
				
	 if(time_count == 0 )
	 {
			delay_ms(500); 		 
			for(loop=0;loop<8;loop++) 
			{
					if(loop==0)
						AT_BUFFER = &Rx1Buffer[0];//&Rx0Buff[0];
					else if(loop == 1)
						AT_BUFFER = &Rx0Buff[0];
					else if(loop == 2)
						AT_BUFFER = &Rx1Buff[0];
					else if(loop==3)
						AT_BUFFER = &Rx2Buff[0];
					else if(loop == 4)
						AT_BUFFER = &Rx3Buff[0]; 
					else if(loop == 5)
						AT_BUFFER = &Rx4Buff[0];
					else if(loop==6)
						AT_BUFFER = &Rx5Buff[0];
					else if(loop == 7)
						AT_BUFFER = &Rx6Buff[0];
					else if(loop == 8)
						AT_BUFFER = &Rx7Buff[0];

					cc = strchr(AT_BUFFER , '+');
					cc1 = cc;
					while(cc != NULL)
					{
						if(*cc !='\0')
						{
							for(loop1=0;loop1<7;loop1++)
							{
								emp[loop1] = *cc++;  
							}
							if(!strncmp(emp,cmgs,5))
							{
								break;
							}						
						}										
						cc = strchr(cc1+1,'+');
						cc1 = cc;
					}
			}
		}
    else
		{
				do
				{
					while(*AT_BUFFER == '\0' && time_count != 0);
					if(*AT_BUFFER != 'O')
					{
						*AT_BUFFER++;
					}
				}while(*AT_BUFFER != 'O' && time_count != 0);
				
				for(loop=0; loop<4; loop++)
				{	
					while(*AT_BUFFER != chkOK[loop] && time_count != 0);
					*AT_BUFFER++;
				} 						
				if(loop >= 3)
					time_count = 0;
				delay_ms(100);
				DMA_Cmd(DMA1_Channel5, DISABLE);
				delay_ms(5); 
		}						 
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		delay_ms(5);   
}
/*************************************************************/
void datasend_Alarm(void)
{
    char chkOK[7]= "OK\r\n",loop,cmgs[8]="+CMGS:",*cmg="\r\n+CMGS:",emp[8],*cc,*cc1,loop1;
		delay_ms(100); 
		
	  DMA_Cmd(DMA1_Channel5, DISABLE);
		delay_ms(5);   
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',200); 			
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
	  start_DMA_access1(400);
		send_DATA(140,ALARM_TR);
		AT_ECHO = &AT_ECHO_SAVE[0];	 	
	  time_count = 5000;              
    delay_ms(1000);
	  
	  while(*AT_BUFFER == '\0'&& time_count != 0);                            // wait 4 buffer to fill     
                           
    do
		{
			 while(*AT_BUFFER == '\0' && time_count != 0);
			 if(*AT_ECHO == *AT_BUFFER)
			 {					
			   *AT_ECHO++; 	
				 *AT_BUFFER++;
			 }
		}while(*AT_ECHO != '\0' && time_count != 0);   
	
	  do
		{
			while(*AT_BUFFER != *cmg && time_count != 0);
			if(time_count != 0)
			{ 
				*AT_BUFFER++; 
				*cmg++; 
			}
			else
				break;
		 }while(*cmg != '\0' && time_count != 0);
				
		 if(time_count == 0 )
		 {							 
        for(loop=0;loop<7;loop++) 
        {
						if(loop==0)
							AT_BUFFER = &Rx0Buff[0];
						else if(loop == 1)
							AT_BUFFER = &Rx1Buff[0];
						else if(loop==2)
							AT_BUFFER = &Rx2Buff[0];
						else if(loop == 3)
							AT_BUFFER = &Rx3Buff[0]; 
						else if(loop == 4)
							AT_BUFFER = &Rx4Buff[0];
						else if(loop==5)
							AT_BUFFER = &Rx5Buff[0];
						else if(loop == 6)
							AT_BUFFER = &Rx6Buff[0];
						else if(loop == 7)
							AT_BUFFER = &Rx7Buff[0];
           		
						cc = strchr(AT_BUFFER , '+');
						cc1 = cc;
						while(cc != NULL)
						{
							if(*cc !='\0')
							{
								for(loop1=0;loop1<7;loop1++)
								{
									emp[loop1] = *cc++;  
								}
								if(!strncmp(emp,cmgs,5))
								{
									break;
								}						
							}										
							cc = strchr(cc1+1,'+');
							cc1 = cc;
						}
				}
			}
      else
			{
					do
					{
						while(*AT_BUFFER == '\0' && time_count != 0);
						
						if(*AT_BUFFER != 'O')
						{
							*AT_BUFFER++;
						}
					}while(*AT_BUFFER != 'O' && time_count != 0);
					for(loop=0; loop<4; loop++)
					{	
						while(*AT_BUFFER != chkOK[loop] && time_count != 0);
						*AT_BUFFER++;
					} 						
					if(loop >= 3 )
						time_count = 0;
					delay_ms(100);
					DMA_Cmd(DMA1_Channel5, DISABLE);
					delay_ms(5);
      }						 
			delay_ms(100);
			DMA_Cmd(DMA1_Channel5, DISABLE);
			delay_ms(5);   
}
/*********************************************************************/
void ASCII_2_HEX(char *hhmmss,struct verifytime *hmins)
{	 
	unsigned char Temp[2],decimal;
	
	*hhmmss++;
	*hhmmss++;
	*hhmmss++;
	*hhmmss++;
	*hhmmss++;
	*hhmmss++;
	
	Temp[0] = *hhmmss++;
	Temp[0] = (Temp[0] & 0x0F);
	Temp[1] = *hhmmss++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);
	hmins->verifyHour = decimal;
	
	Temp[0] = *hhmmss++;
	Temp[0] = (Temp[0] & 0x0F);
	Temp[1] = *hhmmss++;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);
	hmins->verifyMinute = decimal;
	
	Temp[0] = *hhmmss++;
	Temp[0] = (Temp[0] & 0x0F);
	Temp[1] = *hhmmss;
	Temp[1] = (Temp[1] & 0x0F);
	decimal = ((10 *Temp[0])+Temp[1]);
	hmins->verifySecond = decimal;
}
/**********************************************************/
void Send_Data_SMS(void)
{	
		int len;							
		len = strlen(Ph.No1);

		if(len>=10)
		{
			IWDG_ReloadCounter();
			delay_ms(10);   // delay for 10 ms.
			IWDG_ReloadCounter();
			DMA_Cmd(DMA1_Channel5, DISABLE);
			delay_ms(5);   
			clear_ATbuffer();
			dial_number(Ph.No1);
			delay_ms(1000); 
			AT_ECHO = &AT_ECHO_SAVE[0];
			CSQ_timeout = 1500; 
			if(!AT_response("\r\n> "));
				CSQ_timeout = 0;
			delay_ms(20);		
			IWDG_ReloadCounter();
			datasend_GSM();
			IWDG_ReloadCounter();										
			CSQ_timeout = 0;		
			IWDG_ReloadCounter();																	
			delay_ms(200);		
			IWDG_ReloadCounter();				 
		}					
		
		len = strlen(Ph.No2);
			  
		if(len>=10)
		{
		IWDG_ReloadCounter();
		delay_ms(10);   // delay for 10 ms.
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);		
		delay_ms(5);   
		clear_ATbuffer();
		dial_number(Ph.No2);
		delay_ms(1000);
		AT_ECHO = &AT_ECHO_SAVE[0];	 	 
		CSQ_timeout = 1500; 
		if(!AT_response("\r\n> "));
			CSQ_timeout = 0;
		IWDG_ReloadCounter();
		delay_ms(20);			
		IWDG_ReloadCounter();																		
		datasend_GSM();
		IWDG_ReloadCounter();
		CSQ_timeout = 0;		
		IWDG_ReloadCounter();
		delay_ms(200);
		IWDG_ReloadCounter();												
		}	
}		
/**********************************************************/
void Send_Data_SMS1(void) //ADD
{	
		int len;							
		len = strlen(Ph.No1);

		if(len>=10)
		{
			IWDG_ReloadCounter();
			delay_ms(10);   // delay for 10 ms.
			IWDG_ReloadCounter();
			DMA_Cmd(DMA1_Channel5, DISABLE);
			delay_ms(5);   
			clear_ATbuffer();
			dial_number(Ph.No1);
			delay_ms(1000); 
			AT_ECHO = &AT_ECHO_SAVE[0];
			CSQ_timeout = 1500; 
			if(!AT_response("\r\n> "));
				CSQ_timeout = 0;
			delay_ms(20);		
			IWDG_ReloadCounter();
			datasend_GSM();
			IWDG_ReloadCounter();										
			CSQ_timeout = 0;		
			IWDG_ReloadCounter();																	
			delay_ms(200);		
			IWDG_ReloadCounter();				 
		}					
		
}		
/**********************************************************/
void Send_Data_SMS2(void) //ADD
{	
		int len;							
				
		len = strlen(Ph.No2);
			  
		if(len>=10)
		{
		IWDG_ReloadCounter();
		delay_ms(10);   // delay for 10 ms.
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);		
		delay_ms(5);   
		clear_ATbuffer();
		dial_number(Ph.No2);
		delay_ms(1000);
		AT_ECHO = &AT_ECHO_SAVE[0];	 	 
		CSQ_timeout = 1500; 
		if(!AT_response("\r\n> "));
			CSQ_timeout = 0;
		IWDG_ReloadCounter();
		delay_ms(20);			
		IWDG_ReloadCounter();																		
		datasend_GSM();
		IWDG_ReloadCounter();
		CSQ_timeout = 0;		
		IWDG_ReloadCounter();
		delay_ms(200);
		IWDG_ReloadCounter();												
		}	
}		
/**********************************************************/
void delete_buffer(void)
{
		DMA_Cmd(DMA1_Channel5, DISABLE);
		delay_ms(5);   
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 			
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		Send_AT_commands("AT+CMGDA=\"DEL ALL\"\r\n");  	
		AT_ECHO = &AT_ECHO_SAVE[0];	 
		delay_ms(1000);
		if(!AT_response("\r\nOK\r\n"));
			delay_ms(10);   
		DMA_Cmd(DMA1_Channel5, DISABLE);
}
/********************************************/
void version(char *firm_version)
{
	static unsigned char m;
	
	while(*firm_version++ != ':');
	memset(FIRM_Version,'\0',20);
	while(*firm_version != '\r' && *firm_version != '\0')
	{
		FIRM_Version[m++]=*firm_version;
		*firm_version++;
	}
}
/********************************************/
void Initialize_Date(struct Time4NetworkSync *dmy, char *real)
{
	dmy->Date[0] = (unsigned char)*real++;
	dmy->Date[1] = (unsigned char)*real++;
	dmy->Month[0] = (unsigned char)*real++;
	dmy->Month[1] = (unsigned char)*real++;
	dmy->Year[0] = (unsigned char)*real++;
	dmy->Year[1] = (unsigned char)*real;
}
/***************************************************************/
void Initialize_Time(struct networktime *hhmmss, struct timeformat *tt)
{
	hhmmss->Hour = tt->HH;
	hhmmss->Minute = tt->MM;
	hhmmss->Second = tt->SS;
}
/***************************************************************/
char GSM_Initialize(void)
{
  int op,Network_OK;
	volatile char *ptr_one;
  unsigned int network_counter = 0;
	unsigned char ad_int;
	char *space;
	char SIM_SP[4],Chk_state[10];
	char *AT1;
	IPhead= 0;

  MESSAGE_ON
  PWRKey_ON
	LED_ON

	IWDG_ReloadCounter();
  delay_ms(2000);
	IWDG_ReloadCounter();

	MESSAGE_OFF
	PWRKey_OFF
	LED_OFF

  IWDG_ReloadCounter();
  delay_ms(3000);
  IWDG_ReloadCounter();
	
  while(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)); 

	IWDG_ReloadCounter();

  while(*AT_BUFFER++!='\n' && delay_ms(4000));      

	IWDG_ReloadCounter();

	if(*AT_BUFFER == '\0')     
  {
     auto_baud = 1;
	}
  else                                
  {
		auto_baud = 0;
		IWDG_ReloadCounter();
		delay_ms(4000);
		IWDG_ReloadCounter();
	 }

	AT_ack = 0;
	IWDG_ReloadCounter();
  while(AT_ack == 0 && auto_baud==1)              
  {
			DMA_Cmd(DMA1_Channel5, DISABLE);
			memset(Rx1Buffer,'\0',400);
			memset(AT_ECHO_SAVE,'\0',100);
			AT_BUFFER = &Rx1Buffer[0];
			AT_ECHO = &AT_ECHO_SAVE[0];
			start_DMA_access1(400);
			IWDG_ReloadCounter();
			delay_ms(10);
			IWDG_ReloadCounter();
			Send_AT_commands("AT\r\n");
			CSQ_timeout = 500;
			AT_ECHO = &AT_ECHO_SAVE[0];
			op = AT_response("\r\nOK\r\n");
			CSQ_timeout = 0;
			IWDG_ReloadCounter();
			if(op==0)
			{
				AT_ack = 1;                             
			}
			delay_ms(100);
			IWDG_ReloadCounter();
			delay_ms(20);
			IWDG_ReloadCounter();
	}

	IWDG_ReloadCounter();
	delay_ms(1000);
	IWDG_ReloadCounter();
	delay_ms(10);
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100);
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	delay_ms(10);
	IWDG_ReloadCounter();
	Send_AT_commands("AT+IFC=0,0\r\n");
	delay_ms(1000);
	IWDG_ReloadCounter();
	AT_ECHO = &AT_ECHO_SAVE[0];
	if(!AT_response("\r\nOK\r\n"))
	IWDG_ReloadCounter();
	delay_ms(100);
	IWDG_ReloadCounter();


  if(auto_baud == 1)                           
	{
		IWDG_ReloadCounter();
		delay_ms(10);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		delay_ms(10);
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+IPR=9600\r\n");
		CSQ_timeout = 3000;
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		if(!AT_response("\r\nOK\r\n"));
		CSQ_timeout = 0;
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
 }
 else                                                   
 {
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		AT_BUFFER = &Rx1Buffer[0];
		start_DMA_access1(400);
		delay_ms(10);
		Transfer_AT_commands(AT_COMMAND[0]);          
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
		delay_ms(20);
		IWDG_ReloadCounter();
 }
 
	IWDG_ReloadCounter();
	AT_ack = 0;
	IWDG_ReloadCounter();
	delay_ms(10);
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE);
  memset(Rx1Buffer,'\0',400);
	AT_BUFFER = &Rx1Buffer[0]; 
	IWDG_ReloadCounter();
	start_DMA_access1(400);
	IWDG_ReloadCounter();
	USART_SendData(USART1,0x1B);                           
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
	IWDG_ReloadCounter();
	delay_ms(1000);
	IWDG_ReloadCounter();
	Network_OK = 0;
	network_counter = 0;

	while((!Network_OK) && (network_counter++ < 1000))
	{
		delay_ms(10);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);   
		start_DMA_access1(400);
		USART_SendData(USART1,0x1B); 
		
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
		
		IWDG_ReloadCounter();
		delay_ms(500);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(10);
		Send_AT_commands("AT+CSQ\r\n");
		AT_ECHO = &AT_ECHO_SAVE[0];
		IWDG_ReloadCounter();
		delay_ms(3000);
		IWDG_ReloadCounter();
		CSQ_timeout = 3000;
		IWDG_ReloadCounter();
		if(!AT_response("\r\n+CSQ: "))
		{
			CSQ_timeout = 0;
			Signal_Strength = AT_response_CSQ();
			if(Signal_Strength != 0)
			{
				IWDG_ReloadCounter();
				delay_ms(10);
				network_counter = 0;
				Network_OK = 1;
			}
			else
				Network_OK = 0;
			IWDG_ReloadCounter();
			delay_ms(100);
			IWDG_ReloadCounter();
		}
		else
			Network_OK = 0;
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
	}

	IWDG_ReloadCounter();
	delay_ms(10);
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE);
  memset(Rx1Buffer,'\0',400);
	USART_GetFlagStatus(USART1, USART_FLAG_ORE);
	USART_ReceiveData(USART1);
	memset(Rx1Buffer,'\0',400); 
	memset(AT_ECHO_SAVE,'\0',100); 		 
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	delay_ms(10); 
				
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();


	DMA_Cmd(DMA1_Channel5, DISABLE);
	delay_ms(10);
	IWDG_ReloadCounter();
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100);
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	IWDG_ReloadCounter();
	Send_AT_commands("AT+CPIN?\r\n");
	IWDG_ReloadCounter();
	delay_ms(1000);
	IWDG_ReloadCounter();
	CSQ_timeout = 3000;
	IWDG_ReloadCounter();
	AT_ECHO = &AT_ECHO_SAVE[0];
	switch(AT_response("\r\n+CPIN: READY\r\n\r\nOK\r\n"))        
	{
		case 0x00:
			SIM_chk = 1;                
			break;
		case 0x01:
			SIM_chk = 0;
			break;
		default:
			break;
	}
	CSQ_timeout = 0;
	IWDG_ReloadCounter();
	AT_ack = 0;
	delay_ms(100);
	IWDG_ReloadCounter();
	memset(Rx1Buffer,'\0',400);
	USART_GetFlagStatus(USART1, USART_FLAG_ORE);
	USART_ReceiveData(USART1);
	memset(Rx1Buffer,'\0',400); 
	memset(AT_ECHO_SAVE,'\0',100); 		 
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	delay_ms(10); 

	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(10);
	IWDG_ReloadCounter();
		 
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(Rx1Buffer,'\0',400);
	IWDG_ReloadCounter();
	if(SIM_chk==1)                                      
	{
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		delay_ms(10);
		IWDG_ReloadCounter();
		Send_AT_commands("AT+IFC=0,0\r\n");
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		CSQ_timeout = 3000;
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		if(!AT_response("\r\nOK\r\n"))
		{
			CSQ_timeout = 0;
			hard_flow = 1;
		}
		else
		{
			delay_ms(100);
			IWDG_ReloadCounter();
		return 0;
		}
		delay_ms(100);
		IWDG_ReloadCounter();
	}
  if(hard_flow==1)                                      
	{
		IWDG_ReloadCounter();
		delay_ms(10);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		delay_ms(10);
		IWDG_ReloadCounter();
		set_mode(0);	                                   
		AT_ECHO = &AT_ECHO_SAVE[0];
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		CSQ_timeout = 3000;
		if(!AT_response("\r\nOK\r\n"))
		{
			GSM_mode = 1;
		}
		IWDG_ReloadCounter();
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
	}
	if(GSM_mode==1)
	{
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		delay_ms(10);
		IWDG_ReloadCounter();
		Send_AT_commands("AT+CMGF=?\r\n");
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 3000;
		if(!AT_response("\r\n+CMGF: (0,1)\r\n\r\nOK\r\n"))             
		{
			PDU_TEXT_mode = 1;
		}
		IWDG_ReloadCounter();
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
	}
	if(PDU_TEXT_mode==1)
	{
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();	
		Send_AT_commands("AT+CMGF=1\r\n");
		AT_ECHO = &AT_ECHO_SAVE[0];
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		CSQ_timeout = 3000;
		if(!AT_response("\r\nOK\r\n"))        
		{
			PDU_mode = 1;
		}
		IWDG_ReloadCounter();
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
	}
	if(PDU_mode==1)
	{
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		Send_AT_commands("AT+CSMP=17,173,0,0\r\n");    
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 3000;
		if(!AT_response("\r\nOK\r\n"))              
		{
			validity_set = 1;
		}
		IWDG_ReloadCounter();
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
	}
	if(validity_set == 1)                                        
	{
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		space = SIM_SP;
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		Send_AT_commands("AT+CPMS=\"SM\"\r\n");
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 3000;
		if(!AT_response("\r\n+CPMS:"))
		{
			while(*AT_BUFFER != ' ');
			*AT_BUFFER++;
			*AT_BUFFER++;*AT_BUFFER++;
			do
			{
				while(*AT_BUFFER == '\0');
				
				if(*AT_BUFFER != ',')
				{
					*space = *AT_BUFFER++;
					*space++;  
				}
			}while(*AT_BUFFER != ',');
			do
			{
				ptr_one = strchr(Rx1Buffer,'O');
			}while(ptr_one == NULL);
			*ptr_one++;
			
			while(*ptr_one != 'K');	*ptr_one++;
			while(*ptr_one != '\r');	*ptr_one++;
			while(*ptr_one != '\n');	*ptr_one++;

			memset(Chk_state,'\0',10);
			Chk_state[0]='O';Chk_state[1]='K';
			validity_set = 2;
		}
		IWDG_ReloadCounter();
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
	}
	else
		return 0;

	if(validity_set == 2)                                          
	{
		AT1 = Chk_state;
		if(!strcmp(AT1,AT_RESPONSE[8]))
		{
			memset(Rx1Buffer,'\0',400);
			memset(AT_ECHO_SAVE,'\0',100);
			AT_BUFFER = &Rx1Buffer[0];
			AT_ECHO = &AT_ECHO_SAVE[0];
			start_DMA_access1(400);
			IWDG_ReloadCounter();
			delay_ms(1000);
			IWDG_ReloadCounter();
			IWDG_ReloadCounter();
			Send_AT_commands("AT+CSCA?\r\n");
			CSQ_timeout = 3000;
			AT_ECHO = &AT_ECHO_SAVE[0];
			if(!AT_response("\r\n+CSCA: \""))
			{
				IWDG_ReloadCounter();
				delay_ms(100);
				IWDG_ReloadCounter();
				ad_int = 0;
				do
				{
					while(*AT_BUFFER == '\0');
					Service_Ad[ad_int++] = *AT_BUFFER++;
				}while(*AT_BUFFER != 0x22);

				do
				{
					ptr_one = strchr(Rx1Buffer,'O');
				}while(ptr_one == NULL);

				*ptr_one++;
				
				while(*ptr_one != 'K');	*ptr_one++;
				while(*ptr_one != '\r');	*ptr_one++;
				while(*ptr_one != '\n');	*ptr_one++;

				IWDG_ReloadCounter();
				delay_ms(10);
				DMA_Cmd(DMA1_Channel5, DISABLE);
				IWDG_ReloadCounter();
				memset(Rx1Buffer,'\0',400);
				memset(AT_ECHO_SAVE,'\0',100);
				AT_BUFFER = &Rx1Buffer[0];
				AT_ECHO = &AT_ECHO_SAVE[0];
				start_DMA_access1(400);
				delay_ms(10);
				IWDG_ReloadCounter();
				set_address_service_centre(Service_Ad); 
				Count++;
				AT_ECHO = &AT_ECHO_SAVE[0];
				CSQ_timeout = 3000;
				if(!AT_response("\r\nOK\r\n"))
				{
					set_serviceAD = 1;
				}
				IWDG_ReloadCounter();
				delay_ms(100);
				DMA_Cmd(DMA1_Channel5, DISABLE);
				IWDG_ReloadCounter();
			}
			delay_ms(10);
			IWDG_ReloadCounter();
		}
	}
	else
		return 0;

	if(set_serviceAD == 1)
	{
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		Send_AT_commands("AT+CSAS=0\r\n");
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		while(AT_response("\r\nOK\r\n"));
		
		IWDG_ReloadCounter();
		delay_ms(10);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		delay_ms(10);
		IWDG_ReloadCounter();
		Send_AT_commands("AT+CNMI=2,2,0,0,0\r\n");
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		if(!AT_response("\r\nOK\r\n"))
		{
			delay_ms(10);
			IWDG_ReloadCounter();
			DMA_Cmd(DMA1_Channel5, DISABLE);
			memset(Rx1Buffer,'\0',400);
			memset(AT_ECHO_SAVE,'\0',100);
			AT_BUFFER = &Rx1Buffer[0];
			AT_ECHO = &AT_ECHO_SAVE[0];
			start_DMA_access1(400);
			delay_ms(10);
			IWDG_ReloadCounter();
			Send_AT_commands("AT+CMGDA=\"DEL ALL\"\r\n");
			IWDG_ReloadCounter();
			delay_ms(1000);
			IWDG_ReloadCounter();
			AT_ECHO = &AT_ECHO_SAVE[0];
			Count++;
			if(!AT_response("\r\nOK\r\n"))
			{
				delay_ms(10);   
				IWDG_ReloadCounter();
			}
		}
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		
		AT_ack = 0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+CGMR\r\n");     
		
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		version(&AT_BUFFER[0]);
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();

		AT_ack = 0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+CSGS=0\r\n");                  
		CSQ_timeout = 3000;
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		if(!AT_response("\r\nOK\r\n"));
		CSQ_timeout = 0;
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
		IWDG_ReloadCounter();
	}
	else
		return 0;	 
	 
	IWDG_ReloadCounter();
	delay_ms(100); 
	DMA_Cmd(DMA1_Channel5, DISABLE); 
	IWDG_ReloadCounter();
	IPhead = 0;
	delay_ms(10);   
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100); 			
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	Send_AT_commands("AT+CIPHEAD=1\r\n");		
	AT_ECHO = &AT_ECHO_SAVE[0];	 
	IWDG_ReloadCounter();
	delay_ms(1000);
	IWDG_ReloadCounter();
	if(!AT_response("\r\nOK\r\n"))
		IPhead = 1;
	IWDG_ReloadCounter();
	delay_ms(100);
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE); 
	IWDG_ReloadCounter();
						
	if(IPhead == 1)
	{	
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 			
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		Send_AT_commands("AT+CIPMODE=1\r\n");	
		AT_ECHO = &AT_ECHO_SAVE[0];	
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		if(!AT_response("\r\nOK\r\n"))
		
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE); 
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 			
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		Send_AT_commands("AT+CIPCCFG=5,2,1024,1\r\n");	
		AT_ECHO = &AT_ECHO_SAVE[0];	
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		if(!AT_response("\r\nOK\r\n"))

		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);  
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 			
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		memset(tx_bff,'\0',100);
		
		strcat(tx_bff,"AT+CLPORT=\"TCP\",");
		strcat(tx_bff,local_port);
		strcat(tx_bff,"\r\n");

		Send_AT_commands(tx_bff);	
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];	
		if(!AT_response("\r\nOK\r\n"))
		
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE);  
		IWDG_ReloadCounter();
	}
									
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100); 			
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	Send_AT_commands("AT+CGATT?\r\n");
	IWDG_ReloadCounter();
	delay_ms(1000);	
	IWDG_ReloadCounter();							 
	AT_ECHO = &AT_ECHO_SAVE[0];	 
	CSQ_timeout = 5000;
	if(!AT_response("\r\n+CGATT: 1\r\n\r\nOK\r\n"))
	{
		gprs_en = 1; 
		CSQ_timeout = 0;  
		gprs_mux = 1;                        
	}
	else
	{
		delay_ms(100);
		DMA_Cmd(DMA1_Channel5, DISABLE); 
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 			
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		Send_AT_commands("AT+CGATT?\r\n");	
		AT_ECHO = &AT_ECHO_SAVE[0];	
		IWDG_ReloadCounter();
		delay_ms(1000);	
		IWDG_ReloadCounter();
		CSQ_timeout = 5000;
		if(!AT_response("\r\n+CGATT: 0\r\n\r\nOK\r\n"))
		{
			gprs_en = 0; 
			CSQ_timeout = 0;  
			gprs_mux = 1;               
		}  
	}
	delay_ms(100);
	DMA_Cmd(DMA1_Channel5, DISABLE); 
	IWDG_ReloadCounter();

	return gprs_mux;						 
}
/***************************************************************/
char sync_RTC(void)
{
	static unsigned char z;
	volatile unsigned char result_finalized = 0;
	
	IWDG_ReloadCounter();
	delay_ms(100);
	IWDG_ReloadCounter();

	AT_ack = 0;
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100);
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	IWDG_ReloadCounter();
	delay_ms(10);
	IWDG_ReloadCounter();
	IWDG_ReloadCounter();
	AT_ECHO = &AT_ECHO_SAVE[0];
	Send_AT_commands("AT+CLTS=1\r\n");                      
	IWDG_ReloadCounter();
	delay_ms(1000);
	IWDG_ReloadCounter();
	AT_ECHO = &AT_ECHO_SAVE[0];
	CSQ_timeout = 3000;
	if(!AT_response("\r\nOK\r\n"))
	{
		Enable_RTC = 1;
	}
	IWDG_ReloadCounter();
	delay_ms(2000);
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE);
	IWDG_ReloadCounter();

	if(Enable_RTC==1)
	{
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();

		AT_ack = 0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+CCLK?\r\n");                      
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 3000;
		if(!AT_response("\r\n+CCLK: "))
		{
			RTC_without_update = 1;
		}
		IWDG_ReloadCounter();
		delay_ms(10);
		IWDG_ReloadCounter();
		delay_ms(3000);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
	}

	if(RTC_without_update==1)
	{
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();

		AT_ack = 0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+CFUN=0\r\n");                      
		IWDG_ReloadCounter();
		delay_ms(2000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 5000;
		if(!AT_response("\r\n+CPIN: NOT READY\r\n\r\nOK\r\n"))
		{
			Disable_RF= 1;
		}
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
	}	

	if(Disable_RF==1)
	{
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();

		AT_ack = 0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+CFUN=1\r\n");   
		IWDG_ReloadCounter();
		delay_ms(2000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 5000;
		MESSAGE_ON
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		MESSAGE_OFF

		IWDG_ReloadCounter();
		delay_ms(5000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		delay_ms(5000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		delay_ms(5000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		delay_ms(5000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		delay_ms(5000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		delay_ms(5000);
		IWDG_ReloadCounter();

		DMA_Cmd(DMA1_Channel5, DISABLE);
	}				

	DMA_Cmd(DMA1_Channel5, DISABLE);
	delay_ms(10);
	IWDG_ReloadCounter();
	AT_ack = 0;
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100);
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	IWDG_ReloadCounter();

	IWDG_ReloadCounter();
	Send_AT_commands("AT+CPIN?\r\n");
	IWDG_ReloadCounter();
	delay_ms(1000);
	IWDG_ReloadCounter();
	CSQ_timeout = 3000;
	IWDG_ReloadCounter();
	AT_ECHO = &AT_ECHO_SAVE[0];
	switch(AT_response("\r\n+CPIN: READY\r\n\r\nOK\r\n"))        
	{
		case 0x00:
			Resetting_RF = 1;          
			break;
		case 0x01:
			Resetting_RF = 0;
			break;
		default:
			break;
	}
	CSQ_timeout = 0;
	IWDG_ReloadCounter();
	
	AT_ack = 0;
	delay_ms(100);
	DMA_Cmd(DMA1_Channel5, DISABLE);
	IWDG_ReloadCounter();
	memset(Rx1Buffer,'\0',400);
	USART_GetFlagStatus(USART1, USART_FLAG_ORE);
	USART_ReceiveData(USART1);
	memset(Rx1Buffer,'\0',400); 
	memset(AT_ECHO_SAVE,'\0',100); 		 
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	delay_ms(10); 
	
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(4000);
	IWDG_ReloadCounter();
	delay_ms(10);
	IWDG_ReloadCounter();
	
	if(Resetting_RF==1)
	{
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();

		AT_ack = 0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100);
		AT_BUFFER = &Rx1Buffer[0];
		AT_ECHO = &AT_ECHO_SAVE[0];
		start_DMA_access1(400);
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		Send_AT_commands("AT+CCLK?\r\n");                      
		IWDG_ReloadCounter();
		delay_ms(1000);
		IWDG_ReloadCounter();
		AT_ECHO = &AT_ECHO_SAVE[0];
		CSQ_timeout = 3000;
		if(!AT_response("\r\n+CCLK: "))
		{
			RTC_value = &AT_BUFFER[0];
			while(*RTC_value++ != '"');
			z=0;
			memset(Sync.Year,'\0',2);
			while(*RTC_value != '/')
			{
				Sync.Year[z++]=*RTC_value;
				*RTC_value++;  
			}
			*RTC_value++; 
			z=0;
			memset(Sync.Month,'\0',2);
			while(*RTC_value != '/')
			{
				Sync.Month[z++]=*RTC_value;
				*RTC_value++;  
			}
			*RTC_value++; 
			z=0;
			memset(Sync.Date,'\0',2);
			while(*RTC_value != ',')
			{
				Sync.Date[z++]=*RTC_value;
				*RTC_value++;  
			}
			*RTC_value++; 
			z=0;
			memset(Sync.Hour,'\0',2);
			while(*RTC_value != ':')
			{
				Sync.Hour[z++]=*RTC_value;
				*RTC_value++;  
			}
			time.Hour = ASCII_TO_BCD(Sync.Hour);
			*RTC_value++; 
			z=0;
			memset(Sync.Minute,'\0',2);
			while(*RTC_value != ':')
			{
				Sync.Minute[z++]=*RTC_value;
				*RTC_value++;  
			}
			time.Minute = ASCII_TO_BCD(Sync.Minute);
			*RTC_value++; 
			z=0;
			memset(Sync.Second,'\0',2);
			while(*RTC_value != '+')
			{
				Sync.Second[z++]=*RTC_value;
				*RTC_value++;  
			}
			time.Second = ASCII_TO_BCD(Sync.Second);
			result_finalized=1;			
		}
		IWDG_ReloadCounter();
		delay_ms(10);
		IWDG_ReloadCounter();
		delay_ms(100);
		memset(Rx1Buffer,'\0',400);
		AT_BUFFER = &Rx1Buffer[0];
		start_DMA_access1(50);
	}	
	return result_finalized;
}
/***************************************************************/
void bcd2ASCII_RTIME(struct networktime *t,struct Time4NetworkSync *dmy,char *data,char *id)
{
	unsigned char temp;
 	
	*data++ = 0x52;
	*data++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*data++ = *id;
		*id++;
	}
	*data++ = 0x2F;   // send /
	*data++ = dmy->Date[0];
	*data++ = dmy->Date[1];		
	*data++ = dmy->Month[0];
	*data++ = dmy->Month[1];
	*data++ = dmy->Year[0];
	*data++ = dmy->Year[1];

	temp = t->Hour;
	temp = temp&0x7F; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = t->Minute; 
	*data++ = 0x30 + (temp >> 4);
	*data++ = 0x30 + (temp & 0x0F); 

	temp = t->Second; 
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
/******************************************/
void Send_SMS(char *Ph_number)
{
	clear_ATbuffer();
	dial_number(Ph_number);
	AT_ECHO = &AT_ECHO_SAVE[0];
	CSQ_timeout = 1500;
	delay_ms(1000); //ADD //DOUBT!!
	if(!AT_response("\r\n> "))
	{
		CSQ_timeout = 0;
		datasend_GSM();
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
	}
	/********* Reboot system if > not received **************/
	else
	{
		Reset_Error[0]=1; //sukoonerror
		strcat_flash();
		write_in_flash(flash_memory);
		//while(1);		//DOUBT2 
		NVIC_SystemReset(); //ADD
  }
}
/*******************************************/
void haultCall(void)
{
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100); 			
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	Send_AT_commands("ATH\r\n");  	
	AT_ECHO = &AT_ECHO_SAVE[0];	 
	delay_ms(1000);CSQ_timeout = 1000;
	if(!AT_response("\r\nOK\r\n"));
	delay_ms(100);   
	CSQ_timeout = 0;
}
/********************************************/
void strcat_flash(void)
{
	memset(flash_memory,'\0',500);
	strcp = flash_memory;
	strcp1 = Ph.No1;
	strcpy(strcp,strcp1);	 
	strcat(flash_memory,separator);
	strcat(flash_memory,Ph.No2);
	strcat(flash_memory,separator);
	strcat(flash_memory,SiteName);
	strcat(flash_memory,separator);
	strcat(flash_memory,SiteId);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[0].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[1].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[2].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[3].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[4].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[5].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[6].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[7].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[8].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[9].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[10].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,daily[11].AutoReport);
	strcat(flash_memory,separator);
	strcat(flash_memory,Alarm_config);
	for(i=0;i<16;i++) //EVENTSS
	{
		strcat(flash_memory,separator);
		strcat(flash_memory,Event[i].Time);
	}
	strcat(flash_memory,separator);
	strcat(flash_memory,Gparam.ServerIP);
	strcat(flash_memory,separator);
	strcat(flash_memory,Gparam.ServerPort);
	strcat(flash_memory,separator);
	strcat(flash_memory,Gpset.APN);	
	strcat(flash_memory,separator);
	strcat(flash_memory,converter.xcon);
	strcat(flash_memory,separator);
	strcat(flash_memory,converter.No_of_Alerts);
	strcat(flash_memory,separator);
	strcat(flash_memory,converter.LogTimeGprs);	
	strcat(flash_memory,separator);
	strcat(flash_memory,converter.RTU_OP_MODE);	
  strcat(flash_memory,separator);
  	strcat(flash_memory,Reset_Error);  //sukoonerror
	  strcat(flash_memory,separator);
	strcat(flash_memory,KWH_Factor); //ADD
	strcat(flash_memory,separator);
	CUMKWH_FLASH[0] = CurrentPosition.bytes[0];
	strcat(flash_memory,CUMKWH_FLASH);  //SUKI
	strcat(flash_memory,separator);
	CUMKWH_FLASH[0] = CurrentPosition.bytes[1];
	strcat(flash_memory,CUMKWH_FLASH);  //SUKI
	strcat(flash_memory,separator);
	CUMKWH_FLASH[0] = CurrentPosition.bytes[2];
	strcat(flash_memory,CUMKWH_FLASH);  //SUKI
	strcat(flash_memory,separator);
	CUMKWH_FLASH[0] = CurrentPosition.bytes[3];
	strcat(flash_memory,CUMKWH_FLASH);  //SUKI
	
	
}
/**************************************************/
char check_GSM_MODE()
{               
	Switch2CommandMode();
	delay_ms(10);
	DMA_Cmd(DMA1_Channel5, DISABLE); 
	IWDG_ReloadCounter();
	memset(Rx1Buffer,'\0',400);    // clear the array before next transmit starts
	start_DMA_access1(400);
	USART_SendData(USART1,0x1B);   // transmiison for ESC // Used for debug purpose.

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);

	IWDG_ReloadCounter();                
	delay_ms(500);   
	IWDG_ReloadCounter();
	DMA_Cmd(DMA1_Channel5, DISABLE);
	memset(Rx1Buffer,'\0',400);
	memset(AT_ECHO_SAVE,'\0',100); 			
	AT_BUFFER = &Rx1Buffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_access1(400);
	Send_AT_commands("AT+COPS?\r\n");	
	AT_ECHO = &AT_ECHO_SAVE[0];	
	delay_ms(1000);
	CSQ_timeout = 1000; 
	if(!AT_response("\r\n+COPS: 0\r\n"))
	{
		CSQ_timeout = 0; 
		IWDG_ReloadCounter();
		delay_ms(300); 
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 		
		DMA_Cmd(DMA1_Channel5, DISABLE); 
		SMS_FLAG = 1;
		Switch2DataMode();
	}
	else 
	{
		CSQ_timeout = 0; 
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
		memset(Rx1Buffer,'\0',400);
		memset(AT_ECHO_SAVE,'\0',100); 		
		DMA_Cmd(DMA1_Channel5, DISABLE); 
		SMS_FLAG = 0;
		Switch2DataMode();
	}
	IWDG_ReloadCounter();
	return SMS_FLAG;
}
/************************************************************/
void Call_CSQ_Module(void)
{
	if(RTU_MODE == GPRS_MODE)
	Switch2CommandMode();
	EXT_INT_FLAG = 1;
	Check_GSM_Network();
	if(RTU_MODE == GPRS_MODE)
	{
		Switch2DataMode();
		EXT_INT_FLAG = 0;
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
	}
}
/*********************************************************/
void Check_GPRS_Buffer(void)
{
	if(buff.IP0)
	{				
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP0Buff[0],1);
		buff.IP0=0;
		memset(IP0Buff,'\0',400);
	}
	else if(buff.IP1)
	{				
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP1Buff[0],1);
		buff.IP1=0;
		memset(IP1Buff,'\0',400);
	}
	else if(buff.IP2)
	{				
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP2Buff[0],1);
		buff.IP2=0;
		memset(IP2Buff,'\0',400);
	}
	else if(buff.IP3)
	{				
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP3Buff[0],1);
		buff.IP3=0;
		memset(IP3Buff,'\0',400);
	}	
	else if(buff.IP4)
	{		
		IWDG_ReloadCounter();							
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP4Buff[0],1);
		buff.IP4=0;
		memset(IP4Buff,'\0',400);
	}
	else if(buff.IP5)
	{				
		IWDG_ReloadCounter();
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP5Buff[0],1);
		buff.IP5=0;
		memset(IP5Buff,'\0',400);
	}
	else if(buff.IP6)
	{		
		IWDG_ReloadCounter();				
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP6Buff[0],1);
		buff.IP6=0;
		memset(IP6Buff,'\0',400);
	}	
	else if(buff.IP7)
	{		
		IWDG_ReloadCounter();				
		delay_ms(100);
		IWDG_ReloadCounter();
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		SMS_control(&IP7Buff[0],1);
		buff.IP7=0;
		memset(IP7Buff,'\0',400);
	}	
 }
/*********************************************************/
void Check_SMS_Buffer(void)
{
	if(buff.Rx0)                       
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx0Buff[0],0));
		IWDG_ReloadCounter();
		delay_ms(500); 
		IWDG_ReloadCounter();					
		buff.Rx0=0;
	}
	else if(buff.Rx1)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx1Buff[0],0));
		buff.Rx1=0;
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
	}
	else if(buff.Rx2)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx2Buff[0],0));
		buff.Rx2=0;
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
	} 
	else if(buff.Rx3)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx3Buff[0],0));
		buff.Rx3=0;
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
	}
	else if(buff.Rx4)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx4Buff[0],0));
		buff.Rx4=0;
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
	}
	else if(buff.Rx5)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx5Buff[0],0));
		buff.Rx5=0;
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
	}	   
	else if(buff.Rx6)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);		
		IWDG_ReloadCounter();
		if(SMS_control(&Rx6Buff[0],0));
		buff.Rx6=0;
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();
	} 
	else if(buff.Rx7)
	{
		EXT_INT_FLAG = 1;
		IWDG_ReloadCounter();
		delay_ms(1500);
		IWDG_ReloadCounter();
		if(SMS_control(&Rx7Buff[0],0)); 
		buff.Rx7=0;
		delay_ms(500);
		IWDG_ReloadCounter();
		delay_ms(500);
		IWDG_ReloadCounter();									
	}	 
}
/*******************************************************/
void Check_GSM_Network()
{
	DMA_Cmd(DMA1_Channel5, DISABLE); 		
	memset(CSQBuffer,'\0',100); 
	memset(AT_ECHO_SAVE,'\0',100); 		 
	AT_BUFFER = &CSQBuffer[0];
	AT_ECHO = &AT_ECHO_SAVE[0];
	start_DMA_accessCSQ(100);
	Send_AT_commands("AT+CSQ\r\n");
	delay_ms(500);
	AT_ECHO = &AT_ECHO_SAVE[0];
	CSQ_timeout = 1000;	 
	if(!AT_response("\r\n+CSQ: "))
	{
		CSQ_timeout = 0;
		Signal_Strength = AT_response_CSQ();
	}
	delay_ms(50);	
	CSQ_timeout = 0;
}	
/********************************************************/
//unsigned char Date=0, Month=0, Year=0; //ADD6
int main(void)
{
	unsigned char no;
	unsigned char ab;
	unsigned int p=0;
	static unsigned char i,mssg_clarify;
	char Get_Result = 0;
  /* At this stage the microcontroller clock setting is already configured,
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f10x_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f10x.c file */
//Date = ((0x32-48)*10) + (0x38-48); //ADD7
	  pntr = &Server_No_New[0]; //ADD8

	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	for(p=0;p<=999999;p++);
	
	// GPIOD Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOB|
	RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE); 

	// Allow access to BKP Domain
	PWR->CR=PWR->CR|0x00000100;  // Allow acess to backup register
   
	RTC_Configuration();
	SysTick_Configuration();
	USART1_initialisation();
	initialisationIOpins();
	Timer3_Initialize(); 

	Counter = 0;
	LED_OFF
	MESSAGE_OFF
	DTR_PULLDOWN
	CTS_LOW
	 
	memset(flash_memory,'\0',500);
	strcp = flash_memory;
	if(!read_4m_flash(flash_memory))
	{
		Set_Default();
		strcat_flash();
		write_in_flash(flash_memory);
	}
	else
	{
		strcp = flash_memory;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			Ph.No1[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			Ph.No2[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			SiteName[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			SiteId[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[0].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[1].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[2].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[3].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[4].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[5].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[6].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[7].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[8].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[9].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0'	)
		{
			daily[10].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			daily[11].AutoReport[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			Alarm_config[ab++]=*strcp;
			*strcp++;
		}
		configurable = ASCII_2_decimal(Alarm_config);
		*strcp++;
		for(i=0;i<16;i++) //EVENTSS
		{
			ab = 0;
			while(*strcp != 0x3B && *strcp != '\0')
			{
				Event[i].Time[ab++]=*strcp;
				*strcp++;
			}
			*strcp++;
			Event[i].time_event = ASCII_2_decimal(Event[i].Time);
		}
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			Gparam.ServerIP[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			Gparam.ServerPort[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			Gpset.APN[ab++]=*strcp;
			*strcp++;
		}
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			converter.xcon[ab++]=*strcp;
			*strcp++;
		}				
		converter.xcoonh = converter.xcon[0];			
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			converter.No_of_Alerts[ab++]=*strcp;
			*strcp++;
		}			
		converter.NoAlerts = converter.No_of_Alerts[0];						
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			converter.LogTimeGprs[ab++]=*strcp;
			*strcp++;
		}	
		if(converter.LogTimeGprs[0]  == 0)
			converter.LogTimeGprs[0] = 30;
		converter.TimeGprs = converter.LogTimeGprs[0];	
		*strcp++;
		ab = 0;
		while(*strcp != 0x3B && *strcp != '\0')
		{
			converter.RTU_OP_MODE[ab++]=*strcp;
			*strcp++;
		}							
		converter.OPER_MODE = converter.RTU_OP_MODE[0];	
		*strcp++;
while(*strcp != 0x3B && *strcp != '\0')   //sukoonerror
		{
		Reset_Error[0]=*strcp;
			*strcp++;
		}	
		*strcp++;		
while(*strcp != 0x3B && *strcp != '\0')    //ADD
		{
			KWH_Factor[0]=*strcp;
			*strcp++;
		}	
		*strcp++;
while(*strcp != 0x3B && *strcp != '\0')    //SUKI
		{
			CurrentPosition.bytes[0]=*strcp;
		*strcp++;
		}	
			*strcp++;	
while(*strcp != 0x3B && *strcp != '\0')    //SUKI
		{
		CurrentPosition.bytes[1]=*strcp;
		*strcp++;
		}	
			*strcp++;	
while(*strcp != 0x3B && *strcp != '\0')    //SUKI
		{
		CurrentPosition.bytes[2]=*strcp;
		*strcp++;
		}	
			*strcp++;	
while(*strcp != 0x3B && *strcp != '\0')    //SUKI
		{
		CurrentPosition.bytes[3]=*strcp;
		*strcp++;
		}			
	}
		if(Ph.No1[0]=='\0')
	{ 
     for(i=0; i<10; i++)
		{
			Ph.No1[i]=*pntr;
			pntr++;
		}
   }
   
	DMA_Cmd(DMA1_Channel5, DISABLE);
	if(SiteId[0] == '\0')
	{
		RTU_State = Not_Configured;
	}
	else
		RTU_State = Configured;
	
/*************************************************************/
/***** Watch-dog Timer*******/
	
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
		Counter Reload Value = 8000ms/IWDG counter clock period
												 = 8000ms / (LSI/32)
												 = 8s / (LsiFreq/32)
												 = LsiFreq/4
	*/
	IWDG_SetReload(LsiFreq/4);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
 IWDG_Enable();  // Disable in debug mode
	
/*************************************************************/

	RTCinterrupt();
	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/*Enable the RTC Second */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	RTC_on();

	SIM800_ON
	IWDG_ReloadCounter();
	delay_ms(3000);
	IWDG_ReloadCounter();
	SIM800_OFF
	delay_ms(1000);
	PWRKey_OFF
	MESSAGE_OFF
	IWDG_ReloadCounter();

	delay_ms(100);
	memset(Rx1Buffer,'\0',400);
	AT_BUFFER = &Rx1Buffer[0];
	start_DMA_access1(10);
	IWDG_ReloadCounter();
	delay_ms(1500);
	IWDG_ReloadCounter();

	Get_Result = GSM_Initialize();
	IWDG_ReloadCounter();	

  RI_interrupt_modeminit();
  USART2_initialisation();
	RTC_Check_Count=1;
	if(RTC_Check_Count == 1)
	{
		for(test=0;test<3;test++)
		{
			debug_RTC = sync_RTC();
			if(debug_RTC == 1)
			break;
		}
		if(debug_RTC == 0)
		{
			NVIC_SystemReset();
		}
		getRTCfromnetwork(&GSM,&time);
		NewRTCvalue=Time_Regulate(&GSM);
		NewRTCvalue_Lo=NewRTCvalue&0xFFFF;
		NewRTCvalue_Hi=NewAlarm>>16;
		RTC_WaitForLastTask();
		RTC_EnterConfigMode();
		Time_Adjust(NewRTCvalue);  // Write to new RTC register
		RTC_ExitConfigMode();      // Exit configuration mode
		RTC_WaitForLastTask();     // Wait for last operation to end
		RTC_Check_Count = 0;
		debug_RTC = 0;
	}
							
	memset(Rx1Buffer,'\0',400);
	AT_BUFFER = &Rx1Buffer[0];
	start_DMA_access1(50);
	delay_ms(5000);
	IWDG_ReloadCounter();

	#ifdef _GPRS_MODE_
	if((Gpset.APN[0] != '\0') && (Get_Result == OK) && (converter.OPER_MODE <= GPRS_ONLY))
	{
		if(Connection() == ERR_OK)
		{
			IWDG_ReloadCounter();
			memset(ARRAY_TRANS,'\0',200);
			reply_IPadd(ARRAY_TRANS,SiteId,IPAdd,GPRS_MODE);
			Send_Data_GPRS();
			IWDG_ReloadCounter();
			RTU_MODE = GPRS_MODE;
			Counter = SET;
		}
		else
		{
			RTU_MODE = GSM_MODE;
			Counter = SET;
		}
	}
	else
	{
		Counter = SET;
		RTU_MODE = GSM_MODE; 
	}
	#endif

	memset(Rx1Buffer,'\0',400);
	AT_BUFFER = &Rx1Buffer[0];
	start_DMA_access1(50);
	delay_ms(5000);
				
	#ifndef _GPRS_MODE_
		#ifdef _GSM_MODE_
			Counter = SET;
			RTU_MODE = GSM_MODE; 
		#endif
	#endif


	IWDG_ReloadCounter();

	while(Counter != SET);
	LED_ON               

	IWDG_ReloadCounter();

	PWR->CR=PWR->CR|0x00000100;

/************** Read XCON for Alerts  *******************/

	for(i=0;i<24;i++)
	{
		Alert[i]=0xffffffff;
	}
	if(converter.xcoonh==2)
	{
/************** Read No.of Alerts  *******************/
		for(i=0;i<converter.NoAlerts;i++)
		{
			autoTIMEset(daily[i].AutoReport,&AutoGen);
			Alert[i]=Time_Regulate(&AutoGen);
		}
		sort(Alert,converter.NoAlerts);
	}
	else if(converter.xcoonh==1)
	{
		for(i=0;i<24;i++)
		{
			Alert[i]=i*3600;
		}
	}
	set_alert(Alert,converter.xcoonh,converter.NoAlerts);

	if(RTU_MODE == GPRS_MODE)
	{
		EXT_INT_FLAG = 0; 
		DMA_Cmd(DMA1_Channel5, DISABLE);
// 		memset(Rx1Buffer,'\0',400);              
// 		start_DMA_access1(7);
		IWDG_ReloadCounter();
	}
	else
		EXT_INT_FLAG = 1;

	GPRS_retry = RESET;		
	CSQ_time_count = RESET; 
	Data_Log_Time = RESET;
	RTC_sec_count = RESET; 
	Data_Log_Time = RESET;	
	Send_Status = RESET; 
	Sd_Write = RESET;
	reset_status=1;
		
	IWDG_ReloadCounter();
	TIM_Cmd(TIM3, ENABLE);
	start_counter=1;
Send_Msg_To_Server_Once = 1; //12april Can be disabled if events and alarms are working perfectly  
	
	while(1)
	{
		IWDG_ReloadCounter();

		#ifdef _GPRS_MODE_
		/*********** Live status 5 min  *************/
		Check_Task_LiveStatus();
		IWDG_ReloadCounter();
		#endif

		#ifdef _GSM_MODE_
		/*********** CSQ module   ****************/
		Check_Task_CSQ();
		IWDG_ReloadCounter();
		#endif

		delay_ms(1000);         
		IWDG_ReloadCounter();

		/************ Check DCD Pin for GPRS or GSM *****************/
		Check_RTU_Mode_DCD();
		IWDG_ReloadCounter();


		
		if(prev_conn)
		{
			MESSAGE_ON
			if(reset_status == 1)
			{
				message_status = 5;
				reset_status = 0;
			}
			else
				message_status = 2;
			if(RTC_GetCounter() > 0x00000708) //Greater than 12:30 AM
			{Send_Alarm2Server();}
			prev_conn = 0;
			IWDG_ReloadCounter();
			delay_ms(1000); 
			IWDG_ReloadCounter();
			MESSAGE_OFF
		if(Reset_Error[0] == 1)
			{
			Reset_Error[0] = 0; //sukoonerror
             strcat_flash();
			 write_in_flash(flash_memory);
			}
		}
			if(prev_conn_KWH)
		{
			MESSAGE_ON
			if(reset_status == 1)
			{
				message_status = 5;
				reset_status = 0;
			}
			else
				message_status = 2;
			if(RTC_GetCounter() > 0x00000708) //Greater than 12:30 AM
			{Send_Alarm2Server_2();}
			prev_conn_KWH = 0;
			IWDG_ReloadCounter();
			delay_ms(1000); 
			IWDG_ReloadCounter();
			MESSAGE_OFF
			if(Reset_Error[0] == 1)
			{
			Reset_Error[0] = 0;
			strcat_flash();
			write_in_flash(flash_memory);	    
			}	
		}
	
		IWDG_ReloadCounter();

		/*********** Check EXT_INT_FLAG for CALL or SMS in GPRS Mode   **************************/
		Check_ExtIntFlag();

		#ifdef _GPRS_MODE_

		/*********** Check GPRS data log duration   ****************************************/
		Check_Task_GPRSDataLog();
		IWDG_ReloadCounter                                                                                                                                                                                                                                                                                                                                                                                                                                                    ();	

		/************* GPRS connection Retry hard fault 5 min***************/
		Check_Task_GPRSConnRetry();  

		/*******************  Check buffers for incoming packet   *******************/
		Check_GPRS_Buffer();

		#endif

		/*******************  Check buffers for incoming SMS OR CALL   *******************/
		Check_SMS_Buffer();

		if(validSMS != 0)
		{
			IWDG_ReloadCounter();

			if(RTU_MODE==GPRS_MODE && ConnMode == DATM)
				Switch2CommandMode();
			EXT_INT_FLAG = 1;
			DMA_Cmd(DMA1_Channel5, DISABLE);
			IWDG_ReloadCounter();
			delay_ms(5);
			memset(Rx1Buffer,'\0',400);   
			start_DMA_access1(400);
			USART_SendData(USART1,0x1B);   
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
			IWDG_ReloadCounter();
			delay_ms(500);
			IWDG_ReloadCounter();
			if(validSMS == 3)
			{
				ShutConn();
				RTU_MODE = GSM_MODE;
				validSMS = 0;
			}
			else
			{
				IWDG_ReloadCounter();
				validSMS = 0;
				delay_ms(10);
				IWDG_ReloadCounter();
				if(!dial_num_test)
				{ 
					LED_INDICATOR=1;
				}
				else if(converter.OPER_MODE == BLOCK_COMM)
					Block_LED=1;
				else
				{
					dial_num_test=0;
					MESSAGE_ON
				}
				for(mssg_clarify=0;mssg_clarify<10;mssg_clarify++)
				{
					debug =	received_msgtype(Msg[mssg_clarify].MESSAGE);
					delay_ms(500);
					IWDG_ReloadCounter();
					switch(debug)
					{
						case 24 :// XGPDT
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();

											Alert[0]=0xffffffff;
											Alert[1]=0xffffffff;
											Alert[2]=0xffffffff;
											Alert[3]=0xffffffff;
											Alert[4]=0xffffffff;
											Alert[5]=0xffffffff;
											Alert[6]=0xffffffff;
											Alert[7]=0xffffffff;
											Alert[8]=0xffffffff;
											Alert[9]=0xffffffff;
											Alert[10]=0xffffffff;
											Alert[11]=0xffffffff;

											strcat_flash();
											write_in_flash(flash_memory);
											IWDG_ReloadCounter();

											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											read_4m_flash(flash_memory);
											strcp = flash_memory;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												Ph.No1[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											while(*strcp != 0x3B)   
											*strcp++;
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												SiteName[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												SiteId[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[0].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[1].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[2].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[3].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[4].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[5].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[6].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[7].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[8].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[9].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[10].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												daily[11].AutoReport[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											while(*strcp != 0x3B)         
											*strcp++;
											*strcp++;
											for(i=0;i<16;i++) //EVENTSS
											{
												ab = 0;
												while(*strcp != 0x3B)
												*strcp++;
												*strcp++;
											}
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												Gparam.ServerIP[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												Gparam.ServerPort[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												Gpset.APN[ab++]=*strcp;
												*strcp++;
											}
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												converter.xcon[ab++]=*strcp;
												*strcp++;
											}			
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												converter.No_of_Alerts[ab++]=*strcp;
												*strcp++;
											}						
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												converter.LogTimeGprs[ab++]=*strcp;
												*strcp++;
											}							
											*strcp++;
											ab = 0;
											while(*strcp != 0x3B && *strcp != '\0')
											{
												converter.RTU_OP_MODE[ab++]=*strcp;
												*strcp++;
											}							
											*strcp++;
											tag=0;
											while(tag < converter.NoAlerts)
											{
												autoTIMEset(daily[tag].AutoReport,&AutoGen);
												Alert[tag]=Time_Regulate(&AutoGen);
												tag++;
											}
											sort(Alert,converter.NoAlerts);
											set_alert(Alert,converter.xcoonh,converter.NoAlerts);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											memset(ARRAY_TRANS,'\0',200); 
											ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,3,RTU_MODE,Signal_Strength,converter.OPER_MODE);
											Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											delay_ms(3000);
											IWDG_ReloadCounter();
											ShutConn();
											IWDG_ReloadCounter();

											if((converter.OPER_MODE<=GPRS_ONLY)&&(Gpset.APN[0] != '\0') && (Gparam.ServerIP[0] != '\0') && (Gparam.ServerPort[0] != '\0'))
											{
												if(Connection() == ERR_OK)
												{
													RTC_sec_count = 0; Data_Log_Time = 0;
													memset(ARRAY_TRANS,'\0',200);
													reply_IPadd(ARRAY_TRANS,SiteId,IPAdd,GPRS_MODE);
													Send_Data_GPRS();
													IWDG_ReloadCounter();
													delay_ms(200);
													IWDG_ReloadCounter();
													RTU_MODE = GPRS_MODE;
													Switch2CommandMode();
												}
												else
												{
													RTU_MODE = GSM_MODE;
												}
											}
											break;

						case 3 :// For CSTAT
										  DMA_Cmd(DMA1_Channel5, DISABLE);
										  delay_ms(5);
										  if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
										  {
											  memset(ARRAY_TRANS,'\0',200);   
											  ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,0,RTU_MODE,Signal_Strength,converter.OPER_MODE);
											  Switch2DataMode();
											  Send_Data_GPRS();    
											  Switch2CommandMode();
											  IWDG_ReloadCounter();
											  delay_ms(3000);
										  }
										  else
										  {
											  memset(ARRAY_TRANS,'\0',200);    
											  ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,0,RTU_MODE,Signal_Strength,converter.OPER_MODE);
											  Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											  IWDG_ReloadCounter();
											  delay_ms(3000);
												IWDG_ReloadCounter(); //ADD
										  }
										  break;
										 
						case 1://STATS
										  DMA_Cmd(DMA1_Channel5, DISABLE);
										  delay_ms(5);
										  IWDG_ReloadCounter();
										  memset(ARRAY_TRANS,'\0',200);    
										  //ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,SiteName);
						          ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,0); //ADD
										  if(Msg[mssg_clarify].DIAL_NUM[0] != '\0')
										  {
											  Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
										  }
										  break;

						case 4://XTIME
											delay_ms(5);
											IWDG_ReloadCounter();
							
											ASCII_hex_BCD_conversion(&SP1,TFRAME);
											Initialize_Date(&Sync,TFRAME);
											Initialize_Time(&time,&SP1);
											ASCII_2_HEX(TFRAME,&verify);
							
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter(); 

											getRTCfromnetwork(&GSM,&time);
											NewRTCvalue=Time_Regulate(&GSM);
											RTC_on();
											Time_Adjust(NewRTCvalue); 
											RTC_ExitConfigMode();     
											RTC_WaitForLastTask();    

											if(NewRTCvalue - 10 < RTC_GetCounter() < NewRTCvalue + 10)
											{
												result_final1 = 1;
											}
										
											set_alert(Alert,converter.xcoonh,converter.NoAlerts);            
											memset(ARRAY_TRANS,'\0',200);   
											bcd2ASCII_RTIME(&time,&Sync,ARRAY_TRANS,SiteId);
											Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											break;
										
						case 12://RESET
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											memset(ARRAY_TRANS,'\0',200);
											reply_reset(ARRAY_TRANS,SiteId);
											Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											NVIC_SystemReset();
		
						case 2://NUMBR
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											memset(ARRAY_TRANS,'\0',200);    
											send_MobileNo(&Ph,ARRAY_TRANS);
											if(Msg[mssg_clarify].DIAL_NUM[0] != '\0')
											{
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
										
						case 5://XSITE
											strcat_flash();
											write_in_flash(flash_memory);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_extract_phoneNo(SiteId,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_extract_phoneNo(SiteId,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
										
						case 6://XPH01
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											phone_numbers_map();
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);  
												reply_MobileNo(&Ph,ARRAY_TRANS,SiteId,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_MobileNo(&Ph,ARRAY_TRANS,SiteId,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											strcat_flash();
											write_in_flash(flash_memory);
											break;
						
						case 7://DPH01
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);   
												reply_MobileNo(&Ph,ARRAY_TRANS,SiteId,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);   
												reply_MobileNo(&Ph,ARRAY_TRANS,SiteId,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											strcat_flash();
											write_in_flash(flash_memory);
											break;

						case 13://XCONP
											Alert[0]=0xffffffff;
											Alert[1]=0xffffffff;
											Alert[2]=0xffffffff;
											Alert[3]=0xffffffff;
											Alert[4]=0xffffffff;
											Alert[5]=0xffffffff;
											Alert[6]=0xffffffff;
											Alert[7]=0xffffffff;
											Alert[8]=0xffffffff;
											Alert[9]=0xffffffff;
											Alert[10]=0xffffffff;
											Alert[11]=0xffffffff;
											tag=0;
											while(tag < converter.NoAlerts)
											{
												autoTIMEset(daily[tag].AutoReport,&AutoGen);
												Alert[tag]=Time_Regulate(&AutoGen);
												tag++;
											}
											strcat_flash();
											write_in_flash(flash_memory);
											sort(Alert,converter.NoAlerts);
											set_alert(Alert,converter.xcoonh,converter.NoAlerts);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xconp(SiteId,converter.xcoonh,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xconp(SiteId,converter.xcoonh,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
											
						case 14://XNAME
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);   
												reply_xname(SiteId,SiteName , ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);  
												reply_xname(SiteId,SiteName , ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											strcat_flash();
											write_in_flash(flash_memory);
											break;
						
						case 18://IPADD
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_ipadd(ARRAY_TRANS,SiteId,IPAdd,local_port,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);   
												reply_ipadd(ARRAY_TRANS,SiteId,IPAdd,local_port,GPRS_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 22://XCONH
											memset( converter.No_of_Alerts,'\0',2);   
											for(no=0;no<24;no++)
											{
												Alert[no]=no*3600;
											}
											converter.No_of_Alerts[0] = 24;
											converter.NoAlerts = 24;
											strcat_flash();
											write_in_flash(flash_memory);
											set_alert(Alert,converter.xcoonh,converter.NoAlerts);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_xconp(SiteId,converter.xcoonh,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_xconp(SiteId,converter.xcoonh,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
											
						case 23://XCNFG
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    // clear the array before next transmit starts
												reply_extract_phoneNo_config(SiteId,Alarm_config,ARRAY_TRANS,Event,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    // clear the array before next transmit starts
												reply_extract_phoneNo_config(SiteId,Alarm_config,ARRAY_TRANS,Event,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											for(i=0;i<16;i++) //EVENTSS
											{
												Event[i].independent_counter = 0;
											}
											strcat_flash();
											write_in_flash(flash_memory);
											for(i=0;i<16;i++) //EVENTSS
											{
												Event[i].count = 0;
												Event[i].independent_counter = 0;
											}
											break;

						case 25://XDFLT
											IWDG_ReloadCounter();
											ShutConn();
											delay_ms(5);
											IWDG_ReloadCounter();
											DMA_Cmd(DMA1_Channel5, DISABLE);
											RTU_MODE = GSM_MODE;
											Reset_flash();
											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
												if(Msg[mssg_clarify].DIAL_NUM[0] != '\0')
												{
													DMA_Cmd(DMA1_Channel5, DISABLE);
													delay_ms(5);
													memset(ARRAY_TRANS,'\0',200);    // clear the array before next transmit starts
													Reply_xdflt(ARRAY_TRANS,0,GSM_MODE);
													Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
													IWDG_ReloadCounter();
												}
											}
											else
											{
												if(Msg[mssg_clarify].DIAL_NUM[0] != '\0')
													{
													DMA_Cmd(DMA1_Channel5, DISABLE);
													delay_ms(5);
													memset(ARRAY_TRANS,'\0',200);    // clear the array before next transmit starts
													Reply_xdflt(ARRAY_TRANS,1,GSM_MODE);
													Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
													IWDG_ReloadCounter();
												}
											}
											break;

						case 26://XGLOG
											strcat_flash();
											write_in_flash(flash_memory);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_ltime(SiteId,&Gpset,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_ltime(SiteId,&Gpset,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											RTC_sec_count = 0; Data_Log_Time = 0;
											break;

						case 27://XMODE
											strcat_flash();
											write_in_flash(flash_memory);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xmode(SiteId,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xmode(SiteId,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											if(converter.OPER_MODE >= 2)
											{
												IWDG_ReloadCounter();
												ShutConn();
												IWDG_ReloadCounter();
											}
											else if(converter.OPER_MODE <=1)
												GPRS_retry = 1;
											break;

						case 28://XGSIP
											strcat_flash();
											write_in_flash(flash_memory);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgsip(SiteId,&Gparam,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgsip(SiteId,&Gparam,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											IWDG_ReloadCounter();
											delay_ms(100);
											IWDG_ReloadCounter();
											ShutConn();
											IWDG_ReloadCounter();
											GPRS_retry = 1;
											break;

						case 29://XGPRT
											strcat_flash();
											write_in_flash(flash_memory);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgprt(SiteId,&Gparam,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgprt(SiteId,&Gparam,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											IWDG_ReloadCounter();
											delay_ms(100);
											IWDG_ReloadCounter();
											ShutConn();
											IWDG_ReloadCounter();
											GPRS_retry = 1;
											break;

						case 30://XGAPN
											strcat_flash();
											write_in_flash(flash_memory);
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											IWDG_ReloadCounter();
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgapn(SiteId,&Gpset,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgapn(SiteId,&Gpset,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											IWDG_ReloadCounter();
											delay_ms(100);
											IWDG_ReloadCounter();
											ShutConn();
											IWDG_ReloadCounter();
											GPRS_retry = 1;
											break;					 

						case 31://RREAD
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
										
											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Ph.No1[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												while(*strcp != 0x3B)  
												*strcp++;
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteName[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteId[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[0].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[1].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[2].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[3].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[4].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[5].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[6].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[7].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[8].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[9].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[10].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[11].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												while(*strcp != 0x3B)      
												*strcp++;
												*strcp++;
												for(i=0;i<16;i++) //EVENTSS
												{
													ab = 0;
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerIP[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerPort[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gpset.APN[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.xcon[ab++]=*strcp;
													*strcp++;
												}			
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.No_of_Alerts[ab++]=*strcp;
													*strcp++;
												}						
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.LogTimeGprs[ab++]=*strcp;
													*strcp++;
												}							
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.RTU_OP_MODE[ab++]=*strcp;
													*strcp++;
												}							
												*strcp++;
											}
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    
												Send_ConfigAll(SiteName,SiteId,&Gparam,&Gpset,&Ph,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												Send_ConfigAll(SiteName,SiteId,&Gparam,&Gpset,&Ph,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 32://RNAME
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);

											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												ab = 0;
												while(*strcp != 0x3B)
												*strcp++;
												*strcp++;
												while(*strcp != 0x3B) 
												*strcp++;
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
												SiteName[ab++]=*strcp;              
												*strcp++;
												}
												*strcp++;
											}
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_xname(SiteId,SiteName,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_xname(SiteId,SiteName,ARRAY_TRANS,GPRS_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
										
						case 34://RCNFG
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);

											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												for(ab=0;ab<3;ab++)
												{
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteId[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												for(ab=0;ab<12;ab++)
												{
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Alarm_config[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												for(i=0;i<16;i++) //EVENTSS
												{
													ab = 0;
													while(*strcp != 0x3B && *strcp != '\0')
													{
														Event[i].Time[ab++]=*strcp;
														*strcp++;
													}
													*strcp++;
												}
											}
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_extract_phoneNo_config(SiteId,Alarm_config,ARRAY_TRANS,Event,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_extract_phoneNo_config(SiteId,Alarm_config,ARRAY_TRANS,Event,GPRS_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 33://RTIME
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_Rtime(ARRAY_TRANS,SiteId,&Sync,GPRS_MODE );
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_Rtime(ARRAY_TRANS,SiteId,&Sync,GSM_MODE );
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 35://RCONP
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);

											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												for(ab=0;ab<3;ab++)
												{
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteId[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[0].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[1].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[2].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[3].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[4].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[5].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[6].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[7].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[8].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[9].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[10].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[11].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
											}
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);   
												reply_rconp(SiteId,converter.xcoonh,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);    
												reply_rconp(SiteId,converter.xcoonh,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 36://RGLOG
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_ltime(SiteId,&Gpset,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_ltime(SiteId,&Gpset,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 37://RGSIP
											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Ph.No1[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												while(*strcp != 0x3B) 
												*strcp++;
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteName[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteId[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[0].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[1].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[2].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[3].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[4].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[5].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[6].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[7].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[8].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[9].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[10].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[11].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												while(*strcp != 0x3B)          
												*strcp++;
												*strcp++;
												for(i=0;i<16;i++) //EVENTSS
												{
													ab = 0;
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerIP[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerPort[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gpset.APN[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.xcon[ab++]=*strcp;
													*strcp++;
												}			
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.No_of_Alerts[ab++]=*strcp;
													*strcp++;
												}						
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.LogTimeGprs[ab++]=*strcp;
													*strcp++;
												}							
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.RTU_OP_MODE[ab++]=*strcp;
													*strcp++;
												}							
												*strcp++;
											}
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgsip(SiteId,&Gparam,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgsip(SiteId,&Gparam,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 38://RGPRT
											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Ph.No1[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												while(*strcp != 0x3B)   
												*strcp++;
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteName[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteId[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[0].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[1].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[2].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[3].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[4].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[5].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[6].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[7].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[8].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[9].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[10].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[11].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												while(*strcp != 0x3B)           
												*strcp++;
												*strcp++;
												for(i=0;i<16;i++) //EVENTSS
												{
													ab = 0;
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerIP[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerPort[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gpset.APN[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.xcon[ab++]=*strcp;
													*strcp++;
												}			
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.No_of_Alerts[ab++]=*strcp;
													*strcp++;
												}						
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.LogTimeGprs[ab++]=*strcp;
													*strcp++;
												}							
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													converter.RTU_OP_MODE[ab++]=*strcp;
													*strcp++;
												}							
												*strcp++;
											}
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgprt(SiteId,&Gparam,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgprt(SiteId,&Gparam,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
						
						case 39://RMODE
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
											memset(ARRAY_TRANS,'\0',200);
											reply_xmode(SiteId,ARRAY_TRANS,GPRS_MODE);
											Switch2DataMode();
											Send_Data_GPRS();
											Switch2CommandMode();
											}
											else
											{
											memset(ARRAY_TRANS,'\0',200);
											reply_xmode(SiteId,ARRAY_TRANS,GSM_MODE);
											Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;

						case 40://RGAPN
											memset(flash_memory,'\0',500);
											strcp = flash_memory;
											if(!read_4m_flash(flash_memory))
											{
												Set_Default();
											}
											else
											{
												strcp = flash_memory;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Ph.No1[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												while(*strcp != 0x3B)  
												*strcp++;
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteName[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													SiteId[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[0].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[1].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[2].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[3].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[4].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[5].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[6].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[7].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[8].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[9].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[10].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													daily[11].AutoReport[ab++]=*strcp;
													*strcp++;
												}
												while(*strcp != 0x3B)         
												*strcp++;
												*strcp++;
												for(i=0;i<16;i++) //EVENTSS
												{
													ab = 0;
													while(*strcp != 0x3B)
													*strcp++;
													*strcp++;
												}
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerIP[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gparam.ServerPort[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
												ab = 0;
												while(*strcp != 0x3B && *strcp != '\0')
												{
													Gpset.APN[ab++]=*strcp;
													*strcp++;
												}
												*strcp++;
											}
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgapn(SiteId,&Gpset,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_xgapn(SiteId,&Gpset,ARRAY_TRANS,GSM_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;										


						case 41://DVRSN
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_fname(SiteId,&Fver[0],&FIRM_Version[0],&finaldata,ARRAY_TRANS,GPRS_MODE);
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_fname(SiteId,&Fver[0],&FIRM_Version[0],&finaldata,ARRAY_TRANS,GPRS_MODE);
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											break;
											
						case 42://KWH00
											
											KWH_Factor[0] = 0; //ADD											
											strcat_flash();
											write_in_flash(flash_memory);	
						
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GPRS_MODE); //ADD
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GSM_MODE); //ADD
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
										
											break;

						case 43://KWH02
											KWH_Factor[0] = 2; //ADD											
											strcat_flash();
											write_in_flash(flash_memory);	
						
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);
											
											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GPRS_MODE); //ADD
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GSM_MODE); //ADD
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
										
											break;
											
						case 44://KWH05
											KWH_Factor[0] = 5; //ADD											
											strcat_flash();
											write_in_flash(flash_memory);	
						
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);

											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GPRS_MODE); //ADD
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GSM_MODE); //ADD
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}
											
											break;
						case 45://KWH10
											KWH_Factor[0] = 10; //ADD											
											strcat_flash();
											write_in_flash(flash_memory);	
						
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);

											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GPRS_MODE); //ADD
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_KWHXX(ARRAY_TRANS,KWH_Factor,GSM_MODE); //ADD
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}										
											break;

						case 46://XCKWH //14JUNE
											CurrentPosition.CUM_KWH = ASCII_2_decimal(KWH_update);							
											strcat_flash();
											write_in_flash(flash_memory);	
						
											DMA_Cmd(DMA1_Channel5, DISABLE);
											delay_ms(5);

											if((Msg[mssg_clarify].DIAL_NUM[0] == '\0') && (RTU_MODE == GPRS_MODE))
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_XCKWH(ARRAY_TRANS,GPRS_MODE); //14JUNE
												Switch2DataMode();
												Send_Data_GPRS();
												Switch2CommandMode();
											}
											else
											{
												memset(ARRAY_TRANS,'\0',200);
												reply_XCKWH(ARRAY_TRANS,GSM_MODE); //ADD //14JUNE
												Send_SMS(&Msg[mssg_clarify].DIAL_NUM[0]);
											}										
											break;											

						default:
											break;
					}
					memset(Msg[mssg_clarify].DIAL_NUM,'\0',15);    // clear the array before next transmit starts
					memset(Msg[mssg_clarify].MESSAGE,'\0',15);    // clear the array before next transmit starts
				}
				IWDG_ReloadCounter();
				delay_ms(500);
				IWDG_ReloadCounter();
			}
			MESSAGE_OFF
			Block_LED=0;
			LED_INDICATOR=0;



			if(RTU_MODE==GPRS_MODE)
			{
			Switch2DataMode();
			DMA_Cmd(DMA1_Channel5, DISABLE);
			IWDG_ReloadCounter();
			EXT_INT_FLAG = 0;
// 			memset(Rx1Buffer,'\0',400);
// 			start_DMA_access1(7);
			}
			else{
			EXT_INT_FLAG = 1;
			}
		}
		if(SetAlarmIndication == 1)
		{
			MESSAGE_ON
			IWDG_ReloadCounter();
			switch(converter.OPER_MODE)
			{
				case AUTOMATIC:
											if(RTU_MODE == GSM_MODE && check_GSM_MODE()==0)
											{
												memset(ARRAY_TRANS,'\0',200);
												ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,4,RTU_MODE,Signal_Strength,converter.OPER_MODE);
												//ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,4);
												IWDG_ReloadCounter();
												Send_Data_SMS1();
												IWDG_ReloadCounter();
											}
											break;

				case GPRS_ONLY:
											break;
				case GSM_ONLY:
											if(RTU_MODE == GSM_MODE && check_GSM_MODE()==0)
											{
												memset(ARRAY_TRANS,'\0',200);
												ASCII_alignation_OnlineUPS(ARRAY_TRANS,&finaldata,&Sync,SiteId,Fver,4,RTU_MODE,Signal_Strength,converter.OPER_MODE);
												//ASCII_alignation_UPS_mobile_reader(ARRAY_TRANS,&finaldata,4);
												IWDG_ReloadCounter();
												Send_Data_SMS1();
												IWDG_ReloadCounter();
											}
											break;
				default:
								break;
			}
			if(RESET_FLAG == 1)
			{
				NVIC_SystemReset();
				RESET_FLAG=0;
				RESET_COUNT_FLAG=0;
			}
			SetAlarmIndication = 0;
			IWDG_ReloadCounter();
			delay_ms(200);
			IWDG_ReloadCounter();
			set_alert(Alert,converter.xcoonh,converter.NoAlerts);
			MESSAGE_OFF
		}
	}
}

/*******************************************************************************
* Function Name  : UserStrcpy16
* Description    : This Routine Copies the pu8_SRCString in to pu8_DestString.
* Input          : pointer to Strings
* Output         : Length in Bytes copied
* Return         : Length in Bytes copied
*******************************************************************************/
u16 UserStrcpy16(uc8 *pu8_SRCString,u8 *pu8_DestString)
{
  u16 u16_Temp=0;
  while(pu8_SRCString[u16_Temp]!='\0')
  {
    pu8_DestString[u16_Temp]=pu8_SRCString[u16_Temp];
    u16_Temp++;
  }
  pu8_DestString[u16_Temp]='\0';
  return u16_Temp;
}
/*******************************************************************************
* Function Name  : SendATCommand
* Description    : This Routine Sends AT Command to GSM Module, for response
*                   check the u8_TempDataBuff[] .
* Input          : String containing the AT Command and the length of the Command
*including data
* Output         : None
* Return         : None
*******************************************************************************/
void SendATCommand(u8 *u8_ATCommand, u16 u16_Length)
{
  u16 u16_TempCounter=0;
  u8_ATCommand[u16_Length++]=0x0D;

  for (u16_TempCounter=0;u16_TempCounter<u16_Length;u16_TempCounter++)
  {
    USART_SendData(USART3, u8_ATCommand[u16_TempCounter]);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)!=SET);
  }
}



 /*******************************************************************************
* Function Name  : Decrement_TimingDelay
* Description    :This will decrement the delay counter
* Input          : none.
* Output         : None
* Return         : None
*******************************************************************************/
void Decrement_TimingDelay(void)
{
  if(vu32_TimingDelay !=0x00)
    vu32_TimingDelay--;
}
/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : u16_Count: specifies the delay time length (time base 1ms).
* Output         : Status
* Return         : Status
*******************************************************************************/
void Delay(u32 u32_Count)
{
  vu32_TimingDelay = u32_Count;
  while(vu32_TimingDelay != 0)
  {
  }
}
/*******************************************************************************
* Function Name  : SysTick_Configuration
* Description    : Configure a SysTick Base time to 1 ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Configuration(void)
{
  /* SysTick interrupt each 1ms with counter clock equal to 9MHz */
  if (SysTick_Config(9000))
  {
    /* Capture error */
    while (1); //doubt
  }

  /* Select HCLK/8 as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}



#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/*****END OF FILE****/
