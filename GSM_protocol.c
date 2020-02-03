#include "stm32f10x.h"
#include "main.h"
#include "ATCGSM.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "parameters.h"

extern char KWH_Factor[]; //ADD
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

struct GPRS
{
	char ServerIP[20];
	char ServerPort[10];
	char mode[5];
};

struct GprsSet
{
	char APN[50];	
	char log_time_gprs[10];					
};

struct Auto
{
	char AutoReport[8];
}daily[12];  

struct Reply
{
	char PhNo1[15];
	char PhNo2[15]; 
	char Site[15];
}PHvalue;

struct Mobile
{
	char No1[15];
	char No2[15];
};

struct Time4Alarm
{
	char Time[5];
	unsigned int time_event;
	unsigned int independent_counter;
	unsigned int count;
};

struct MSSG
{
	char DIAL_NUM[15];
	char MESSAGE[15];
	char ServerIndex;	
};

union CumulativeKWH //14JUNE
{
   unsigned long CUM_KWH;
   char bytes[4];
};

extern union CumulativeKWH CurrentPosition;
extern struct typeconverter converter;
extern struct GPRS Gparam;	
extern struct GprsSet Gpset;	
extern struct Mobile Ph;
extern struct oneP datareceived,finaldata;
extern struct MSSG Msg[10];
struct Time4Alarm Event[32];    

static char *STATUS_ACK[] = {"XCODE","STATS","NUMBR","CSTAT","XTIME","XSITE","XPH01","DPH01","XPH03","XPH04","XPH05","XPH06",
														 "RESET","XCONP","XNAME","MSTAT","XEBRH","XBTRH","IPADD","XLOGG","ASTAT","DSTAT","XCONH","XCNFG",
														 "XGPDT","XDFLT","XGLOG","XMODE","XGSIP","XGPRT","XGAPN","RREAD","RNAME","RTIME","RCNFG","RCONP",
                             "RGLOG","RGSIP","RGPRT","RMODE","RGAPN","DVRSN","KWH00","KWH02","KWH05","KWH10","XCKWH"}; //ADD //14JUNE
static char *SMS_RESPONSE[]={"+CMT: ","RING","+IPD,","+RECEIVE","CLOSED"};
	
unsigned char validSMS,garbage;
unsigned int dial_num_test=0;
		
char BUFFER[20];
char CNTRYCODE[5];
char DevType[5];
char SiteId[15];
char Alarm_config[15];
char SiteName[15];
char IPAdd[15];
char LogTime[15];
char *ptrDIAL[10];
char *ptrMESSAGE[10];
char *tfr;
char BUFF[100];
char *ptSMS,*si;
char TFRAME[15];
char KWH_update[15]; //14JUNE

extern unsigned int increment,__configure;
extern unsigned int configurable;
extern unsigned int __configure;
extern unsigned char msg_slot;
extern char Rx1Buffer[200];

void reply_MobileNo(struct Mobile *ext,char *dt,char *id,char cmode);
void reply_extract_phoneNo_config(char *,char *,char *,struct Time4Alarm *,char cmode);
void reply_extract_phoneNo(char *id,char *drag,char cmode);
void reply_SITEID(char *,char *);
void reply_xconp(char *,char,char *,char);
void reply_xname(char *id,char *snam, char *drag,char cmode);
void reply_fname(char *id,char *snam,char *fname,struct OnlineUPS *, char *drag,char cmode);
void reply_gpdat(char *,char *,struct GPRS *,struct GprsSet *,struct Mobile *,char *);
void reply_gpdt1(char *,struct GPRS *,char *);
void reply_ltime(char *,struct GprsSet *,char *,char);
void reply_xgsip(char *,struct GPRS *,char *,char);
void reply_xgprt(char *,struct GPRS *,char *,char);
void reply_xgapn(char *,struct GprsSet *,char *,char);
void reply_xmode(char *,char *,char);
void reply_IPadd(char *,char *,char *,char );
void reply_ipadd(char *,char *,char *,char *,char );
void Send_ConfigAll(char *sname,char *id,struct GPRS *gpdt,struct GprsSet *gset,struct Mobile *ext,char *dt,char cmode);
void reply_rconp(char *id,char xcn, char *drag,char cmode);
void reply_reset(char *, char *id);
//*******************
void reply_KWHXX(char *, char *res_factor,char ); //ADD
void reply_XCKWH(char *, char); //14JUNE
//*******************

char SMS_control(char *,char);
unsigned char received_msgtype(char *);

extern unsigned int delay_ms(volatile unsigned int);
extern unsigned int ASCII_2_decimal(char *);
extern void Write_BKP_register(unsigned char , char );
extern void haultCall(void);
extern void decimal_ASCII_conversion(unsigned int,char *,unsigned char);
extern void decimal_ASCII_conversion_4byte(unsigned long,char *); //14JUNE

/******************************************************/
char SMS_control(char *SMS,char ref)
{
	static unsigned char i,j,k,ki,std;
	char debugBuff[400],TableEvent[6];
	static unsigned char valid_data;
	unsigned char icnf,jcnf,kcnf;
	
	std = 0;
	do
	{  		   
		if(std >= 10)
		{
			std = 0; 
		}
		valid_data = 0;
		garbage = 0;
		while(valid_data==0 && garbage==0)
		{
			if(ref==0)
			{
				memset(debugBuff,'\0',400);
				while(*SMS++ != '\n');
				ki=0;
				while(*SMS != '\r')
				{
					debugBuff[ki++]=*SMS;
					*SMS++;  
				}
				*SMS++;
				si=&debugBuff[0];
				if(!strcmp(si,SMS_RESPONSE[1]))
				{
					garbage = 1;
					delay_ms(5000);
					haultCall();
				}
				else
				{
					memset(BUFF,'\0',100);   
					ptSMS = &BUFF[0];
					for(j=0;j<6;j++)
					{
						BUFF[j] = *si++;
					}
					if(!strcmp(ptSMS,SMS_RESPONSE[0]))
					{
						valid_data = 1;
						*SMS++;
						debugBuff[ki++]='\r';
						debugBuff[ki++]='\n';
						while(*SMS != '\r')
						{
							debugBuff[ki++]=*SMS;
							*SMS++;  
						}
						while(*SMS++ != '\n');
					}
					else
					{
						*SMS++;
						if(*SMS=='\0')
						{
							garbage = 1; 
						}
					}
				}
			}
			else
			{
				memset(debugBuff,'\0',400);
				while(*SMS != '\r' && *SMS != '\0');
				*SMS++;
				while(*SMS != '\n' && *SMS != '\0');
				*SMS++;
				ki=0;               
				while(*SMS != '\r'&& *SMS != '\0')
				{
					debugBuff[ki++]=*SMS;
					*SMS++;  
				}
				*SMS++;
				si=&debugBuff[0];
				if(!strcmp(si,SMS_RESPONSE[4]))
				{
					validSMS = 3;
				} 
				else if(*SMS =='\n')
				{
					valid_data = 1;
					*SMS++;
				}
				else
				{
					*SMS++;
					if(*SMS=='\0')
					{
						garbage = 1; 
					}
				}
			}
		}
		if(valid_data==1)        
		{
			valid_data=0;
			si=&debugBuff[0];
			if(ref == 0)
			{
				while(*si++ != '\"');
				memset(Msg[std].DIAL_NUM,'\0',15);
				j=0;
				while(*si != '\"' && *si != '\0')
				{
					Msg[std].DIAL_NUM[j++] = *si;
					*si++;
				}
				dial_num_test=1;  
				while(*si++ != '\n' && *si != '\0');
			}
			memset(Msg[std].MESSAGE,'\0',15);
			for(k=0;k<5;k++)
			{
				Msg[std].MESSAGE[k] = *si;
				*si++;
			} 
			switch(received_msgtype(Msg[std].MESSAGE))
			{
				case 24://XGPDT
								if(ref==0)
								{			
									*si++;
									k = 0;
									memset(Gparam.ServerIP,'\0',15); 
									memset(Gpset.APN,'\0',50); 
									memset(Gparam.ServerPort,'\0',15); 
									memset(converter.xcon,'\0',2);
									memset(converter.No_of_Alerts,'\0',2);
									memset(converter.RTU_OP_MODE,'\0',2);
									memset(converter.LogTimeGprs,'\0',2);

									while(*si != '/' && *si!='\0')
									{
										Gparam.ServerIP[k++] = *si;              // SERVER IP
										*si++;
									}
									*si++; k =0;
									while(*si != '/' && *si!='\0')
									{
										Gparam.ServerPort[k++] = *si;  	   // server Port
										*si++;
									}
									*si++;	k=0;
									while(*si != '/' && *si!='\0')
									{
										Gpset.APN[k++] = *si;                    // APN
										*si++;
									}
									*si++;	k=0;
									memset(Gpset.log_time_gprs,'\0',10); 											
									while(*si != '/' && *si!='\0')
									{
										Gpset.log_time_gprs[k++] = *si;        // log duration
										*si++;
									}
									if(Gpset.log_time_gprs[0]=='\0')
										strcpy(Gpset.log_time_gprs,"30");
									converter.LogTimeGprs[0] = ASCII_2_decimal(Gpset.log_time_gprs);
									converter.TimeGprs = ASCII_2_decimal(Gpset.log_time_gprs);
									if(converter.TimeGprs == 0)
									{ 
										converter.LogTimeGprs[0] = 1;
										converter.TimeGprs = 1;
									}
									else if(converter.TimeGprs > 120)
									{
										converter.LogTimeGprs[0] = 120;
										converter.TimeGprs = 120;
									}
									*si++;	
									k=0;	
									memset(SiteId,'\0',15); 
									while(*si != '/' && *si!='\0')
									{
										SiteId[k++] = *si;              // site ID
										*si++;
									}
									*si++;	
									k = 0;
									memset(SiteName,'\0',15);   // extract Site Name
									while(*si != '/' && *si!='\0')
									{
										SiteName[k++] = *si;      // site name
										*si++;
									}
									*si++;
									memset(Ph.No1,'\0',15); 
									k = 0;    // extract PH01
									while(*si != '\0' && *si != '/')
									{
										Ph.No1[k++] = *si;                   // server Number
										*si++;
									}
									*si++;
									k=0; 
									memset(Gparam.mode,'\0',5); 
									while(*si!='\0' && *si != '/')
									{
										Gparam.mode[k++] = *si;
										*si++;
									}
									if(Gparam.mode[0] == '\0')
									Gparam.mode[0] = 0x30;
									converter.RTU_OP_MODE[0] = ASCII_2_decimal(Gparam.mode);
									converter.OPER_MODE = ASCII_2_decimal(Gparam.mode);
									*si++;
									memset(daily[0].AutoReport,'\0',8);
									converter.No_of_Alerts[0] = 0;
									converter.NoAlerts = 0;
									k=0;
									while(*si!='\r' && *si!='\0')
									{
										daily[0].AutoReport[k++] = *si;
										*si++;
									}
									*si++;
									converter.No_of_Alerts[0]++;
									converter.NoAlerts++;	 
									validSMS = 1;
									converter.xcon[0] = 2;
									converter.xcoonh =2;						
								}
								++std;   
								break;

				case 3://CSTAT 
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;
								
				case 1://STATS
								if(*si == '\0')
								{
									if(ref==0)
										validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;					
			
				case 2://NUMBR
								if(*si == '\0')
								{
									if(ref==0)
										validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;  

				case 4://XTIME
								tfr = TFRAME;
								memset(TFRAME,'\0',15); 
								while(*si != '\0')
								{
									*tfr++ = *si;
									*si++;
								}
								*si++;
								validSMS = 1;
								++std; 
								break;
				
				case 5://XSITE
								k=0;
								memset(SiteId,'\0',15); 
								while(*si != '\0')
								{
									SiteId[k++] = *si;
									*si++;
								}
								*si++;
								validSMS = 1;
								++std;  
								break;
				
				case 6://XPH01
								k=0;
								memset(Ph.No1,'\0',15); 
								while(*si != '\0')
								{
									Ph.No1[k++] = *si;
									*si++;
								}
								*si++;
								validSMS = 1;
								++std;
								break;
				
				case 7://DPH02
								k=0;
								memset(Ph.No2,'\0',15);  
								while(*si != '\0')
								{
									Ph.No2[k++] = *si;
									*si++;
								}
								*si++;
								validSMS = 1;
								++std;
								break;

				case 12://RESET
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;
				
				case 13://XCONP
								memset(converter.xcon,'\0',2);
								memset(converter.No_of_Alerts,'\0',2);
								for(i=0;i<12;i++)
								{
									memset(daily[i].AutoReport,'\0',8);
								}
								i=0;
								converter.No_of_Alerts[0] = 0;
								converter.NoAlerts = 0;
								while(*si!='\0')
								{ 
									k=0;
									while(*si!=0x2F && *si!='\0')
									{
										daily[i].AutoReport[k++] = *si;
										*si++;
									}
									*si++;
									i++;
									converter.No_of_Alerts[0]++;
									converter.NoAlerts++;	 
								}
								validSMS = 1;
								++std;
								converter.xcon[0] = 2;
								converter.xcoonh =2;
								break;
				
				case 14://XNAME
								k=0;
								memset(SiteName,'\0',15); 
								while(*si != '\0')
								{
									SiteName[k++] = *si;
									*si++;
								}
								*si++;
								validSMS = 1;
								++std;
								break;
		
				case 18://IPADD
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;
				
				case 22://XCONH
				memset(converter.xcon,'\0',2);
				if(*si == '\0')
				{
					validSMS = 1;
					*si++;
				}
				else
				{
					while(*si != '\n')
					{
						*si++;
					}
				}	  	 
				++std;
				converter.xcon[0] = 1;
				converter.xcoonh =1;
				break;

				case 23://XCNFG
								k=0;
								for(icnf=0;icnf<25;icnf++)
								{
									memset(Event[icnf].Time,'\0',5); 
								}
								memset(Alarm_config,'\0',15); 
								while(*si!=':' && *si!='\0')
								{
									Alarm_config[k++] = *si;
									*si++;
								}
								*si++;
								for(icnf=0;icnf<25;icnf++)
								{
									jcnf=0;
									while(*si!='/' && *si!='\0')
									{
										Event[icnf].Time[jcnf++]=*si;
										*si++;
									}
									*si++;
								}
								validSMS = 1;
								++std;
								configurable = ASCII_2_decimal(Alarm_config);
								for(icnf=0;icnf<25;icnf++)
								{ 
									memset(TableEvent,'\0',6);
									kcnf = 0;
									while(Event[icnf].Time[kcnf] != '\0')
									{
										TableEvent[kcnf] = Event[icnf].Time[kcnf];
										kcnf++;
									}
									Event[icnf].time_event = ASCII_2_decimal(TableEvent); 
								} 
								break;
				
				case 25://XDFLT
								if(*si == '\0')
								{
									if(ref==0)
										validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;

				case 26://XGLOG
								memset(converter.LogTimeGprs,'\0',2);						
								k=0; 
								memset(Gpset.log_time_gprs,'\0',10); 
								while(*si != '\0' && *si != '\r')
								{
									Gpset.log_time_gprs[k++] = *si;
									*si++;
								}
								if(Gpset.log_time_gprs[0]=='\0')
									strcpy(Gpset.log_time_gprs,"30");
								converter.LogTimeGprs[0] = ASCII_2_decimal(Gpset.log_time_gprs);
								converter.TimeGprs = ASCII_2_decimal(Gpset.log_time_gprs);
								if(converter.LogTimeGprs[0] == 0)
									converter.LogTimeGprs[0] = 1;
								else if(converter.LogTimeGprs[0] > 120)
									converter.LogTimeGprs[0] = 120;
								*si++;	
								validSMS = 1;
								++std;   
								break;																				

				case 27://XMODE
								memset(converter.RTU_OP_MODE,'\0',2); 
								k=0; 
								memset(Gparam.mode,'\0',5); 
								while(*si!='\0')
								{
									Gparam.mode[k] = *si;
									*si++;
								}
								if(Gparam.mode[0] == '\0')
									Gparam.mode[0] = 0x30;											
								*si++;	
								k=0; 
								converter.RTU_OP_MODE[0] = ASCII_2_decimal(Gparam.mode);
								converter.OPER_MODE = ASCII_2_decimal(Gparam.mode);
								*si++;
								validSMS = 1;
								++std;   
								break;

				case 28://XGSIP
								k=0; 
								memset(Gparam.ServerIP,'\0',15); 
								while(*si!='\0')
								{
									Gparam.ServerIP[k++] = *si;
									*si++;
								}
								*si++;	
								validSMS = 1;
								++std;   
								break;		

				case 29://XGPRT
								memset(Gparam.ServerPort,'\0',15); 
								k =0;
								while(*si!='\0')
								{
									Gparam.ServerPort[k++] = *si;
									*si++;
								}
								*si++;	
								validSMS = 1;
								++std;   
								break;	

				case 30://XGAPN
								memset(Gpset.APN,'\0',50); 
								k=0;
								while(*si!='\0')
								{
									Gpset.APN[k++] = *si;
									*si++;
								}
								*si++;	
								validSMS = 1;
								++std;   
								break;


				case 31://RREAD
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;
				
				case 32://RNAME
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;

				case 33://RTIME
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;		

				case 34://RCNFG
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;
				
				case 35://RCONP
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;

				case 36://RGLOG
								if(*si == '\0')
								{
								validSMS = 1;
								*si++;
								}
								else
								{
								while(*si != '\n')
								{
								*si++;
								}
								}	  	 
								++std; 
								break;

				case 37://RGSIP
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;		

				case 38://RGPRT
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;

				case 39://RMODE
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;		

				case 40://RGAPN
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;		

				case 41://DVRSN
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;	

				case 42://KWH00
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;	

				case 43://KWH02
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;	

				case 44://KWH05
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;	

				case 45://KWH10
								if(*si == '\0')
								{
									validSMS = 1;
									*si++;
								}
								else
								{
									while(*si != '\n')
									{
										*si++;
									}
								}	  	 
								++std; 
								break;

				case 46://XCKWH //14JUNE
								k=0;
								memset(KWH_update,'\0',15); 
								while(*si != '\0')
								{
									KWH_update[k++] = *si;
									*si++;
								}
								*si++;
								validSMS = 1;
								++std;  
								break;									

				default:
								while(*si != '\n')
								{
									*si++;
								}	  
								memset(Msg[std].MESSAGE,'\0',15);    // clear the buffer in which message is stored
								memset(Msg[std].DIAL_NUM,'\0',15);   // clear the buffer in which number is stored.
								break;
				
			}
		}
		else 
		{
			while(*si++ != '\0');
		} 
	}while(*SMS != '\0');
	
	return 	validSMS ;		
}
/******************************************************/
void reply_extract_phoneNo(char *id,char *drag,char cmode)
{
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_extract_phoneNo_config(char *id,char *alarm,char *drag,struct Time4Alarm *eve,char cmode)
{
	unsigned char i,j;
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	while(*alarm != '\0')
	{
		*drag = *alarm++;
		*drag++;
	}
	*drag++ = ':';
	for(i=0;i<15;i++)
	{ 
		j=0;
		while((eve+i)->Time[j] != '\0')   
		{
			*drag++ = (eve+i)->Time[j];
			j++;
		}
		*drag++ = '/'; 
	}
	j=0; 	   
	while((eve+i)->Time[j] != '\0')   
	{
		*drag++ = (eve+i)->Time[j];
		j++;
	}  
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_MobileNo(struct Mobile *ext,char *dt,char *id,char cmode)
{
	unsigned char i;
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // Slash
	i = 0;
	while(*id != '\0')
	{
		*dt++ = *id++;
		i++;
	}
	*dt++ = 0x2F;   // Slash
	i = 0;
	while(ext->No1[i] != '\0')
	{
		*dt++ = ext->No1[i++];
	}
	*dt++ = 0x2F;   // Slash
	i = 0;
	while(ext->No2[i] != '\0')
	{
		*dt++ = ext->No2[i++];
	}
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}
/******************************************************/
unsigned char received_msgtype(char *typemsg)
{
	if(!strcmp(typemsg,STATUS_ACK[0]))
	{
		return 0;
	}  
	else if(!strcmp(typemsg,STATUS_ACK[1]))
	{
		return 1;
	}  
	else if(!strcmp(typemsg,STATUS_ACK[2]))
	{
		return 2;
	}  
	else if(!strcmp(typemsg,STATUS_ACK[3]))
	{
		return 3;
	}  
	else if(!strcmp(typemsg,STATUS_ACK[4]))
	{
		return 4;
	}
	else if(!strcmp(typemsg,STATUS_ACK[5]))
	{
		return 5;
	}
	else if(!strcmp(typemsg,STATUS_ACK[6]))
	{
		return 6;
	}
	else if(!strcmp(typemsg,STATUS_ACK[7]))
	{
		return 7;
	}
	else if(!strcmp(typemsg,STATUS_ACK[8]))
	{
		return 8;
	}
	else if(!strcmp(typemsg,STATUS_ACK[9]))
	{
		return 9;
	}
	else if(!strcmp(typemsg,STATUS_ACK[10]))
	{
		return 10;
	}
	else if(!strcmp(typemsg,STATUS_ACK[11]))
	{
		return 11;
	}
	else if(!strcmp(typemsg,STATUS_ACK[12]))
	{
		return 12;
	}
	else if(!strcmp(typemsg,STATUS_ACK[13]))
	{
		return 13;
	}
	else if(!strcmp(typemsg,STATUS_ACK[14]))
	{
		return 14;
	}
	else if(!strcmp(typemsg,STATUS_ACK[15]))
	{
		return 15;
	}
	else if(!strcmp(typemsg,STATUS_ACK[16]))
	{
		return 16;
	}
	else if(!strcmp(typemsg,STATUS_ACK[17]))
	{
		return 17;
	}
	else if(!strcmp(typemsg,STATUS_ACK[18]))
	{
		return 18;
	}
	else if(!strcmp(typemsg,STATUS_ACK[19]))
	{
		return 19;
	}
	else if(!strcmp(typemsg,STATUS_ACK[20]))
	{
		return 20;
	}
	else if(!strcmp(typemsg,STATUS_ACK[21])) 
	{
		return 21;
	}								   	  
	else if(!strcmp(typemsg,STATUS_ACK[22]))
	{
		return 22; 
	}
	else if(!strcmp(typemsg,STATUS_ACK[23]))
	{
		return 23;
	}
	else if(!strcmp(typemsg,STATUS_ACK[24]))    
	{
		return 24;
	}
	else if(!strcmp(typemsg,STATUS_ACK[25]))   
	{
		return 25;
	}
	else if(!strcmp(typemsg,STATUS_ACK[26]))   
	{
		return 26;
	}
	else if(!strcmp(typemsg,STATUS_ACK[27]))  
	{
		return 27;
	}
	else if(!strcmp(typemsg,STATUS_ACK[28]))  
	{
		return 28;
	}
	else if(!strcmp(typemsg,STATUS_ACK[29]))   
	{
		return 29;
	}
	else if(!strcmp(typemsg,STATUS_ACK[30]))   
	{
		return 30;
	}
	else if(!strcmp(typemsg,STATUS_ACK[31]))   
	{
		return 31;
	}
	else if(!strcmp(typemsg,STATUS_ACK[32]))   
	{
		return 32;
	}
	else if(!strcmp(typemsg,STATUS_ACK[33]))  
	{
		return 33;
	}
	else if(!strcmp(typemsg,STATUS_ACK[34]))    
	{
		return 34;
	}
	else if(!strcmp(typemsg,STATUS_ACK[35]))    
	{
		return 35;
	}
	else if(!strcmp(typemsg,STATUS_ACK[36]))    
	{
		return 36;
	}
	else if(!strcmp(typemsg,STATUS_ACK[37]))    
	{
		return 37;
	}
	else if(!strcmp(typemsg,STATUS_ACK[38]))   
	{
		return 38;
	}
	else if(!strcmp(typemsg,STATUS_ACK[39]))   
	{
		return 39;
	}
	else if(!strcmp(typemsg,STATUS_ACK[40]))    
	{
		return 40;
	}
	else if(!strcmp(typemsg,STATUS_ACK[41]))    
	{
		return 41;
	}
	else if(!strcmp(typemsg,STATUS_ACK[42]))    
	{
		return 42;
	}
	else if(!strcmp(typemsg,STATUS_ACK[43]))    
	{
		return 43;
	}
	else if(!strcmp(typemsg,STATUS_ACK[44]))    
	{
		return 44;
	}
	else if(!strcmp(typemsg,STATUS_ACK[45]))    
	{
		return 45;
	}	
	else if(!strcmp(typemsg,STATUS_ACK[46]))  //14JUNE  
	{
		return 46;
	}	
	else
	{
		return 99;  
	}
}
/******************************************************/
void reply_xconp(char *id,char xcn, char *drag,char cmode)
{
	char i,k;
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	*drag++ = (xcn + 0x30);
	if(xcn == 2)
	{
		*drag++ = 0x2F;   // send /
		for(i=0; i<12; i++)
		{
			k = 0;
			while(daily[i].AutoReport[k] !='\0')
			{ 
				*drag =  daily[i].AutoReport[k++];
				*drag++;	
			}
			if(i<11)
				*drag++ = 0x2F;			
		}
	}
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}	
}	 
/******************************************************/
void reply_xname(char *id,char *snam, char *drag,char cmode)
{
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	*drag++ = 0x52;   // 'S' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	while(*snam != '\0')
	{
		*drag = *snam++;
		*drag++;
	}
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_reset(char *drag, char *id)
{
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	*drag++ = 'H';
	*drag++ = 'a';
	*drag++ = 'r';
	*drag++ = 'd';
	*drag++ = 'w';
	*drag++ = 'a';
	*drag++ = 'r';
	*drag++ = 'e';
	*drag++ = 'R';
	*drag++ = 'E';
	*drag++ = 'S';
	*drag++ = 'E';
	*drag++ = 'T';
}
/******************************************************/
void reply_fname(char *id,char *snam,char *fname,struct OnlineUPS *UPSDATA, char *drag,char cmode)
{
	char PL[6];
	unsigned char i;
	if(cmode)                     // header
	{
		*drag++ = 0x0D;           // \r
		*drag++ = 0x0A;           // \n
	}
	*drag++ = 0x52;   // 'S' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	memset(PL,'\0',6);
	decimal_ASCII_conversion(UPSDATA->FirmwareVersion.parameter,PL,0);
	i=0;
	while(PL[i] != '\0')
	{
		*drag++ = PL[i++];
	}
	*drag++ = 0x2F;   // send /
	while(*snam != '\0')
	{
		*drag = *snam++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	while(*fname != '\0')
	{
		*drag = *fname++;
		*drag++;
	}
	if(cmode)                     // header
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
}
/******************************************************/
void Send_ConfigAll(char *sname,char *id,struct GPRS *gpdt,struct GprsSet *gset,struct Mobile *ext,char *dt,char cmode)
{
	unsigned char i,rmode,logtm;
	rmode=converter.xcoonh;
	if(cmode)                     // header
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	i = 0;
	while(gpdt->ServerIP[i] != '\0')
	{
		*dt++ = gpdt->ServerIP[i];
		i++;
	}
	*dt++ = 0x2F;   // Slash
	i = 0;
	while(gpdt->ServerPort[i] != '\0')
	{
		*dt++ = gpdt->ServerPort[i];
		i++;
	}
	*dt++ = 0x2F;   // Slash	
	i = 0;
	while(gset->APN[i] != '\0')
	{
		*dt++ = gset->APN[i];
		i++;
	}
	*dt++ = 0x2F;   // Slash
	logtm = converter.TimeGprs;
	if(logtm == 1)
	logtm=0;
	decimal_ASCII_conversion(logtm,gset->log_time_gprs,0);
	i = 0;
	while(gset->log_time_gprs[i] != '\0')
	{
		*dt++ = gset->log_time_gprs[i];
		i++;
	}
	*dt++ = 0x2F;   // Slash
	while(*sname != '\0')
	{
		*dt = *sname++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	i = 0;
	while(ext->No1[i] != '\0')
	{
		*dt++ = ext->No1[i];
		i++;
	}
	*dt++ = 0x2F;   // Slash
	*dt++ = rmode+0x30;
	if(cmode)                     // footer
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}		 
}	
/******************************************************/	 
void reply_rconp(char *id,char xcn, char *drag,char cmode)
{
	char i,k;
	
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*drag = *id++;
		*drag++;
	}
	*drag++ = 0x2F;   // send /
	*drag++ = (xcn + 0x30);
	*drag++ = 0x2F;   // send /
	for(i=0; i<12; i++)
	{
		k = 0;
		while(daily[i].AutoReport[k] !='\0')
		{ 
			*drag =  daily[i].AutoReport[k++];
			*drag++;	
		}
		if(i<11)
			*drag++ = 0x2F;			
	}
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_ipadd(char *dt,char *id,char *IP,char *port,char cmode)
{
	if(cmode)                     // header
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	while(*IP != '\0')
	{
		*dt = *IP++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	while(*port != '\0')
	{
		*dt = *port++;
		*dt++;
	}
	if(cmode)                     // footer
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}	 
/******************************************************/
void reply_ltime(char *id,struct GprsSet *gpdt,char *dt,char cmode)
{
	unsigned char i,logtm;

	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	logtm = converter.TimeGprs;
	if(logtm == 1)
		logtm=0;
	decimal_ASCII_conversion(logtm,gpdt->log_time_gprs,0);
	i = 0;
	while(gpdt->log_time_gprs[i] != '\0')
	{
		*dt++ = gpdt->log_time_gprs[i];
		i++;
	}
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_xmode(char *id,char *dt,char cmode)
{
	unsigned char mode_rtu;

	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	mode_rtu = converter.OPER_MODE;
	*dt++ = mode_rtu+0x30;
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_xgsip(char *id,struct GPRS *gpdt,char *dt,char cmode)
{
	unsigned char i;

	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	i = 0;
	while(gpdt->ServerIP[i] != '\0')
	{
		*dt++ = gpdt->ServerIP[i];
		i++;
	}
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_xgprt(char *id,struct GPRS *gpdt,char *dt,char cmode)
{
	unsigned char i;
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	i = 0;
	while(gpdt->ServerPort[i] != '\0')
	{
		*dt++ = gpdt->ServerPort[i];
		i++;
	}
	if(cmode)                   
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_xgapn(char *id,struct GprsSet *gset,char *dt,char cmode)
{
	unsigned char i;
	if(cmode)                     // header
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x52;   // 'R' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	i = 0;
	while(gset->APN[i] != '\0')
	{
		*dt++ = gset->APN[i];
		i++;
	}
	if(cmode)                     // footer
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}
/******************************************************/
void reply_IPadd(char *dt,char *id,char *IP,char cmode)
{
	if(cmode)                     // header
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
	*dt++ = 0x53;   // 'S' for reply
	*dt++ = 0x2F;   // send /
	while(*id != '\0')
	{
		*dt = *id++;
		*dt++;
	}
	*dt++ = 0x2F;   // send /
	while(*IP != '\0')
	{
		*dt = *IP++;
		*dt++;
	}
	if(cmode)                     // footer
	{
		*dt++ = 0x0D;          // \r
		*dt++ = 0x0A;          // \n
	}
}	
//***************************************************
void reply_KWHXX(char *drag, char *res_factor,char cmode)  // reply to turn on command // sukoon13
{
		if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	
	*drag++ = 'K';
	*drag++ = 'W';
	*drag++ = 'H';
	*drag++ = ' ';
	*drag++ = 'F';
	*drag++ = 'A';
	*drag++ = 'C';
	*drag++ = 'T';
	*drag++ = 'O';
	*drag++ = 'R';
	*drag++ = '=';
	*drag++ = ' ';
if(KWH_Factor[0] == 10)
	*drag++ = '1' ;
else
{
	*drag++ = 0x30;
	*drag++ = 0x2E;
	
*drag = res_factor[0] + 0x30;
*drag++;
}
  *drag++ = ' ';
	*drag++ = 'U';
	*drag++ = 'N';
	*drag++ = 'I';
  *drag++ = 'T';

	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	
}

/******************************************************************************/
void reply_XCKWH(char *drag, char cmode)  //XCKWH //14JUNE
{
	unsigned char i;
	char FOURBYTEPlace[11];
	
	if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	
	*drag++ = 0x52;   // 'R' for reply
	*drag++ = 0x2F;   // send /
	
	*drag++ = 'C';
	*drag++ = 'u';
	*drag++ = 'm';
	*drag++ = 'u';
	*drag++ = 'l';
	*drag++ = 'a';
	*drag++ = 't';
	*drag++ = 'i';
	*drag++ = 'v';
	*drag++ = 'e';
	*drag++ = ' ';
	*drag++ = 'k';
	*drag++ = 'W';
	*drag++ = 'h';
	*drag++ = '=';
	*drag++ = ' ';
	memset(FOURBYTEPlace,'\0',11);                                                     // Cum_KWH
	decimal_ASCII_conversion_4byte(CurrentPosition.CUM_KWH,FOURBYTEPlace);
	i=0;
  while(FOURBYTEPlace[i] != '\0' )
	{
		*drag++ = FOURBYTEPlace[i++];
	}
	
if(cmode)                   
	{
		*drag++ = 0x0D;          // \r
		*drag++ = 0x0A;          // \n
	}
	
}