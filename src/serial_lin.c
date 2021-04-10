/*
  Platform specific functions for serial port and serial data log
  for linux
  tested on: Ubuntu 17.10 x86_64, compiler gcc 7.2.0
 */

#include "main.h"

#if defined(__linux__)

#include <time.h>
#include "rs232.h" /* https://www.teuniz.net/RS-232/index.html */


char* LogPatch="./RobotPrLog.txt";
FILE *logfile;

//Port numbers for config[] see rs232.c (__linux__ defined)
#define  port_Q 7 //Qty. of port variants in settings page
char port_sel[port_Q]={2,16,17,18,19,20,21}; // ttyS2, ttyUSB0..ttyUSB5
//number of port in port_sel and UI
int portN=1;


int RbSerial_open_port()
{
	printf("TTY port selection =============================\n");
	printf("Select tty port (serial port):\n");
	printf("[%s] %-12s - default port for Orange Pi\n",portN==0?">":" ",comports[port_sel[0]]);
	for(uint8_t i=1;i<port_Q;i++){
		printf("[%s] %-12s - USB-UART adapter \n",portN==i?">":" ",comports[port_sel[i]]);
	}
	printf("------------------------------------------------\n");
	printf("Change port:  'W' - Down,      'S' - Up   \n");
	printf("Check:        'Z' - Check                 \n");

	int update_disp=0;
	while(update_disp==0){
		char character=getch();
		update_disp=1;
		switch(character)
		{
			case 'W':
			case 'w': { //Up
			 if(portN){portN--;}
			 else{portN=port_Q-1;}
			 break;
			}
			case 'S':
			case 's': { //Down
			 portN++;
			 if(portN>port_Q-1){portN=0;}
			 break;
			}
			case 'Z':
			case 'z': {
				if(RS232_OpenComport(port_sel[portN],portbaund,"8N1\0")) {
					printf("FAIL: port not opened\n");
					return 1;
				}
				//RS232_flushRX(port_sel[portN]);
				//RS232_flushTX(port_sel[portN]);
				printf("OK: port opened\n");
				#if MasterLog==1
				logfile = fopen(LogPatch, "w+");
				if (logfile<0) {
					printf("WARNING: log not created\n");
					RbSerial_Delay(1000);
				}
				else{
					fprintf(logfile,"App ver.: %s\n",version_string);
					fprintf(logfile,"Log for: %s (%7d-%3s)\n",comports[port_sel[portN]],portbaund,portformat);
					LogTimeStamp_Reset();
					printf("OK:Log started\n");
					fclose(logfile);
					RbSerial_Delay(500);
				}
				#if Reset_Port_Auto>0
				close(port);
				#endif
				#else
				RbSerial_Delay(500);
				#endif //MasterLog
				return 0;
			  break;
			}
			default:
			{ update_disp=0;}
		}
	}

	return 1;
}

void RbSerial_print_port(){
	printf("Connected to: %-12s (%7d-%3s)\n",comports[port_sel[portN]],portbaund,portformat);
}


uint16_t RbSerial_send_Buff(uint8_t* data, uint16_t len){
	return RS232_SendBuf(port_sel[portN],data,len)==len?len:0;
}

uint8_t RbSerial_receive(uint8_t* data){
	return RS232_PollComport(port_sel[portN],data,1);
}

void RbSerial_Delay(uint16_t ms){
	struct timespec tw;
	tw.tv_sec=ms/1000;
	tw.tv_nsec=ms%1000;
	tw.tv_nsec*=10e6;
	if(tw.tv_nsec>10e9-1) {
		tw.tv_nsec=10e9-1;
	}
	nanosleep(&tw,NULL);
}


#if TimeStampAbsolute == 0
struct timespec time_ms_null;
#endif
// Set print current data and time to log
void LogTimeStamp_Reset(){
  time_t time_now = time(NULL);
  struct tm time_loc = *localtime(&time_now);
  fprintf(logfile,"Date: %02d/%02d/%04d\n",
		  time_loc.tm_mday,time_loc.tm_mon,time_loc.tm_year+1900);
  fprintf(logfile,"Time: %02d:%02d:%02d\n",
		  time_loc.tm_hour,time_loc.tm_min,time_loc.tm_sec);
  #if TimeStampAbsolute == 0
  timespec_get(&time_ms_null, TIME_UTC);
  time_ms_null.tv_nsec/=10e6; //ns to ms
  fprintf(logfile,"\r\n %4s.%-3s","s","ms");
  #endif
}

void LogTimeStamp_Write(){
  #if TimeStampAbsolute == 0
  struct timespec time_ms;
  timespec_get(&time_ms, TIME_UTC);
  time_ms.tv_nsec/=10e6; //ns to ms
  if(time_ms_null.tv_nsec>time_ms.tv_nsec)
  {   time_ms.tv_nsec+=1000;
	  time_ms.tv_sec--;  }
  time_ms.tv_nsec-=time_ms_null.tv_nsec;
  time_ms.tv_sec-=time_ms_null.tv_sec;
  fprintf(logfile,"\r\n %04d.%03d : ",
		  time_ms.tv_sec,time_ms.tv_nsec);
  #else
  struct timespec time_ms;
  struct tm time_loc;
  time_t time_now;
  time_now = time(NULL);
  time_loc = *localtime(&time_now);
  timespec_get(&time_ms, TIME_UTC);
  fprintf(logfile,"\r\n %02d:%02d:%02d.%03d : ",
  time_loc.tm_hour,time_loc.tm_min,time_loc.tm_sec,(int)(time_ms.tv_nsec/10e6));
  #endif
}

void MaserLogAddLine(uint8_t* data, uint8_t Len){
    #if MasterLog==1
    	logfile = fopen(LogPatch, "a");
    	LogTimeStamp_Write();
        fprintf(logfile,"Tx: ");
        if(Len>50){Len=50;}
        for(uint8_t i=0;i<Len-1;i++){
        	fprintf(logfile,"%02X ",data[i]);
        }
        fclose(logfile);
    #endif //MasterLog
}

void MaserLogAddFrstRx(uint8_t data){
    #if MasterLog==1
	logfile = fopen(LogPatch, "a");
	LogTimeStamp_Write();
	fprintf(logfile,"Rx: %02X",data);
	fclose(logfile);
    #endif //MasterLog
}

void MaserLogAddNextRx(uint8_t data){
    #if MasterLog==1
	logfile = fopen(LogPatch, "a");
	fprintf(logfile," %02X",data);
	fclose(logfile);
    #endif //MasterLog
}

void MaserLogAddComment(char* data) {
    #if MasterLog==1
	logfile = fopen("./RobotPrLog.txt", "a+");
	LogTimeStamp_Write();
	fprintf(logfile,"%s",data);
	fclose(logfile);
    #endif //MasterLog
}

#endif //defined(__linux__)

