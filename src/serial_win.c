/*
  Platform specific functions for serial port and serial data log
  for Windows
  tested on: Windows 10 x86_64, compiler TDM-GCC-32
 */

#include "main.h"

#if !defined(__linux__)

#include <windows.h> /* WinAPI header */

HANDLE hSerial;

#if MasterLog==1
HANDLE hFile;
uint8_t LogOn=0;
SYSTEMTIME lt;
DWORD dwTemp;
uint8_t TimeStr[30];
#endif

int PortN=0;
char port[20];
DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};

int RbSerial_Start_Acces(){
	return 1;
}

void RbSerial_Stop_Acces(){

}


int RbSerial_open_port()
{
	printf("\nCOM port selection =============================\n");
	printf("Select Port number (between 1 and 99):");

	while(scanf("%d",&PortN) != 1)
	{
		printf("Select Port number (between 1 and 99):");
		while(getchar() != '\n');
	}
	if((0<PortN)&&(PortN<100))
	{
		sprintf(port,"\\\\.\\COM%d",PortN);
		printf("Connecting to : COM%d ",PortN);

		hSerial = CreateFile(port, GENERIC_READ|GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
		if (hSerial == INVALID_HANDLE_VALUE){
			printf("- FAIL: port not opened\n");
			CloseHandle(hSerial);
			Sleep(1000);
			return 1;
		}

		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
		if (GetCommState(hSerial, &dcbSerialParams) == 0){
			printf("- FAIL: cannot get params\n");
			CloseHandle(hSerial);
			Sleep(1000);
			return 2;
		}
		dcbSerialParams.BaudRate = portbaund;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;
		if(SetCommState(hSerial, &dcbSerialParams) == 0){
			printf("- FAIL: cannot set params\n");
			CloseHandle(hSerial);
			Sleep(1000);
			return 3;
		}

		timeouts.ReadIntervalTimeout = 20;
		timeouts.ReadTotalTimeoutConstant = 20;
		timeouts.ReadTotalTimeoutMultiplier = 10;
		timeouts.WriteTotalTimeoutConstant = 20;
		timeouts.WriteTotalTimeoutMultiplier = 10;
		if(SetCommTimeouts(hSerial, &timeouts) == 0){
			printf("- FAIL: cannot set timeouts\n");
			CloseHandle(hSerial);
			Sleep(1000);
			return 4;
		}
		#if MasterLog==1
		hFile = CreateFile("RobotPrLog.txt", GENERIC_WRITE, 0, NULL,
							   CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
		LogOn=1;
		if(INVALID_HANDLE_VALUE == hFile) {
		printf("- FAIL: cannot create log \n");
		CloseHandle(hFile);
		Sleep(1000);
		LogOn=0;
		}
		GetLocalTime(&lt);
		uint8_t tmpLen=0;
		tmpLen=sprintf(TimeStr,"  App ver.: %s\n",version_string);
		WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
		tmpLen=sprintf(TimeStr,"  Log for: COM%d (%-6d-%3s)\n",PortN,portbaund,portformat);
		WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
		tmpLen=sprintf(TimeStr,"  Date: %02d/%02d/%04d\n",lt.wDay,lt.wMonth,lt.wYear);
		WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
		#endif //MasterLog
		printf("- OK: port ready\n");
        Sleep(700);
		return 0;
	}
	return 1;
}

void RbSerial_print_port(){
	printf("Connected to: COM%-2d (%-6d-%3s)\n",PortN,portbaund,portformat);
}

DWORD bytes_written, total_bytes_written = 0;

uint16_t RbSerial_send_Buff(uint8_t* data, uint16_t len){
    if(!WriteFile(hSerial,data,len, &bytes_written, NULL)){
        printf("FAIL: cannot send data\n");
        CloseHandle(hSerial);
        return 0;
    }
    return len;
}

uint8_t RbSerial_receive(uint8_t* data){
      DWORD iSize;
      uint8_t sReceived;
      ReadFile(hSerial, &sReceived, 1, &iSize, NULL);
      if (iSize) {
        *data=sReceived;
        //printf("%02X",sReceived);
        return 1;
      }
      return 0;
}

void RbSerial_Delay(uint16_t ms){
    Sleep(ms);
}

void MaserLogAddLine(uint8_t* data, uint8_t Len){
    #if MasterLog==1
    if(LogOn){
        GetLocalTime(&lt);
        uint8_t tmpLen=0;
        tmpLen=sprintf(TimeStr,"\r\n %02d:%02d:%02d.%03d  Tx: ",lt.wHour,lt.wMinute,lt.wSecond,lt.wMilliseconds);
        WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
        if(Len>50){Len=50;}
        for(uint8_t i=0;i<Len-1;i++){
            tmpLen=sprintf(TimeStr,"%02X ",data[i]);
            WriteFile(hFile,TimeStr,tmpLen,&dwTemp, NULL);
        }
    }
    #endif //MasterLog
}

void MaserLogAddFrstRx(uint8_t data){
    #if MasterLog==1
    if(LogOn){
        GetLocalTime(&lt);
        uint8_t tmpLen=0;
        tmpLen=sprintf(TimeStr,"\r\n %02d:%02d:%02d.%03d  Rx: %02X",lt.wHour,lt.wMinute,lt.wSecond,lt.wMilliseconds,data);
        WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
    }
    #endif //MasterLog
}

void MaserLogAddNextRx(uint8_t data){
    #if MasterLog==1
    if(LogOn){
        uint8_t tmpLen=0;
        tmpLen=sprintf(TimeStr," %02X",data);
        WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
    }
    #endif //MasterLog
}

void MaserLogAddComment(uint8_t* data){
    #if MasterLog==1
    if(LogOn){
        GetLocalTime(&lt);
        uint8_t tmpLen=0;
        tmpLen=sprintf(TimeStr,"\r\n %02d:%02d:%02d.%03d %s\n",lt.wHour,lt.wMinute,lt.wSecond,lt.wMilliseconds,data);
        WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
    }
    #endif //MasterLog
}

#endif //!defined(__linux__)

