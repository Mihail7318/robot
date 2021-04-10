#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>

#include "RobotSerProt.h"


#define MasterLog 1

HANDLE hSerial;

#if MasterLog==1
HANDLE hFile;
uint8_t LogOn=0;
SYSTEMTIME lt;
DWORD dwTemp;
uint8_t TimeStr[30];
#endif //MasterLog

void MaserLogAddComment(uint8_t* data);

int main(int argc, char **argv)
{
    #define version_string "0.2"
    #define baund (int)115200

    #define Param_Q 11

    typedef struct{
    int16_t read;
    int16_t write;
    int16_t min;
    int16_t max;
    uint8_t inc;
    }param_t;

    param_t Param[Param_Q]={
        {.read=1,   .write=1,   .min=0,     .max=20,    .inc=1},    //ID
        {.read=0,   .write=1,   .min=0,     .max=1,     .inc=1},    //On
        {.read=0,   .write=0,   .min=0,     .max=1000,  .inc=50},   //Pos
        {.read=1000,.write=1000,.min=0,     .max=30000, .inc=200},  //Time
        {.read=0,   .write=1,   .min=0,     .max=1,     .inc=1},    //Wait
        {.read=0,   .write=0,   .min=0,     .max=950,   .inc=50},   //Pos min
        {.read=1000,.write=1000,.min=50,    .max=1000,  .inc=50},   //Pos max
        {.read=0,   .write=0,   .min=-125,  .max=125,   .inc=5},    //Adj
        {.read=85,  .write=85,  .min=50,    .max=100,   .inc=5},    //Tmax
        {.read=6500,.write=6500,.min=6500,  .max=7000,  .inc=250},  //Vmin
        {.read=8500,.write=8500,.min=7500,  .max=9000,  .inc=250},  //Vmax
    };

    enum {
      ID=0,
      ON=1,
      POS=2,
      TIME=3,
      WAIT=4,
      POS_MIN=5,
      POS_MAX=6,
      POS_ADJ=7,
      T_MAX=8,
      V_MIN=9,
      V_MAX=10,
    }; 
    uint8_t cursor=0;

    uint16_t Bat;
    uint8_t v5V1=0; 
    uint8_t v5V2=0;
    
    uint8_t  servo_State=0;
    uint8_t  servo_Temp=25;
    uint16_t servo_Volt=8000;
    
    uint8_t bat_flag=0;
    uint8_t update_disp=1;
    uint8_t write_cmd=0;
    uint8_t read_cmd=1;
    uint8_t start_cmd=0;
    uint8_t stop_cmd=0;
    
    //Sensors data
    int16_t Accel[3];
    int16_t Giro[3];
    uint16_t Force[6];
    
    uint8_t NoAccel=1;
    uint8_t NoForce=1;
    
    int PortN=0;
    char port[20];
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};

    typedef enum {PORT_SEL,MODE_SEL,SERVO} state_t;
    state_t state=PORT_SEL;
    
    while(1){
        switch(state){
            case PORT_SEL:{
                system("cls");
                printf("Robot Console Protocol Tester v %s\n",version_string);
                printf("\nCOM port selection =============================\n");
                
                printf("Select Port number (between 1 and 99):");
                
                while(scanf("%d",&PortN) != 1)
                {
                    printf("Select Port number (between 1 and 99):");
                    while(getchar() != '\n');
                }
                if((0<PortN)&&(PortN<100)){
                    sprintf(port,"\\\\.\\COM%d",PortN);
                    printf("Connecting to : COM%d ",PortN);
                    
                    hSerial = CreateFile(port, GENERIC_READ|GENERIC_WRITE, 0, NULL,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
                    if (hSerial == INVALID_HANDLE_VALUE){
                        printf("- FAIL: port not open\n");
                        CloseHandle(hSerial);
                        Sleep(1000);
                        break;
                    }
                    
                    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
                    if (GetCommState(hSerial, &dcbSerialParams) == 0){
                        printf("- FAIL: cannot get params\n");
                        CloseHandle(hSerial);
                        Sleep(1000);
                        break;
                    }
                    dcbSerialParams.BaudRate = baund;
                    dcbSerialParams.ByteSize = 8;
                    dcbSerialParams.StopBits = ONESTOPBIT;
                    dcbSerialParams.Parity = NOPARITY;
                    if(SetCommState(hSerial, &dcbSerialParams) == 0){
                        printf("- FAIL: cannot set params\n");
                        CloseHandle(hSerial);
                        Sleep(1000);
                        break;
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
                        break;
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
                    tmpLen=sprintf(TimeStr,"  Log for: COM%d (%d-8-N-1)\n",PortN,baund);
                    WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
                    tmpLen=sprintf(TimeStr,"  Date: %02d/%02d/%04d\n",lt.wDay,lt.wMonth,lt.wYear);
                    WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
                    #endif //MasterLog

                    printf("- OK: port ready\n");
                    Sleep(700);
                    state=MODE_SEL;
                    getchar();
                }
                system("cls");
            break;} 
            case MODE_SEL:{
                state=SERVO;
                read_cmd=1;
            break;} 
            case SERVO:{
                
                //Set servo
                if(write_cmd){            
                    write_cmd=0;
                    uint8_t State=0;
                    MaserLogAddComment("write\0");
                    RbSer_Servo_Set_State((uint8_t)Param[ID].write,0,(uint8_t)Param[ON].write,&State);
                    RbSer_Servo_Prot_Set((uint8_t)Param[ID].write,Param[POS_MIN].write,Param[POS_MAX].write, 
                    (int8_t)Param[POS_ADJ].write,(int8_t)Param[T_MAX].write, 
                    Param[V_MIN].write,Param[V_MAX].write,&State);
                    printf("Write");
                    Sleep(200);
                    read_cmd=1;
                }
                //Move control
                if(stop_cmd){
                   MaserLogAddComment("stop\0");
                   uint8_t State=0;
                   RbSer_Servo_Set_State((uint8_t)Param[ID].write,0,(uint8_t)Param[ON].write,&State);
                   stop_cmd=0;
                   printf("Stop");
                   Sleep(200);
                   read_cmd=1;
                }
                else{
                    if(start_cmd){
                        uint8_t State=0;
                        Param[ON].write=1;
                        MaserLogAddComment("start\0");
                        RbSer_Servo_Set_State((uint8_t)Param[ID].write,1,1,&State);
                        if(State==0){
                            RbSer_Servo_Set_Pos((uint8_t)Param[ID].write,Param[POS].write,Param[TIME].write,Param[WAIT].write,&State);
                            if((State==0)&&(!Param[WAIT].write)){
                              RbSer_Servo_Set_State((uint8_t)Param[ID].write,1,1,&State);  
                            }                        
                        }
                        start_cmd=0;
                        printf("Start");
                        Sleep(200);
                        read_cmd=1;
                    }
                }
                //Check servo
                if(read_cmd){
                    MaserLogAddComment("read\0");
                    bat_flag=RbSer_bus_test(&Bat,&v5V1,&v5V2)?1:0;
                    if(RbSer_Accel_Giro_Get(Accel,Giro)){
                        NoAccel=0;
                    }
                    else{
                        NoAccel=1;
                    }
                    if(RbSer_Force_Get(Force)){
                        NoForce=0;
                    }
                    else{
                        NoForce=1;
                    }
                    uint8_t Temp[2];
                    if(RbSer_Servo_State((uint8_t)Param[ID].write,&servo_State,&Temp[0],&Param[POS].read)){
                        Param[ON].read=Temp[0];
                        if(RbSer_Servo_Prot_Get((uint8_t)Param[ID].write,&Param[POS_MIN].read,&Param[POS_MAX].read,
                        &Temp[0],&Temp[1],&servo_Temp,&Param[V_MIN].read, &Param[V_MAX].read,&servo_Volt)){
                            Param[POS_ADJ].read=Temp[0];
                            Param[T_MAX].read=Temp[1];
                            }
                    }
                    read_cmd=0;
                }

                //Show UI
                system("cls");
                printf("Robot Console Protocol Tester v %s\n",version_string);
                printf("Connected to: COM%-2d (%-6d-8-N-1)\n",PortN,baund);
                printf("\nStatus =========================================\n");
                if(bat_flag)
                {
                   printf("STATUS: Bat: %.2f ",((float)Bat)/1000.0);
                   if(v5V1>0){ printf("5Vch1: HIGH ");}
                   if(v5V1<0){ printf("5Vch1: LOW  ");}
                   if(v5V1==0){printf("5Vch1: OK   ");}
                   if(v5V2>0){ printf("5Vch2: HIGH \n");}
                   if(v5V2<0){ printf("5Vch2: LOW  \n");}
                   if(v5V2==0){printf("5Vch2: OK   \n");}
                }
                else
                {
                   printf("NO STATUS DATA\n");
                }
                printf("\nSensors ========================================\n");
                if(NoAccel==0){
                    printf("%-12s  X=%-8.3f  Y=%-8.3f  Z=%-8.3f\n","Accel[g]:",((float)Accel[0])/10000.0,((float)Accel[1])/10000.0,((float)Accel[2])/10000.0);
                    printf("%-12s  X=%-8.3f  Y=%-8.3f  Z=%-8.3f\n","Giro[dps]:",((float)Giro[0])/100.0,((float)Giro[1])/100.0,((float)Giro[2])/100.0);
                }
                else{
                    printf("NO ACCEL/GIRO DATA\n");
                }
                if(NoForce==0){
                printf("%-12s F1=%-8.2f F2=%-8.2f F3=%-8.2f\n","Force[N]:",((float)Force[0])/100.0,((float)Force[1])/100.0,((float)Force[2])/100.0);
                printf("%-12s F4=%-8.2f F5=%-8.2f F6=%-8.2f\n"," ",((float)Force[3])/100.0,((float)Force[4])/100.0,((float)Force[5])/100.0);
                }
                else{
                    printf("NO FORCE DATA\n");
                }
                printf("\nServo test =====================================\n");
                if(servo_State!=0xFF){
                printf("%s Servo ID: %02d [CONNECTED]\n",cursor==ID?"[>]":"[ ]",Param[ID].write);
                printf("    %.2f V %s - %d Deg %s - Rotor %s - %02x \n",
                ((float)servo_Volt)/1000.0,servo_State&0x02?"Ok":"Err",
                servo_Temp,servo_State&0x01?"Ok":"Err",
                servo_State&0x04?"unblocked":"blocked",servo_State);
                printf("-------------+----------------+----------------+\n");
                printf("    Param    |      Read      |     Write      |\n");
                printf("-------------+----------------+----------------+\n");
                
                printf("%s ON/OFF   |   %s - 0x%02x   |   %s - 0x%02x   |\n",cursor==ON?"[>]":"[ ]",
                Param[ON].read?" ON":"OFF",Param[ON].read,Param[ON].write?" ON":"OFF",Param[ON].write);
                
                printf("%s position | %5.1f - 0x%04x | %5.1f - 0x%04x |\n",cursor==POS?"[>]":"[ ]",
                ((float)Param[POS].read*240.0/1000.0),Param[POS].read,  ((float)Param[POS].write*240.0/1000.0),Param[POS].write);
                
                printf("%s mov time |   --  -  --    | %5.2f - 0x%04x |\n",cursor==TIME?"[>]":"[ ]",
                ((float)Param[TIME].write/1000.0),Param[TIME].write);
                
                printf("%s mov type |   --  -  --    | %s - 0x%02x   |\n",cursor==WAIT?"[>]":"[ ]",
                Param[WAIT].write?"IMMED":" WAIT",Param[WAIT].write);
                
                printf("%s pos min  | %5.1f - 0x%04x | %5.1f - 0x%04x |\n",cursor==POS_MIN?"[>]":"[ ]",
                ((float)Param[POS_MIN].read*240.0/1000.0),Param[POS_MIN].read,  ((float)Param[POS_MIN].write*240.0/1000.0),Param[POS_MIN].write);
                
                printf("%s pos max  | %5.1f - 0x%04x | %5.1f - 0x%04x |\n",cursor==POS_MAX?"[>]":"[ ]",
                ((float)Param[POS_MAX].read*240.0/1000.0),Param[POS_MAX].read,  ((float)Param[POS_MAX].write*240.0/1000.0),Param[POS_MAX].write);
                
                printf("%s pos adj  | %5.1f - 0x%04x | %5.1f - 0x%04x |\n",cursor==POS_ADJ?"[>]":"[ ]",
                ((float)Param[POS_ADJ].read*240.0/1000.0),Param[POS_ADJ].read,  ((float)Param[POS_ADJ].write*240.0/1000.0),Param[POS_ADJ].write);
                
                printf("%s Tmax     |   %03d - 0x%02x   |   %03d - 0x%02x   |\n",cursor==T_MAX?"[>]":"[ ]",
                Param[T_MAX].read,Param[T_MAX].read,Param[T_MAX].write,Param[T_MAX].write);
                
                printf("%s Vmin     | %5.2f - 0x%04x | %5.2f - 0x%04x |\n",cursor==V_MIN?"[>]":"[ ]",
                ((float)Param[V_MIN].read/1000.0),Param[V_MIN].read,  ((float)Param[V_MIN].write/1000.0),Param[V_MIN].write);
                
                printf("%s Vmax     | %5.2f - 0x%04x | %5.2f - 0x%04x |\n",cursor==V_MAX?"[>]":"[ ]",
                ((float)Param[V_MAX].read/1000.0),Param[V_MAX].read,  ((float)Param[V_MAX].write/1000.0),Param[V_MAX].write);
                printf("-------------+----------------+----------------+\n");
                printf("Select param: 'W' - Up,        'S' - Down       \n");
                printf("Chenge param: 'A' - Down,      'D' - Up         \n");
                printf("Update:       'Z' - Read,      'C' - Write      \n");
                printf("Move:         'Q' - Start,     'E' - Stop       \n");
                }
                else{
                cursor=0;
                printf("%s Servo ID: %02d [DISCONNECTED]\n\n",cursor==ID?"[>]":"[ ]",Param[ID].write);
                printf("------------------------------------------------\n");
                printf("Chenge ID:    'A' - Down,      'D' - Up   \n");
                printf("Update:       'Z' - Read       \n");
                }
                
                
                //Process KEY
                do{
                    update_disp=1;
                    char character=getch();
                    switch(character)
                    {
                        case 'W':  
                        case 'w': { //Up
                         if(servo_State!=0xFF){
                             if(cursor){cursor--;}
                             else{cursor=Param_Q-1;}
                         }
                         else{
                             cursor=0;
                         }
                         break;
                        }
                        case 'S':  
                        case 's': { //Down
                         if(servo_State!=0xFF){
                             cursor++;
                             if(cursor==Param_Q){cursor=0;}
                         }
                         else{
                             cursor=0;
                         }
                         break;
                        }
                        case 'D':  
                        case 'd': { //Right
                         Param[cursor].write+=Param[cursor].inc;
                         if(Param[cursor].write>Param[cursor].max){
                            Param[cursor].write=Param[cursor].max;
                         }
                         if(cursor==0){
                          read_cmd=1;
                         }
                         break;
                        }
                        case 'A':  
                        case 'a': { //Left
                         Param[cursor].write-=Param[cursor].inc;
                         if(Param[cursor].write<Param[cursor].min){
                            Param[cursor].write=Param[cursor].min;
                         }
                         if(cursor==0){
                          read_cmd=1;
                         }
                         break;
                        }
                        case 'Z':  
                        case 'z': {
                         read_cmd=1;
                         break;
                        }
                        case 'C':  
                        case 'c': {
                         write_cmd=1;
                         break;
                        }
                        case 'Q':  
                        case 'q': {
                         start_cmd=1;
                         break;
                        }
                        case 'E':  
                        case 'e': {
                         stop_cmd=1;
                         break;
                        }
                        default:
                        { update_disp=0;}
                    }
                    
                    if(Param[POS_MAX].write<Param[POS_MIN].write){
                        Param[POS_MIN].write=Param[POS_MIN].min;
                    }
                    if(Param[POS_MAX].write<Param[POS_MIN].write){
                        Param[POS_MAX].write=Param[POS_MAX].max;
                    }
                
                }while(!update_disp);
                
            break;} 
        }
    }
	return 0;
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
        tmpLen=sprintf(TimeStr,"\r\n %02d:%02d:%02d.%03d  Cm: %s",lt.wHour,lt.wMinute,lt.wSecond,lt.wMilliseconds,data);
        WriteFile(hFile,TimeStr,tmpLen, &dwTemp, NULL);
    }
    #endif //MasterLog
}