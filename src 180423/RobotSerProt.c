#include "RobotSerProt.h"

#if RbSerMaster==0
uint8_t RbSerial_Port_No=1; 
#endif 

void RbSer_callback(uint8_t Code);

/*  Private variables and types */

#define HEAD0 0x55 
#define HEAD1 0xA5
#define HEAD2 0xAA

#define MSG_BUFF_SZ  (100)
uint8_t Message_buff[MSG_BUFF_SZ];
uint8_t data[20];

typedef struct{
  uint8_t Code;
  uint8_t M_Len;
  uint8_t S_Len;
} 
RbSerCmdDesc_t;

#define RbSer_CmdQty 8

typedef enum {
  BUS_TEST=0,
  SERVO_STATE=1,
  SERVO_SET_POS=2,
  SERVO_SET_STATE=3,
  SERVO_PROT_SET=4,
  SERVO_PROT_GET=5,
  ACCEL_GIRO_GET=6,
  FORCE_GET=7,
}RbSerCmdName_t; 


const RbSerCmdDesc_t RbSerCmdDesc[RbSer_CmdQty]=
{
  {.Code=BUS_TEST,              .M_Len=0,       .S_Len=4},      //BUS_TEST
  {.Code=SERVO_STATE,           .M_Len=1,       .S_Len=5},      //SERVO_STATE
  {.Code=SERVO_SET_POS,         .M_Len=6,       .S_Len=2},      //SERVO_SET_POS
  {.Code=SERVO_SET_STATE,       .M_Len=3,       .S_Len=2},      //SERVO_SET_STATE
  {.Code=SERVO_PROT_SET,        .M_Len=11,      .S_Len=2},      //SERVO_PROT_SET
  {.Code=SERVO_PROT_GET,        .M_Len=1,       .S_Len=14},     //SERVO_PROT_GET
  {.Code=ACCEL_GIRO_GET,        .M_Len=0,       .S_Len=12},     //ACCEL_GIRO_GET
  {.Code=FORCE_GET,             .M_Len=0,       .S_Len=12},     //FORSE_GET
};

uint8_t  RbSer_rxstage=0;                 //Stage of parse command
uint16_t RbSer_Count=0;                   //Qty of unprocessed bytes in FIFO
uint16_t RbSer_Head=0;                    //Head index for FIFO (empty cell index)
uint16_t RbSer_Tail=0;                    //Tail index for FIFO (earliest full cell index)
uint16_t RbSer_tempTail=0;                //Parsing cell index
uint16_t RbSer_dataIndex=0;               //Received data index
uint16_t RbSer_crc_test;                  //Received crc
#if !RbSerMaster
uint8_t  RbSer_code_res=0;                //Received command code                 
#endif

uint16_t crc16_calc=0; // calculated CRC16

/*  Private  functions */

// Reset CRC16 calculation
void CRC16_ini(){       
  crc16_calc=0xFFFF; 
}

// Process one byte in CRC16 calculation
void CRC16_proc(uint8_t Data){            
crc16_calc^= Data<<8;
for (uint8_t i = 0; i < 8; i++){
    crc16_calc = crc16_calc & 0x8000 ? (crc16_calc << 1) ^ 0x1021 : crc16_calc << 1;
    }
}

// Transmit command
uint16_t Transmit_Cmd(uint8_t code){
    //Command head (no CRC test)
    Message_buff[0]=HEAD0;
    Message_buff[1]=HEAD1;
    Message_buff[2]=HEAD2;
    //Command code
    CRC16_ini();
    Message_buff[3]=code;
    CRC16_proc(Message_buff[3]);
    #if RbSerMaster
      Message_buff[4]=RbSerCmdDesc[code].M_Len;  //Master Tx data lengh
    #else
      Message_buff[4]=RbSerCmdDesc[code].S_Len;  //Slave Tx data lengh
    #endif 
    CRC16_proc(Message_buff[4]);
    uint16_t Index=5;
    //Command data
    uint8_t len=Message_buff[4];
    uint8_t i=0;
    while(len){
        Message_buff[Index]=data[i];
        CRC16_proc(Message_buff[Index]);
        Index++;
        i++;
        len--;
    }
    //Command CRC 
    Message_buff[Index]=(crc16_calc>>8)&0xFF;
    Index++;
    Message_buff[Index]=crc16_calc&0xFF; 
    Index++;
    #if RbSerMaster
    MaserLogAddLine(Message_buff,Index);
    #endif
    //Transmit
    return RbSerial_send_Buff(Message_buff,Index)==Index?1:0; //Transmit byte buffer for master
}

// Receive command
#if RbSerMaster
uint8_t Receive_Cmd(uint8_t code)           //Master receive only defined command
#else
uint8_t Receive_Cmd()                       //Slave receive all valid commands                     
#endif  
{
    uint8_t  RbSer_rxdat=0;                 //Received data
    #if RbSerMaster
    uint8_t  RbSer_TO=6;                    //Timeout for master
    while(RbSer_TO--)
    #endif
    {
        //Add data to FIFO
        #if RbSerMaster
        uint8_t First=1;
        #endif 
        while(RbSerial_receive(&RbSer_rxdat)){
            Message_buff[RbSer_Head]=RbSer_rxdat;
            #if RbSerMaster
            if(First){
                First=0;
                MaserLogAddFrstRx(Message_buff[RbSer_Head]);
            }
            else{
                MaserLogAddNextRx(Message_buff[RbSer_Head]);
            }
            #endif
            RbSer_Head++;
            if(RbSer_Head>=MSG_BUFF_SZ)
            {RbSer_Head=0;}
            RbSer_Count++;
            if(RbSer_Count>MSG_BUFF_SZ)
            {RbSer_Count=MSG_BUFF_SZ;}            

        }
        //Parse FIFO
        while(RbSer_Count){ 
            
            switch(RbSer_rxstage){
                case 1:{ //Test command head 1 (0 in default case)
                    if(Message_buff[RbSer_tempTail]==HEAD1){
                      RbSer_rxstage++; 
                    }
                    else{
                      RbSer_rxstage=0; 
                    }
                break;}
                case 2:{ //Test command head 2
                    if(Message_buff[RbSer_tempTail]==HEAD2){
                      RbSer_rxstage++; 
                    }
                    else{
                      RbSer_rxstage=0; 
                    }
                break;}
                case 3:{ //Test command code
                    #if RbSerMaster
                    if(Message_buff[RbSer_tempTail]==code)
                    #else
                    if(Message_buff[RbSer_tempTail]<RbSer_CmdQty)
                    #endif  
                    {
                        #if !RbSerMaster
                        RbSer_code_res=Message_buff[RbSer_tempTail];
                        #endif
                        CRC16_proc(Message_buff[RbSer_tempTail]);
                        RbSer_rxstage++; 
                    }
                    else{
                      RbSer_rxstage=0;
                    } 
                break;}
                case 4:{ //Test command len
                    #if RbSerMaster
                    if(Message_buff[RbSer_tempTail]==RbSerCmdDesc[code].S_Len)
                    #else
                    if(Message_buff[RbSer_tempTail]==RbSerCmdDesc[RbSer_code_res].M_Len)
                    #endif 
                    {
                        CRC16_proc(Message_buff[RbSer_tempTail]);
                        RbSer_dataIndex=0;
                        RbSer_rxstage++;
                    }
                    else{
                        RbSer_rxstage=0;
                    }
                break;}
                case 5:{ //Receive data
                    #if RbSerMaster
                    if(RbSer_dataIndex<RbSerCmdDesc[code].S_Len)
                    #else
                    if(RbSer_dataIndex<RbSerCmdDesc[RbSer_code_res].M_Len)
                    #endif 
                    {
                      data[RbSer_dataIndex]=Message_buff[RbSer_tempTail];
                      CRC16_proc(Message_buff[RbSer_tempTail]);
                      RbSer_dataIndex++;
                    }
                    else{
                      RbSer_crc_test=Message_buff[RbSer_tempTail]<<8;
                      RbSer_rxstage++;
                    }
                break;}
                case 6:{ //Resive CRC low and test CRC
                    RbSer_crc_test+=Message_buff[RbSer_tempTail];
                    if(crc16_calc==RbSer_crc_test){
                      RbSer_rxstage=0;
                      #if !RbSerMaster
                      RbSer_callback(RbSer_code_res);
                      Transmit_Cmd(RbSer_code_res);
                      #endif
                      return 1;
                    }
                    RbSer_rxstage=0;
                break;}
                default:{ //Test command head 0
                    RbSer_rxstage=0;
                    RbSer_tempTail=RbSer_Tail;
                    RbSer_Tail++;
                    if(RbSer_Tail>=MSG_BUFF_SZ){
                      RbSer_Tail=0;
                    }
                    if(RbSer_Count){
                      RbSer_Count--;
                    }
                    if(Message_buff[RbSer_tempTail]==HEAD0){
                      RbSer_rxstage=1; 
                      CRC16_ini();
                    }
                }
            }
            //Update tail index
            RbSer_tempTail++;
            if(RbSer_tempTail>=MSG_BUFF_SZ){
              RbSer_tempTail=0;                   
            }
        }
        #if RbSerMaster
        RbSerial_Delay(20);  //Timeout for master
        #endif
    }
    return 0; //Command not resived
}


#if RbSerMaster

/*  Master (Orange Pi / PC ) Functions                  */
#define Cmd_Tx(x) Transmit_Cmd(x)
#define Cmd_Rx(x) Receive_Cmd(x)

int RbSer_bus_test(uint16_t *Bat,int8_t *v5V1,int8_t *v5V2){
    if(Cmd_Tx(BUS_TEST)){
        if(Cmd_Rx(BUS_TEST)){
            uint16_t Temp=data[1];
            Temp<<=8;
            Temp+=data[0];
            *Bat=Temp;
            *v5V1=(int8_t)data[2]; 
            *v5V2=(int8_t)data[3];
            return 1;
        }
    }
    return 0;
}

int RbSer_Servo_State(uint8_t ID, uint8_t* State, uint8_t* On, uint16_t* Pos){
    data[0]=ID; 
    if(Cmd_Tx(SERVO_STATE)){
        if(Cmd_Rx(SERVO_STATE)){
            *On=data[1];
            *State=data[2];
            uint16_t Temp=data[4];
            Temp<<=8;
            Temp+=data[3];
            *Pos=Temp;
            return 1;
        }
    }
    *State=0xFF;
    return 0;
}

int RbSer_Servo_Set_Pos(uint8_t ID, uint16_t Pos,uint16_t Time, uint8_t Wait, uint8_t *Status){
    data[0]=ID; 
    if(Pos>1000){Pos=1000;}
    data[2]=Pos>>8;     data[1]=Pos&0xFF;
    if(Time<200){Time=200;}
    if(Time>30000){Time=30000;}
    data[4]=Time>>8;    data[3]=Time&0xFF;
    data[5]=Wait;
    if(Cmd_Tx(SERVO_SET_POS)){
        if(Cmd_Rx(SERVO_SET_POS)){
            *Status=data[1];
            return 1;
        }
    }
    *Status=0xFF;
    return 0;
}

int RbSer_Servo_Set_State(uint8_t ID, uint8_t Start, uint8_t On, uint8_t* Status){
    data[0]=ID; data[1]=On; data[2]=Start;
    if(Cmd_Tx(SERVO_SET_STATE)){
        if(Cmd_Rx(SERVO_SET_STATE)){
            *Status=data[1];
            return 1;
        }
    }
    *Status=0xFF;
    return 0;
}

int RbSer_Servo_Prot_Set(uint8_t ID, uint16_t Min, uint16_t Max, int8_t Adj, uint8_t Tmax_Deg, uint16_t Vmin_mV, uint16_t Vmax_mV,  uint8_t *Status){
    data[0]=ID; 
    
    if(Min>1000){Min=1000;}
    if(Max>1000){Max=1000;}
    if(Min>Max){Min=0; Max=1000;}
    data[1]=Min&0xFF; 
    Min>>=8; 
    data[2]=Min&0xFF; 
    data[3]=Max&0xFF; 
    Max>>=8; 
    data[4]=Max&0xFF; 
    
    if(Adj<-125){Adj=-125;}
    if(Adj>+125){Adj=+125;}
    data[5]=Adj;
    
    if(Tmax_Deg>100){Tmax_Deg=100;}
    if(Tmax_Deg<50){Tmax_Deg=50;}
    data[6]=Tmax_Deg;  
    
    if(Vmin_mV>7000){Vmin_mV=7000;}
    if(Vmin_mV<6000){Vmin_mV=6000;}
    if(Vmax_mV>9000){Vmax_mV=9000;}
    if(Vmax_mV<7500){Vmax_mV=7500;}  
    data[7]=Vmin_mV&0xFF;
    Vmin_mV>>=8;
    data[8]=Vmin_mV&0xFF;    
    data[9]=Vmax_mV&0xFF;
    Vmax_mV>>=8;
    data[10]=Vmax_mV&0xFF;
 
    if(Cmd_Tx(SERVO_PROT_SET)){
        if(Cmd_Rx(SERVO_PROT_SET)){
            *Status=data[1];
            return 1;
        }
    }
    *Status=0xFF;
    return 0;
}

int RbSer_Servo_Prot_Get(uint8_t ID, uint16_t *Min, uint16_t *Max, int8_t *Adj, uint8_t *Tmax_Deg, uint8_t *T_Deg, uint16_t *Vmin_mV, uint16_t *Vmax_mV, uint16_t *V_mV){
    data[0]=ID; 
    if(Cmd_Tx(SERVO_PROT_GET)){
        if(Cmd_Rx(SERVO_PROT_GET)){
            *Min=data[2];  *Min<<=8; *Min+=data[1];
            *Max=data[4];  *Max<<=8; *Max+=data[3];
            *Adj=data[5];
            *Tmax_Deg=data[6];
            *Vmin_mV=data[8];  *Vmin_mV<<=8; *Vmin_mV+=data[7];
            *Vmax_mV=data[10]; *Vmax_mV<<=8; *Vmax_mV+=data[9];
            *T_Deg=data[11];
            *V_mV=data[13];  *V_mV<<=8; *V_mV+=data[12];
            return 1;
        }
    }
    return 0;
}

int RbSer_Accel_Giro_Get(int16_t *Accel, int16_t *Giro){
    if(Cmd_Tx(ACCEL_GIRO_GET)){
        if(Cmd_Rx(ACCEL_GIRO_GET)){
          uint8_t low=0; 
          uint8_t high=1;
          for(uint8_t i=0;i<3;i++){
            Accel[i]=data[high]; Accel[i]<<=8; Accel[i]+=data[low];
            high+=2; low+=2;
          }
          for(uint8_t i=0;i<3;i++){
            Giro[i]=data[high]; Giro[i]<<=8; Giro[i]+=data[low];
            high+=2; low+=2;
          }
          return 1;
        }
    }
    return 0;
}

int RbSer_Force_Get(uint16_t *Force){
    if(Cmd_Tx(FORCE_GET)){
        if(Cmd_Rx(FORCE_GET)){
          uint8_t low=0; 
          uint8_t high=1;
          for(uint8_t i=0;i<6;i++){
            Force[i]=data[high]; Force[i]<<=8; Force[i]+=data[low];
            high+=2; low+=2;
          }
          return 1;
        }
    }
    return 0;
}
#endif //RbSerMaster>0

/*  Public slave (STM32) functions */

#if RbSerMaster==0

void RbSer_callback(uint8_t Code){
  switch(Code){
    case BUS_TEST:{
    Bat_Test(&data[0]);
    break;}
    case SERVO_STATE:{
    Servo_State(&data[0]);
    break;}
    case SERVO_SET_POS:{
    Servo_Set_Pos(&data[0]);
    break;}
    case SERVO_SET_STATE:{
    Servo_Set_State(&data[0]);
    break;}
    case SERVO_PROT_SET:{
    Servo_Prot_Set(&data[0]);
    break;}
    case SERVO_PROT_GET:{
    Servo_Prot_Get(&data[0]);
    break;} 
    case ACCEL_GIRO_GET:{
    Accel_Giro_Get(&data[0]);
    break;} 
    case FORCE_GET:{
    Force_Get(&data[0]);
    break;} 
  }
}

#endif 

