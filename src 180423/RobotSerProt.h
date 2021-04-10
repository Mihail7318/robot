#ifndef _RobotSerProt_H_
#define _RobotSerProt_H_

/*!
    \file RobotSerProt.h
    \brief Header file with declaration of public protocol function 
*/

#include <stdint.h>

#ifndef RbSerMaster
/// Role of device Master - 1 / Slave - 0
#define  RbSerMaster 1 
#warning "RobotSerProt.h: Role not defined - selected master"
#endif

#if RbSerMaster>0
/*! \defgroup Protocol functions for master (Orange Pi / PC) 
    @{
*/

/*! Master bus test (status of slave)
 * \param[out] Bat - pointer, bat voltage in mV 
 * \param[out] v5V1 - pointer, 5V supply Orange Pi state: 0: [4.75V,5.25V], -127:[0, 4.75V) , +128:(5.25V,5.5V]
 * \param[out] v5V2 - pointer, 5V supply external devices state: 0: [4.75V,5.25V], -127:[0, 4.75V) , +128:(5.25V,5.5V]
 * \return - 1 slave process command,  0 no data
*/
int RbSer_bus_test(uint16_t *Bat,int8_t *v5V1,int8_t *v5V2);

/*! \defgroup Servo
     \ingroup Protocol functions for master (Orange Pi / PC)
    @{
*/

/*! Master servo state get
 * \param[in] ID - servo ID
 * \param[out] State - pointer, state flags (if 0xFF - not connected): bit 0: 1 - temperature OK (0 - over temperature), 
 * bit 1: 1 - voltage OK (0 - over voltage), bit 2: 1 - unblocked rotor (0 - blocked rotor).
 * \param[out] On - pointer, motor ON/OFF: 0 - OFF (servo has no torque output), 1 - ON (servo has torque output)
 * \param[out] Pos - pointer, position of servo range [0,1000]. corresponding to the servo angle of [0,240]
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Servo_State(uint8_t ID, uint8_t* State, uint8_t* On, uint16_t* Pos);

/*! Master servo set position
 * \param[in] ID - servo ID
 * \param[in] Pos - position of servo range [0,1000]. corresponding to the servo angle of [0,240]
 * \param[in] Time - time for set in new position in ms (0,30000]
 * \param[in] Wait - select move type, 0 - wait for start command, 1 - start move immediately
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Servo_Set_Pos(uint8_t ID, uint16_t Pos,uint16_t Time, uint8_t Wait, uint8_t *Status);

/*! Master servo set state
 * \param[in] ID - servo ID
 * \param[in] Start - start/stop move: 0- stop, 1 - start
 * \param[in] On - motor ON/OFF: 0 - OFF (servo has no torque output), 1 - ON (servo has torque output)
 * \param[out] Status - pointer, 0xFF - servo not connected, 0x00 - servo OK
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Servo_Set_State(uint8_t ID, uint8_t Start, uint8_t On, uint8_t* Status);

/*! Master servo set params
 * \param[in] ID - servo ID
 * \param[in] Min - minimum position of servo range [0,1000]. corresponding to the servo angle of [0,240]
 * \param[in] Max - maximum position of servo range [0,1000]. corresponding to the servo angle of [0,240]
 * \param[in] Adj - adjustment position of servo range [-125,+125]. corresponding to the servo angle of [-30,+30]
 * \param[in] Tmax_Deg - maximum working temperature in range [50,100] deg.
 * \param[in] Vmin_mV - minimum servo voltage in range [6000,7000] mV
 * \param[in] Vmax_mV - maximum servo voltage in range [7500,9000] mV
 * \param[out] Status - pointer, 0xFF - servo not connected, 0x00 - servo OK
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Servo_Prot_Set(uint8_t ID, uint16_t Min, uint16_t Max, int8_t Adj, uint8_t Tmax_Deg, uint16_t Vmin_mV, uint16_t Vmax_mV,  uint8_t *Status);

/*! Master servo set params
 * \param[in] ID - servo ID
 * \param[in] Min - pointer,minimum position of servo range [0,1000]. corresponding to the servo angle of [0,240]
 * \param[in] Max - pointer,maximum position of servo range [0,1000]. corresponding to the servo angle of [0,240]
 * \param[in] Adj - pointer,adjustment position of servo range [-125,+125]. corresponding to the servo angle of [-30,+30]
 * \param[in] Tmax_Deg - pointer,maximum working temperature in range [50,100] deg.
 * \param[in] T_Deg - pointer, current temperature inside the servo deg.
 * \param[in] Vmin_mV - pointer,minimum servo voltage in range [6000,7000] mV
 * \param[in] Vmax_mV - pointer,maximum servo voltage in range [7500,9000] mV
 * \param[in] V_mV - pointer,current input voltage value mV
 * \param[out] Status - pointer, 0xFF - servo not connected, 0x00 - servo OK
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Servo_Prot_Get(uint8_t ID, uint16_t *Min, uint16_t *Max, int8_t *Adj, uint8_t *Tmax_Deg, uint8_t *T_Deg, uint16_t *Vmin_mV, uint16_t *Vmax_mV, uint16_t *V_mV);

/*! @} */

/*! \defgroup Sensors
     \ingroup Protocol functions for master (Orange Pi / PC)
    @{
*/

/*! Accelerometer / gyroscope sensors get 
 * \param[out] Accel - pointer to an array X,Y,Z acceleration (signed) [-2g,2g] in 0.001g
 * \param[out] Giro - pointer to an array X,Y,Z rotate speed (signed) [-250dps,250dps] in 0.01dps
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Accel_Giro_Get(int16_t *Accel, int16_t *Giro);

/*! Force sensors get 
 * \param[out] Forse - pointer to an array of 6 force values (unsigned) [0,50N] in 0.01N
 * \return - 1 slave process command,  0 no data
*/
int RbSer_Force_Get(uint16_t *Force);

/*! @} */

/*! @} */

/*! \defgroup Platform specific functions for master (Orange Pi / PC) 
    @{
*/

/*! Master receive 1 byte from serial port
    \param[out] data - received byte pointer
    \return - 1 if read data, 0 if no data
*/
uint8_t RbSerial_receive(uint8_t* data);  


/*! Master transmit buffer to serial port
    \param[in] data - buffer pointer
    \param[in] len - buffer lengh
    \return - number of sended bytes
*/
uint16_t RbSerial_send_Buff(uint8_t* data, uint16_t len);

/*! Master context switch while waiting data
    \param[in] ms - ticks for wait in ms
*/
void RbSerial_Delay(uint16_t ms);
///Add TX data to log
void MaserLogAddLine(uint8_t* data, uint8_t Len);
///Add Frst byte in Rx message
void MaserLogAddFrstRx(uint8_t data);
///Add Next byte in Rx message
void MaserLogAddNextRx(uint8_t data);

/*! @} */

#else //RbSerMaster>0

/*! Slave process protocol
    \return - 1 if valid command received, 0 - if not.
*/
uint8_t Receive_Cmd();


/*! \defgroup Platform specific functions for slave (STM32) 
    @{
*/

/// \brief   Slave port number
/// \details 0- SER1 (X16 connector), 1- Orange Pi port (X2 connector), 2 - SER2 (X17 connector)
extern uint8_t RbSerial_Port_No;  

/*! Slave receive 1 byte from serial port
    \param[out] dat - received byte pointer
    \return - 1 if read data, 0 if no data
*/
#define RbSerial_receive(dat)  Serial_receive((dat),RbSerial_Port_No) 


/*! Slave transmit buffer to serial port
    \param[in] dat - buffer pointer
    \param[in] ln - buffer lengh
    \return - number of sended bytes
*/
#define RbSerial_send_Buff(dat,ln) Serial_send_Buff((dat),(ln),RbSerial_Port_No)

/*! Slave context switch while waiting data
    \param[in] ms - ticks for wait in ms
*/
#define RbSerial_Delay(ms) vTaskDelay((ms));

/*! @} */

#endif //RobotSerProt_H_

#endif //RobotSerProt_H_
