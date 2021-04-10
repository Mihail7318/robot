/*
 * main.h
 *
 *  Created on: Apr 27, 2018
 *      Author: john
 */
/*
  Configuration for cross platform (Linux/Windows) protocol functions tester CUI
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define portbaund  115200  //bit/sec port speed
#define portformat "8N1/0" //8 bit data / no parity / 1 stop bit

#define RbSerMaster 1 //Orange Pi and PC - master (=1), MCU device - slave (=0)
#define MasterLog   1 //Allow make log file "RobotPrLog.txt" in binary dir


#if defined(__linux__)
#include <termios.h>
#define getch()  getchar()
#define version_string "0.3(Linux)"
#define TimeStampAbsolute 0 // Time stamp format 0 - time from start / >0 - local time
#else
#define version_string "0.3(Windows)"
#endif

//Low level terminal functions only for test mode
int RbSerial_open_port();
void RbSerial_print_port();
void RbSerial_Delay(uint16_t ms);


#endif /* MAIN_H_ */
