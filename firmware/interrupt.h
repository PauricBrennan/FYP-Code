/**
***  This is the interrupt header file declaring all relevant functions 
***  for handling enabled interrupts. The interrupts triggered by the PIC16F18855's
***  SMT and I2C modules are handled here for measuring the motor encoders output signal 
***  and interpreting commands from the raspberry pi 4. Code was Developed with the help of
***  MPLAB MCC code generator. 
*** 
***  Developer: Pauric Brennan
*** 
*/

#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <xc.h>

/**
 *  Enables global interrupts  
*/
void GlobalInterruptEnable(void);//(INTCONbits.GIE = 1)


/**
 *  Disables global interrupts
*/
void GlobalInterruptDisable(void);// (INTCONbits.GIE = 0)


/**
 *  Enables peripheral interrupts.
*/
void PeripheralInterruptEnable(void);// (INTCONbits.PEIE = 1)

/**
 *  Disables peripheral interrupts.
*/
void PeripheralInterruptDisable(void); //(INTCONbits.PEIE = 0)

static void SendAck(void);

static void SendNack(void);

static void CheckHostReadWriteFunction (void);

static void AddressRecievedFunction(void);

static void I2C_EventHandler(void);

static void I2C_ErrorHandler(void);

static void tx(void);

static void rx(void);

static void I2C_WriteByte(uint8_t data);

static uint8_t I2C_ReadByte(void);

static void SMT_DataAcquisitionEnable(void);

static void SMT_GetCapturedPeriod(void);

static void SMT_PR_ACQ_ISR(void);

static void SMT_ISR(void);

static void CommandHandler(int command);

static void PWM_LoadDutyValue(uint16_t dutyValue);

#endif  // INTERRUPT_H