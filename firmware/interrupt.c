/**
***  This is the interrupt C file defining all relevant functions 
***  for handling enabled interrupts. The interrupts triggered by the PIC16F18855's
***  SMT and I2C modules are handled here for measuring the motor encoders output signal 
***  and interpreting commands from the raspberry pi 4. Code was Developed with the help of
***  MPLAB MCC code generator. 
*** 
***  Developer: Pauric Brennan
*** 
*/
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "interrupt.h"
#include "system.h"

static volatile uint16_t i2c_address;
static volatile uint8_t smt_buffer[3] = {0,0,0};  
static volatile uint16_t pwm = 0;
static volatile uint32_t period = 0;

/**
 * 
 */
void __interrupt() ISR (void){
    if(INTCONbits.PEIE == 1) {
        if(PIE8bits.SMT1PRAIE == 1 && PIR8bits.SMT1PRAIF == 1) {
            SMT_PR_ACQ_ISR();
        
        } else if(PIE8bits.SMT1IE == 1 && PIR8bits.SMT1IF == 1) {
            SMT_ISR();
        } else if(PIE3bits.BCL1IE == 1 && PIR3bits.BCL1IF == 1) {
            I2C_ErrorHandler();
            
        } else if(PIE3bits.SSP1IE == 1 && PIR3bits.SSP1IF == 1) {
            I2C_EventHandler();
        } 
    }      
}

/**
 *  Enables global interrupts  
*/
void GlobalInterruptEnable(void){
    INTCONbits.GIE = 1;
}


/**
 *  Disables global interrupts
*/
void GlobalInterruptDisable(void){
    INTCONbits.GIE = 0;
}


/**
 *  Enables peripheral interrupts.
*/
void PeripheralInterruptEnable(void){
    INTCONbits.PEIE = 1;
}

/**
 *  Disables peripheral interrupts.
*/
void PeripheralInterruptDisable(void){
    INTCONbits.PEIE = 0;
}

/**
 * ==================== I2C INTERRUPT FUNCTIONS ====================
 */


static bool EventFunctions(i2c_events event){

    switch(event){
        case I2C_STOP_BIT_EVENT:                                     return true; break;
        case I2C_ADDR_MATCH_EVENT:                                   return true; break; 
        case I2C_TX_EVENT:                     tx();                 return true; break;
        case I2C_RX_EVENT:                     rx();                 return true; break;
        case I2C_BUS_COLLISION_ERROR_EVENT:    LED_Toggle(RED_LED);               break;
        case I2C_WRITE_COLLISION_ERROR_EVENT:  LED_Toggle(RED_LED);               break;
    }
    return false;
}

/**
 *  This function called when an I2C interrupt is triggered, it handles
 *  all of the possible defined events that can occur with respect to the 
 *  I2C module.  
*/
static void I2C_EventHandler(void){
    PIR3bits.SSP1IF = 0;
    
    // STOP EVENT HAS OCCURRED
    if (SSP1STATbits.P) EventFunctions(I2C_STOP_BIT_EVENT); 
        
    // I2C ADDRESS RECIEVED    
    else if (!SSP1STATbits.D_nA) AddressRecievedFunction();
    
    // CHECK IF TX OR RX EVENT OCCURRED (PIC Transmit or Recieve)
    else CheckHostReadWriteFunction();

    // Release the clock stretch 
    SSP1CON1bits.CKP = 1;
}

static void I2C_ErrorHandler(void){
    // BUS COLLISION OCCURRED
    if (PIR3bits.BCL1IF){
        EventFunctions(I2C_BUS_COLLISION_ERROR_EVENT);
        PIR3bits.BCL1IF = 0; /* Clear the Bus collision */
        
    // WRITE COLLISION OCCURRED    
    } else if (SSP1CON1bits.WCOL) {
        EventFunctions(I2C_WRITE_COLLISION_ERROR_EVENT);
        SSP1CON1bits.WCOL = 0; /* Clear the Write collision */
    }
    
    /* Data written by the application; release the clock stretch */
    SSP1CON1bits.CKP = 1;
}

static void AddressRecievedFunction (void) {
    i2c_address = I2C_ReadByte();             // Recieved address must be read out
        
    // ADDRESS MATCH EVENT OCCURRED
    if (EventFunctions(I2C_ADDR_MATCH_EVENT) == true) {
        
        // TX EVENT OCCURRED - HOST WANTS TO READ
        if ( (SSP1STATbits.R_nW && !SSP1STATbits.BF) && !EventFunctions(I2C_TX_EVENT)) SendNack();

        SendAck();  // Send Ack for address match event (TX event as well if evaluated as true) 
    } else {
        SendNack();
    }
}

static void CheckHostReadWriteFunction (void) {
    // TX EVENT OCCURRED - HOST WANTS TO READ
    if (SSP1STATbits.R_nW && (!SSP1STATbits.BF) && (!SSP1CON2bits.ACKDT)){
        if (EventFunctions(I2C_TX_EVENT) == false) SendNack();

    // OTHERWISE RX EVENT OCCURRED - HOST WANTS TO WRITE    
    } else if (SSP1STATbits.BF){
        if (EventFunctions(I2C_RX_EVENT) == true) {
            SendAck();
        } else {
            SendNack();
        }
    }
        
}

static void tx(void){
   static uint8_t byte_number = 0;
   
   if(byte_number == 0){
       //SMT_DataAcquisitionEnable();
       
       SMT_GetCapturedPeriod();      
   } 
   
   switch(byte_number){
       case 0: I2C_WriteByte(smt_buffer[0]); byte_number++; break;
       case 1: I2C_WriteByte(smt_buffer[1]); byte_number++; break;
       case 2: I2C_WriteByte(smt_buffer[2]); byte_number = 0; break;   
   }
}

static void rx(void){
   static uint8_t byte_number = 0;
   uint16_t value = 255;
   uint16_t command_mask = 0xf000;
   
   switch(byte_number){
       case 0: pwm = pwm | I2C_ReadByte(); byte_number++; break;
       case 1: pwm = pwm | ( (value & I2C_ReadByte()) << 8);
               //LED_Toggle(GREEN_LED);
               CommandHandler( (pwm & command_mask) >> 12 );
               PWM_LoadDutyValue(pwm); 
               pwm = 0; byte_number = 0; 
               break;
   }
}

static void CommandHandler(int command){
    switch(command){
        case I2C_NO_COMMAND:                                          break;
        case I2C_MOTOR_FORWARD_COMMAND:         PWM_MotorFoward();    break;
        case I2C_MOTOR_REVERSE_COMMAND:         PWM_MotorReverse();   break;
        //case I2C_PWM_POLARITY_INV_COMMAND:      PWM_InvertWave(1);    break;
        //case I2C_PWM_POLARITY_NOT_INV_COMMAND:  PWM_InvertWave(0);    break;
    }
}

static void SendAck(void) {
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
}

static void SendNack(void) {
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
}

static void I2C_WriteByte(uint8_t data){
    SSP1BUF = data;
}

static uint8_t I2C_ReadByte(void){
    return SSP1BUF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * ==================== SMT FUNCTIONS ====================
 */

static void SMT_DataAcquisitionEnable(void){
    // Start the SMT module by writing to SMTxGO bit
    SMT1CON1bits.SMT1GO = 1;
}

static void SMT_GetCapturedPeriod(void){
    //uint32_t overflow = 160000;
//    PIR8bits.SMT1PRAIF = 0;
//    SMT1STATbits.SMT1RST = 1;
//    SMT1CON1bits.SMT1GO = 1;
//    uint16_t i = 0;
//    __delay_ms(1);
    
    //while( PIR8bits.SMT1PRAIF == 0 && (SMT1STATbits.SMT1TS == 1) ) continue;
    
    //uint32_t period = (PIR8bits.SMT1PRAIF == 1) ? SMT1CPR : 0;
    
    //PIR8bits.SMT1PRAIF = 0;
    //PIR8bits.SMT1IF = 0;
    //SMT1CON1bits.SMT1GO = 0;
    //if(period > 100000) LED_SetHigh(GREEN_LED);   
    //period = 150000;
    smt_buffer[0] = period & 0xff;
    period = period >> 8;
       
    smt_buffer[1] = period & 0xff;
    period = period >> 8;       
       
    smt_buffer[2] = period & 0xff;
    //if(smt_buffer[2] > 10) LED_SetHigh(RED_LED);
    //else LED_SetLow(RED_LED);
    period = 0;
}

static void SMT_PR_ACQ_ISR(void){
    PIR8bits.SMT1PRAIF = 0;
    //if(SMT1CPR > 200000) LED_SetHigh(RED_LED);
    period = SMT1CPR;
    //if(period > 200000) LED_SetHigh(RED_LED);
    //SMT1STATbits.SMT1RST = 1;
}

static void SMT_ISR(void){
    //LED_Toggle(RED_LED);
    //if(SMT1STATbits.SMT1TS && SMT1STATbits.SMT1AS) LED_Toggle(YELLOWR_LED);
    //SMT1CON0bits.SMT1EN = 0;
    SMT1CON1bits.SMT1GO = 0;
    SMT1STATbits.SMT1RST = 1;
    PIR8bits.SMT1IF = 0;
    PIR8bits.SMT1PRAIF = 0;
    //SMT1CON0bits.SMT1EN = 1;
    SMT1CON1bits.SMT1GO = 1;
    period = 0;
}

///////////////////////////////////

static void PWM_LoadDutyValue(uint16_t dutyValue) {
     // Writing to 8 MSBs of PWM duty cycle in PWMDCH register
     PWM6DCH = (uint8_t) ((dutyValue & 0x03FCu) >> 2);
     
     // Writing to 2 LSBs of PWM duty cycle in PWMDCL register
     PWM6DCL = (uint8_t) ((dutyValue & 0x0003u) << 6);
 }