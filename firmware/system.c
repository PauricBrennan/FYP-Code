/**
 * System Driver Source File
 * 
 * @file system.c
 * 
 * @ingroup systemdriver
 * 
 * @brief This file contains the API implementation for the System driver.
 *
 * @version Driver Version 2.0.3
 *
 * @version Package Version 4.1.3
*/

#include "system.h"


void SYSTEM_Init(void)
{
    CLOCK_Initialize();
    PINS_Init();
    CWG_Init();
    I2C_Init();
    PWM_Init();
    SMT_Init();
    TMR2_Init();
}

void I2C_Init(void){
    /* CKE enabled; SMP Standard Speed;  */
    SSP1STAT = 0xE4;
    /* SSPM 7 Bit Polling; CKP disabled; SSPEN disabled; SSPOV no_overflow; WCOL no_collision;  */
    SSP1CON1 |= 0x6;
    /* SEN enabled; RSEN disabled; PEN disabled; RCEN disabled; ACKEN disabled; ACKDT acknowledge; GCEN disabled;  */
    SSP1CON2 = 0x1;
    /* DHEN disabled; AHEN disabled; SBCDE disabled; SDAHT 100ns; BOEN disabled; SCIE disabled; PCIE disabled;  */
    SSP1CON3 = 0x0;
    /* SSPADD 16;  */
    SSP1ADD = 0x10;
    /* SSPMSK 254;  */
    SSP1MSK = 0xFE;
    /* Enable Interrupts */
    PIE3bits.SSP1IE = 1;
    PIE3bits.BCL1IE = 1;
    SSP1CON3bits.PCIE = 1;
    SSP1CON1bits.SSPEN = 1;
}

void SMT_Init(void){

    // SMTEN enabled; SMTSTP rolls over to 0x000000; SMTWPOL high/rising edge enabled; SMTSPOL high/rising edge enabled; SMTCPOL rising edge; SMTPS 1:1 Prescaler; 
    SMT1CON0 = 0x80;
    
    // SMTGO disabled; SMTREPEAT Single Acquisition mode; SMTMODE Period and Duty-Cycle Acquisition; 
    SMT1CON1 = 0x42; 
    
    // SMTCPRUP SMT1CPR1 update complete; SMTCPWUP SMT1CPW1 update complete; SMTRST SMT1TMR1 update complete; 
    SMT1STAT = 0x0; 
    
    // SMTCSEL HFINTOSC 16 MHz; 
    SMT1CLK = 0x2; 
    
    // SMTWSEL SMT1WINPPS; 
    SMT1WIN = 0x0; 
    
    // SMTSSEL SMT1SIGPPS; 
    SMT1SIG = 0x0; 
    
    // SMTPRU 244; 
    SMT1PRU = 3; 
    
    // SMTPRH 36; 
    SMT1PRH = 13; 
    
    // SMTPRL 0; 
    SMT1PRL = 64; 

    //SMT1CON0bits.SMT1STP = 1;
    // Enabling SMT1 overflow interrupt.
    PIE8bits.SMT1IE = 1;

    // Enabling SMT1 pulse width acquisition interrupt.
    // PIE8bits.SMT1PWAIE = 1; 

    // Enabling SMT1 period acquisition interrupt.
    PIE8bits.SMT1PRAIE = 1; 
    SMT1STATbits.SMT1RST = 1;
    //SMT1CON1bits.SMT1REPEAT = 1;
    SMT1CON1bits.SMT1GO = 1;
    
    

}

 void PWM_Init(void) {
    // Set the PWM to the options selected in the
    
    // CTSEL PWM6timer2; 
    CCPTMRS1bits.P6TSEL = 0x1;
    
    PWM6DCH = (uint8_t) ((0 & 0x03FCu) >> 2);
    PWM6DCL = (uint8_t) ((0 & 0x0003u) << 6);

    // PWMPOL active_hi; PWMEN enabled; 
    PWM6CON = 0x80;
 }
 
 void TMR2_Init(void) {

    // Set TMR2 to the options selected in the User Interface
    // TCS FOSC/4; 
    T2CLKCON = 0x1;
    // TMODE Software control; TCKSYNC Not Synchronized; TCKPOL Rising Edge; TPSYNC Not Synchronized; 
    T2HLT = 0x0;
    // TRSEL T2CKIPPS pin; 
    T2RST = 0x0;
    // PR 219; 
    T2PR = 0xDB;
    // TMR 0x0; 
    T2TMR = 0x0;

    // Clearing IF flag.
    PIR4bits.TMR2IF = 0;
    // TCKPS 1:1; TMRON on; TOUTPS 1:1; 
    T2CON = 0x80;
}
 
 void CWG_Init(void) {
    // CWGCS FOSC; 
    CWG1CLKCON = 0x0;
    // CWGPOLA non inverted; CWGPOLB non inverted; CWGPOLC non inverted; CWGPOLD non inverted; 
    CWG1CON1 = 0x1;
    // CWGDBR 0; 
    CWG1DBR = 0x0;
    // CWGDBF 0; 
    CWG1DBF = 0x0;
    // CWGSHUTDOWN No Auto-shutdown; CWGREN disabled; CWGLSBD tri-stated; CWGLSAC tri-stated; 
    CWG1AS0 = 0x14;
    // AS0E disabled; AS1E disabled; AS2E disabled; AS3E disabled; AS4E disabled; AS5E disabled; AS6E disabled; 
    CWG1AS1 = 0x0;
    // CWGOVRD low; CWGOVRC low; CWGOVRB low; CWGOVRA low; CWGSTRD disabled; CWGSTRC disabled; CWGSTRB disabled; CWGSTRA enabled; 
    CWG1STR = 0x1;
    // CWGIS PWM6_OUT; 
    CWG1ISM = 0x6;
    /*CWGEN enabled; CWGMODE Half bridge mode; CWGLD Buffer_not_loaded; */
    CWG1CON0 = 0x84;
    
}