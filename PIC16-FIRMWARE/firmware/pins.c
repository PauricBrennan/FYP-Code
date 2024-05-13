/**
 * Generated Driver File
 * 
 * @file pins.c
 * 
 * @ingroup  pinsdriver
 * 
 * @brief This is generated driver implementation for pins. 
 *        This file provides implementations for pin APIs for all pins selected in the GUI.
 *
 * @version Driver Version 3.0.0
*/

#include "pins.h"


void PINS_Init(void){
   /**
    LATx registers
    */
    LATA = 0x0;
    LATB = 0x2;
    LATC = 0x18;

    /**
    TRISx registers
    */
    TRISA = 0xF0;
    TRISB = 0xFC;
    TRISC = 0xFF;

    /**
    ANSELx registers
    */
    ANSELA = 0xF0;
    ANSELB = 0xFD;
    ANSELC = 0xE5;

    /**
    WPUx registers
    */
    WPUA = 0x0;
    WPUB = 0x0;
    WPUC = 0x0;
    WPUE = 0x0;
  
    /**
    ODx registers
    */
   
    ODCONA = 0x0;
    ODCONB = 0x0;
    ODCONC = 0x0;
    /**
    SLRCONx registers
    */
    SLRCONA = 0xFF;
    SLRCONB = 0xFF;
    SLRCONC = 0xFF;
    /**
    INLVLx registers
    */
    INLVLA = 0xFF;
    INLVLB = 0xFF;
    INLVLC = 0xFF;
    INLVLE = 0x8;

    /**
    PPS registers
    */
    SMT1SIGPPS = 0x11; //RC1->SMT1:SMT1SIG;
    RB0PPS = 0x05;  //RB0->CWG1:CWG1A;
    SSP1CLKPPS = 0x13;  //RC3->MSSP1:SCL1;
    RC3PPS = 0x14;  //RC3->MSSP1:SCL1;
    SSP1DATPPS = 0x14;  //RC4->MSSP1:SDA1;
    RC4PPS = 0x15;  //RC4->MSSP1:SDA1;

    /**
    APFCON registers
    */

   /**
    IOCx registers 
    */
    IOCAP = 0x0;
    IOCAN = 0x0;
    IOCAF = 0x0;
    IOCBP = 0x0;
    IOCBN = 0x0;
    IOCBF = 0x0;
    IOCCP = 0x0;
    IOCCN = 0x0;
    IOCCF = 0x0;
    IOCEP = 0x0;
    IOCEN = 0x0;
    IOCEF = 0x0;


}
  
void LED_SetHigh(int led){
    switch(led){
        case RED_LED:         LATAbits.LATA1 = 1; break;
        case GREEN_LED:       LATAbits.LATA0 = 1; break;
        case YELLOWR_LED:     LATAbits.LATA2 = 1; break;
        case YELLOWL_LED:     LATAbits.LATA3 = 1; break;
    }
}


void LED_SetLow(int led){
    switch(led){
        case RED_LED:         LATAbits.LATA1 = 0; break;
        case GREEN_LED:       LATAbits.LATA0 = 0; break;
        case YELLOWR_LED:     LATAbits.LATA2 = 0; break;
        case YELLOWL_LED:     LATAbits.LATA3 = 0; break;
    }
}

void LED_Toggle(int led){
    switch(led){
        case RED_LED:         LATAbits.LATA1 = ~LATAbits.LATA1; break;
        case GREEN_LED:       LATAbits.LATA0 = ~LATAbits.LATA0; break;
        case YELLOWR_LED:     LATAbits.LATA2 = ~LATAbits.LATA2; break;
        case YELLOWL_LED:     LATAbits.LATA3 = ~LATAbits.LATA3; break;
    }
}

void PWM_MotorFoward(void){
    RB0PPS = 0x00;
    RB1PPS = 0x05;
    LATBbits.LATB0 = 1;
    
}


void PWM_MotorReverse(void){
    RB0PPS = 0x05;
    RB1PPS = 0x00;
    LATBbits.LATB1 = 1;
}