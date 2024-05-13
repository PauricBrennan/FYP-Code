 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/

#include "system.h"

/*
    Main application
*/
void delay(int);

int main(void) {
    SYSTEM_Init();

    // Enable the Global Interrupts 
    GlobalInterruptEnable(); 

    // Enable the Peripheral Interrupts 
    PeripheralInterruptEnable(); 

    LED_SetHigh(RED_LED);
    LED_SetHigh(GREEN_LED);
    LED_SetHigh(YELLOWR_LED);
    LED_SetHigh(YELLOWL_LED);
    delay(20);
    LED_SetLow(RED_LED);
    LED_SetLow(GREEN_LED);
    LED_SetLow(YELLOWR_LED);
    LED_SetLow(YELLOWL_LED);
    while(1) {

        
    }    
}

void delay(int n){
    int i;
    for(i = 0; i < n; i++){
        __delay_ms(100);
    }
}