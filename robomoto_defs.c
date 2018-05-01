/*
 * File:   motors_defs.c
 * Author: Grant
 *
 * Created on April 5, 2018, 12:18 PM
 */

#include "xc.h"
#include "p24Fxxxx.h"
#include "robot.h"

//void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void){
//    _T2IF = 0;
//
//}
//void setupTMR2(void){
//    //Create a one second timer
//    T2CONbits.TON = 0;
//    TMR2 = 0;
//    T2CONbits.TCKPS0 = 1;
//    T2CONbits.TCKPS1 = 1;
//    PR2 = ONE_SECOND;
//    _T2IF = 0;
//    //_T2IP = 0b010;
//    _T2IE = 1;
//    T2CONbits.TON = 1;
//}

void setup_motors(void){    
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;
    I2C2CONbits.I2CEN = 0;
    TRISBbits.TRISB4 = 0;
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB9 = 0;
    TRISAbits.TRISA2 = 0;
    TRISBbits.TRISB3 = 0;
    //All 1's will disengage the motor, only goes to the side with low voltage
    PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
    PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB3 = 1; //Right motor backward
    PORTAbits.RA2 = 1; //Right motor forward , Should be RB2 in the final version
    //setupTMR2();
}

void forward(void){
    PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
    PORTAbits.RA4 = 0; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB3 = 1; //Right motor backward
    PORTAbits.RA2 = 0; //Right motor forward , Should be RB2 in the final version
}

void reverse(void){
    PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version
    PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB3 = 0; //Right motor Reverse
    PORTAbits.RA2 = 1; //Right motor Forward , Should be RB2 in the final version
}

void left(void){
    PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version
    PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB3 = 1; //Right motor Reverse
    PORTAbits.RA2 = 0; //Right motor forward , Should be RB2 in the final version
}

void right(void){
    PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
    PORTAbits.RA4 = 0; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB3 = 0; //Right motor backward
    PORTAbits.RA2 = 1; //Right motor forward , Should be RB2 in the final version
}

void stop(void){
    PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
    PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB3 = 1; //Right motor backward
    PORTAbits.RA2 = 1; //Right motor forward , Should be RB2 in the final version
}