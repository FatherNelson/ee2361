/*
 * File:   motors_defs.c
 * Author: Grant
 *
 * Created on April 5, 2018, 12:18 PM
 */

#include "xc.h"
#include "p24Fxxxx.h"
#include "robot.h"

//The vector of possible velocities
#define MOTOR_WIDTH 249 

#define MAX_SPEED 4 

void setupTMR2();
void setupTMR4();
void setupOC2();
void setupOC3();
void setupOC4();
void setupOC5();


unsigned int speed = MAX_SPEED;

unsigned int unassisted = 0;

unsigned long int SPEED_ARRAY[5] = {MOTOR_WIDTH, MOTOR_WIDTH*3/8, MOTOR_WIDTH/4, MOTOR_WIDTH*1/8, 0};
//General setup for the motors

void setUserMode(int i){
    unassisted = i;
}


void __attribute__((__interrupt__,__no_auto_psv__)) _T2Interrupt(void){
    _T2IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _T4Interrupt(void){
    _T4IF = 0;
    stop();
}

void setup_motors(void){    
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;
    setupTMR2();
    setupTMR4();

    setupOC2(); //left motor reverse
    setupOC3(); //left motor forward
    setupOC4(); //right motor backward
    setupOC5(); //right motor forward
    I2C2CONbits.I2CEN = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB3 = 0;
    //All 1's will disengage the motor, only goes to the side with low voltage
    PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
    //PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
    PORTBbits.RB4 = 1; // Left Motor forward, final
    PORTBbits.RB3 = 1; //Right motor backward
    PORTBbits.RB5 = 1; //Right motor forward
//    PORTAbits.RA2 = 1; //Right motor forward , Should be RB2 in the final version
}
void setupTMR2(){
    T2CONbits.TON = 0;
    TMR2 = 0;
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    PR2 = MOTOR_WIDTH;
    _T2IF = 0;
    _T2IE = 1; // Enable the T3 Interrupt
    T2CONbits.TON = 1;
}

void setupTMR4(){
    T4CON = 0;
    TMR4 = 0;
    T4CONbits.TCKPS1 = 1;
    T4CONbits.TCKPS0 = 0;
    PR4 = 15999;
    _T4IF = 0;
    _T4IE = 1; // Enable the T3 Interrupt
   // T4CONbits.TON = 1;
}


void speedUp(void){
    speed++;
    if(speed>MAX_SPEED){
        speed = MAX_SPEED;
    }
}

void speedDown(void){
    speed--;
    if(speed<1){
        speed = 1;
    }
}


//Left motor reverse
void setupOC2(){
    TRISBbits.TRISB9 = 0;
    //TRISB = 0x0000;
    AD1PCFG = 0x9fff;
    T2CONbits.TON = 0;
    TMR2 = 0;
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR4bits.RP9R = 19;  // Use Pin RP12 for Output Compare 2 = "19" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC2CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC2R = SPEED_ARRAY[speed];   // servo start position. We won?t touch OC1R again
    OC2RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC2CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC2CONbits.OCTSEL = 0; // Use Timer 3 for compare source
    //enable the PWM
    OC2CON = OC2CON | 0x0006;
    _T2IE = 1; // Enable the T3 Interrupt
  //  _OC2IE = 1; //Enable the OC1 Interrupt
    T2CONbits.TON = 1;
}
//Left motor forward
void setupOC3(){
    TRISBbits.TRISB4 = 0;
    AD1PCFG = 0x9fff;
    T2CONbits.TON = 0;
    TMR2 = 0;
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR2bits.RP4R = 20;  // Use Pin RP12 for Output Compare 3 = "20" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC3CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC3R = SPEED_ARRAY[speed];   // servo start position. We won?t touch OC1R again
    OC3RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC3CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC3CONbits.OCTSEL = 0; // Use Timer 3 for compare source
    //enable the PWM
    OC3CON = OC3CON | 0x0006;
    _T2IE = 1; // Enable the T3 Interrupt
   // _OC3IE = 1; //Enable the OC1 Interrupt
    T2CONbits.TON = 1;
}
//Right motor backward
void setupOC4(){
    TRISBbits.TRISB3 = 0;
    AD1PCFG = 0x9fff;
    T2CONbits.TON = 0;
    TMR2 = 0;
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR1bits.RP3R = 21;  // Use Pin RP3 for Output Compare 4 = "21" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC4CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC4R = SPEED_ARRAY[speed];   // servo start position. We won?t touch OC1R again
    OC4RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC4CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC4CONbits.OCTSEL = 0; // Use Timer 3 for compare source
    //enable the PWM
    OC4CON = OC4CON | 0x0006;
    _T2IE = 1; // Enable the T3 Interrupt
   // _OC4IE = 1; //Enable the OC1 Interrupt
    T2CONbits.TON = 1;
}
//Right motor forward
void setupOC5(){
    TRISBbits.TRISB5 = 0;
    AD1PCFG = 0x9fff;
    T2CONbits.TON = 0;
    TMR2 = 0;
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR2bits.RP5R = 22;  // Use Pin RP3 for Output Compare 5 = "22" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC5CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC5R = SPEED_ARRAY[speed];   // servo start position. We won?t touch OC1R again
    OC5RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC5CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC5CONbits.OCTSEL = 0; // Use Timer 3 for compare source
    //enable the PWM
    OC5CON = OC5CON | 0x0006;
    _T2IE = 1; // Enable the T3 Interrupt
   // _OC5IE = 1; //Enable the OC1 Interrupt
    T2CONbits.TON = 1;
}
void forward(void){
//    if(!(STOP_LEFT && STOP_RIGHT)){
        PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 0;
        PORTBbits.RB3 = 1; //Right motor backward
        
        int thisSpeed = SPEED_ARRAY[speed];
        
        //left forward
        
        //int distRight = getAvgDistRight();
        
        if((getTurnRight()) && (unassisted == 0)){
            OC3RS = SPEED_ARRAY[0];
        }
        else{
            OC3RS = thisSpeed;
        }

        
        
        //PORTAbits.RA2 = SPEED_ARRAY[SPEED]; //Right motor forward , Should be RB2 in the final version
        
        //int distLeft = getAvgDistLeft();
        
        //right forward
        if((getTurnLeft()) && (unassisted == 0)){
            OC5RS = SPEED_ARRAY[0];
        }
        else{
            OC5RS = thisSpeed;
        }
        
        
        //    OC5RS = 0;

    
        OC2RS = SPEED_ARRAY[0];
        
        OC4RS = SPEED_ARRAY[0];
        TMR4 = 0;
        T4CONbits.TON = 1;
     //   OC3RS = 0;        
        //PORTAbits.RA2 = SPEED_ARRAY[SPEED]; //Right motor forward , Should be RB2 in the final version
 //   }
        
}

void autoForward(void){
//    if(!(STOP_LEFT && STOP_RIGHT)){
        PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 0;
        PORTBbits.RB3 = 1; //Right motor backward
        OC3RS = SPEED_ARRAY[speed];        
        //PORTAbits.RA2 = SPEED_ARRAY[SPEED]; //Right motor forward , Should be RB2 in the final version
        OC5RS = SPEED_ARRAY[speed];
    //    OC5RS = 0;

    
        OC2RS = SPEED_ARRAY[0];
        
        OC4RS = SPEED_ARRAY[0];
        T4CONbits.TON = 0;
     //   OC3RS = 0;        
        //PORTAbits.RA2 = SPEED_ARRAY[SPEED]; //Right motor forward , Should be RB2 in the final version
 //   }
        
}


void reverse(void){
  //  if(!STOP_REAR){
        //PORTBbits.RB9 = SPEED_ARRAY[SPEED]; //Left Motor Reverse,  should be RB3 in the finale version
    
        PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        PORTBbits.RB4 = 1;
        PORTBbits.RB5 = 1;
        PORTBbits.RB3 = 0; //Right motor backward
    
        OC2RS = SPEED_ARRAY[speed];
        OC3RS = SPEED_ARRAY[0];
        
        OC5RS = SPEED_ARRAY[0];
        //PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
     //   PORTBbits.RB4 = 1; // Left Motor forward, final
        //PORTBbits.RB3 = SPEED_ARRAY[SPEED]; //Right motor Reverse
        OC4RS = SPEED_ARRAY[speed];
        //PORTAbits.RA2 = 1; //Right motor Forward , Should be RB2 in the final version
      //  PORTBbits.RB5 = 1;
        TMR4 = 0;
        T4CONbits.TON = 1;
  //  }
}

void autoReverse(void){
  //  if(!STOP_REAR){
        //PORTBbits.RB9 = SPEED_ARRAY[SPEED]; //Left Motor Reverse,  should be RB3 in the finale version
    
        PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        PORTBbits.RB4 = 1;
        PORTBbits.RB5 = 1;
        PORTBbits.RB3 = 0; //Right motor backward
    
        OC2RS = SPEED_ARRAY[speed];
        OC3RS = SPEED_ARRAY[0];
        
        OC5RS = SPEED_ARRAY[0];
        //PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
     //   PORTBbits.RB4 = 1; // Left Motor forward, final
        //PORTBbits.RB3 = SPEED_ARRAY[SPEED]; //Right motor Reverse
        OC4RS = SPEED_ARRAY[speed];
        //PORTAbits.RA2 = 1; //Right motor Forward , Should be RB2 in the final version
      //  PORTBbits.RB5 = 1;
        T4CONbits.TON = 0;
  //  }
}

void left(void){
  //  if(!STOP_LEFT){
        //PORTBbits.RB9 = SPEED_ARRAY[SPEED]; //Left Motor Reverse,  should be RB3 in the finale version
    
        PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        PORTBbits.RB4 = 1;
        PORTBbits.RB5 = 0;
        PORTBbits.RB3 = 1; //Right motor backward
    
        OC2RS = SPEED_ARRAY[speed];
        //PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
     //   PORTBbits.RB4 = 1;
      //  PORTBbits.RB3 = 1; //Right motor Reverse
        //PORTAbits.RA2 = SPEED_ARRAY[SPEED]; //Right motor forward , Should be RB2 in the final version
        OC5RS = SPEED_ARRAY[speed];
        OC3RS = SPEED_ARRAY[0];
        
        OC4RS = SPEED_ARRAY[0];
        TMR4 = 0;
        T4CONbits.TON = 1;
        
 //   }
}

void right(void){
  //  if(!STOP_RIGHT){
        
        
        PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 1;
        PORTBbits.RB3 = 0; //Right motor backward
        
    //    PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
        //PORTAbits.RA4 = SPEED_ARRAY[SPEED]; //Left Motor Forward, should be RB3in the final version
        OC3RS = SPEED_ARRAY[speed];
        //PORTBbits.RB3 = SPEED_ARRAY[SPEED]; //Right motor backward
        OC4RS = SPEED_ARRAY[speed];
        //PORTAbits.RA2 = 1; //Right motor forward , Should be RB2 in the final version
     //   PORTBbits.RB5 = 1;
  //  }
        OC2RS = SPEED_ARRAY[0];
        
        OC5RS = SPEED_ARRAY[0];
        
        TMR4=0;
        T4CONbits.TON = 1;
        
        
}

void stop(void){
    OC2RS = SPEED_ARRAY[0];
    OC3RS = SPEED_ARRAY[0];
    OC4RS = SPEED_ARRAY[0];  
    OC5RS = SPEED_ARRAY[0];
    T4CONbits.TON = 0;
}
