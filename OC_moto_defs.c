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
#define MOTOR_WIDTH 249 //Defines the width of the PWM pulses to the motors, choice in PWM width is hardware dependent

#define MAX_SPEED 4 //Defines the position in SPEED_ARRAY of the maximum speed. At full speed, that value is zero

void setupTMR2(); //timer for output compares
void setupTMR4();
void setupOC2(); // Left motor reverse
void setupOC3(); // Left motor forward
void setupOC4(); // Right motor reverse
void setupOC5(); // Right motor forward


unsigned int speed = MAX_SPEED;  //Defaults the speed of the device to its maximum speed

unsigned int unassisted = 0; //local variable to the OC_MOTO program that tells if unassisted or not

unsigned long int SPEED_ARRAY[5] = {MOTOR_WIDTH, MOTOR_WIDTH*3/8, MOTOR_WIDTH/4, MOTOR_WIDTH*1/8, 0};
//General setup for the motors

void setUserMode(int i){
    unassisted = i;
}


void __attribute__((__interrupt__,__no_auto_psv__)) _T2Interrupt(void){
    _T2IF = 0; //Boilerplate interrupt clear, necessary for operation of output compare
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
//Establish the timer for output compare
void setupTMR2(){
    T2CONbits.TON = 0; //Reset internal state of timer 2
    TMR2 = 0; //Set value of timer 2 to zero
    T2CONbits.TCKPS1 = 0; //Give a prescaler of one to the timer
    T2CONbits.TCKPS0 = 0;
    PR2 = MOTOR_WIDTH; //Give the timer the PWM_WIDTH as its period
    _T2IF = 0; //Clear the interrupt flag
    _T2IE = 1; // Enable the T3 Interrupt
    T2CONbits.TON = 1; //Turn on the T2 interrupt
}

void setupTMR4(){
    T4CON = 0;
    TMR4 = 0;
    T4CONbits.TCKPS1 = 1;
    T4CONbits.TCKPS0 = 0;
    PR4 = 15999;
    _T4IF = 0;
    _T4IE = 1; // Enable the T4 Interrupt
   // T4CONbits.TON = 1;
}


void speedUp(void){
    speed++; //increase speed by reducing the duty cycle to output compare
    if(speed>MAX_SPEED){ //Makes sure there are no out-of-bounds array accesses
        speed = MAX_SPEED; //If the user tries to increase speed while at MAX_SPEED keep at maximum speed
    }
}

void speedDown(void){
    speed--; //decrease speed by increasing the duty cycle of the output compare
    if(speed<1){ //If the user tries to achieve a speed less than the available speeds, keep them at minimum.
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
    OC2R = SPEED_ARRAY[speed];   // Starting speed of the motors
    OC2RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC2CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC2CONbits.OCTSEL = 0; // Use Timer 2 for compare source
    //enable the PWM
    OC2CON = OC2CON | 0x0006;
    _T2IE = 1; // Enable the T2 Interrupt
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
    RPOR2bits.RP4R = 20;  // Use Pin RP4 for Output Compare 3 = "20" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC3CON = 0;    // turn off OC3 for now
    //These lines simply give a starting position. Not entirely essential
    OC3R = SPEED_ARRAY[speed];   //Starting speed
    OC3RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC3CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC3CONbits.OCTSEL = 0; // Use Timer 2 for compare source
    //enable the PWM
    OC3CON = OC3CON | 0x0006;
    _T2IE = 1; // Enable the T2 Interrupt
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
    OC4CON = 0;    // turn off OC4 for now
    //These lines simply give a starting position. Not entirely essential
    OC4R = SPEED_ARRAY[speed];   // Starting speed
    OC4RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC4CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC4CONbits.OCTSEL = 0; // Use Timer 2 for compare source
    //enable the PWM
    OC4CON = OC4CON | 0x0006;
    _T2IE = 1; // Enable the T2 Interrupt
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
    RPOR2bits.RP5R = 22;  // Use Pin RP5 for Output Compare 5 = "22" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC5CON = 0;    // turn off OC5 for now
    //These lines simply give a starting position. Not entirely essential
    OC5R = SPEED_ARRAY[speed];   // Starting speed
    OC5RS = SPEED_ARRAY[speed];  // We will only change this once PWM is turned on
    OC5CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC5CONbits.OCTSEL = 0; // Use Timer 2 for compare source
    //enable the PWM
    OC5CON = OC5CON | 0x0006;
    _T2IE = 1; // Enable the T2 Interrupt
   // _OC5IE = 1; //Enable the OC1 Interrupt
    T2CONbits.TON = 1;
}
//Throughout the following functions, all navigation can be achieved simply by using the first four lines, however,
//the speed control is handled by the code blocks which change the speed for output compares
void forward(void){
//    if(!(STOP_LEFT && STOP_RIGHT)){
        PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 0;
        PORTBbits.RB3 = 1; //Right motor backward
        //Define the speed from speed array
        int thisSpeed = SPEED_ARRAY[speed];

        if((getTurnRight()) && (unassisted == 0)){
            OC3RS = SPEED_ARRAY[0]; //Off condition
        }
        else{
            OC3RS = thisSpeed; //User defined speed
        }

        if((getTurnLeft()) && (unassisted == 0)){
            OC5RS = SPEED_ARRAY[0]; //Off condition
        }
        else{
            OC5RS = thisSpeed; //user defined speed
        }
    
        OC2RS = SPEED_ARRAY[0]; //makes the left rear "high" causing left tire to spin forward with PWM defined speed
        OC4RS = SPEED_ARRAY[0]; //makes the right rear "high" causing right tire to spin forward with PWM defined speed
        
        TMR4 = 0; //Clear timer four
        T4CONbits.TON = 1; //Turn the timer on       
}
//Cruise control forward
void autoForward(void){
        PORTBbits.RB9 = 1; //Left Motor Reverse,  should be RB3 in the finale version
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 0;
        PORTBbits.RB3 = 1; //Right motor backward
    
        OC3RS = SPEED_ARRAY[speed]; //left rear PWM pulse. shorter PWM is greater speed        
        OC5RS = SPEED_ARRAY[speed]; //right rear PWM pulse. Shorter PWM is greater speed
    
        OC2RS = SPEED_ARRAY[0]; //causes left tire to spin forward
        OC4RS = SPEED_ARRAY[0]; //causes right tire to spin forward
        
        T4CONbits.TON = 0;       
}

//Drive the robot in reverse
void reverse(void){
    
        PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version

        PORTBbits.RB4 = 1;
        PORTBbits.RB5 = 1;
        PORTBbits.RB3 = 0; //Right motor backward
    
        OC2RS = SPEED_ARRAY[speed]; //Set PWM width of left motor reverse 
        OC3RS = SPEED_ARRAY[0]; //spin left tire rearward
        
        OC5RS = SPEED_ARRAY[0]; //spin right tire rearward
        OC4RS = SPEED_ARRAY[speed]; //defien PWM width of the right tire

        TMR4 = 0; //clear timer 4
        T4CONbits.TON = 1; //turn timer four on again
  //  }
}
//Drive the robot in cruise control reverse
void autoReverse(void){

    
        PORTBbits.RB9 = 0; //Left Motor Reverse,  should be RB3 in the finale version
        PORTBbits.RB4 = 1;
        PORTBbits.RB5 = 1;
        PORTBbits.RB3 = 0; //Right motor backward
    
        OC2RS = SPEED_ARRAY[speed]; //set reverse speed
        OC3RS = SPEED_ARRAY[0]; //spin left  tire reverse
        
        OC5RS = SPEED_ARRAY[0]; //spin right tire reverse
        OC4RS = SPEED_ARRAY[speed]; //set reverse speed

        T4CONbits.TON = 0; //turn the timer off
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
    
        OC2RS = SPEED_ARRAY[speed]; //set left tire speed 
        //PORTAbits.RA4 = 1; //Left Motor Forward, should be RB3in the final version
     //   PORTBbits.RB4 = 1;
      //  PORTBbits.RB3 = 1; //Right motor Reverse
        //PORTAbits.RA2 = SPEED_ARRAY[SPEED]; //Right motor forward , Should be RB2 in the final version
        OC5RS = SPEED_ARRAY[speed]; //set right tire speed
        OC3RS = SPEED_ARRAY[0]; //move left in reverse
        
        OC4RS = SPEED_ARRAY[0]; //move right forward
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
        
 
        OC3RS = SPEED_ARRAY[speed]; //set left speed
        //PORTBbits.RB3 = SPEED_ARRAY[SPEED]; //Righ t motor backward
        OC4RS = SPEED_ARRAY[speed]; //set right speeed
  
        OC2RS = SPEED_ARRAY[0]; //spin left tire rearward
        
        OC5RS = SPEED_ARRAY[0]; //spin right tire forward
        
        TMR4=0;
        T4CONbits.TON = 1;
        
        
}
//stop the vehicle
void stop(void){
    OC2RS = SPEED_ARRAY[0]; //set all OC2's to max duty cycle, stops droid
    OC3RS = SPEED_ARRAY[0];
    OC4RS = SPEED_ARRAY[0];  
    OC5RS = SPEED_ARRAY[0];
    T4CONbits.TON = 0;
}
