/*
 * File:   rear_hc.c
 * Author: Grant
 *
 * Created on April 19, 2018, 5:44 PM
 */


#include "xc.h"
#include "p24Fxxxx.h"
#include "robot.h"
/*These choices in TRIG_WIDTH and TRIG_PAT are to establish a 17Hz PWM signal with a 16uS pulse, fitting within
the bounds of the HC-SR04 which demands a maximum trigger rate of 17Hz and a minimum TTL trigger of 10uS at 5V.
The names of these constants do not make that abundantly clear, but were the fastest stable choices discovered 
during the production of version 1*/
#define TRIG_WIDTH 3759 //Value in PR3 for both trigger signals.
#define TRIG_PAT 3758 //Duty cycle of OC1. It is one less than TRIG_WIDTH to send the 10uS TTL trigger to the HC-SR04
#define START 20


#define TURNAWAY_DISTANCE 3000
#define AUTOSTOP_DISTANCE 1500
#define NOISE_TOLERANCE 100
volatile unsigned int T1OV; //current number of overflows on timer one. Used for timing calculation in the external interrupt
//routines
volatile unsigned int sent_ov; //number of overflows on timer1 at the time a rising edge is detected. This variable more than
//likely was the root of errors during testing, will be refactored in future renditions. This simply gives a context of how 
//many times TMR1 overflowed at the time that 

volatile int detectFront = 0;
volatile int detectBack = 0;

volatile int turnLeft = 0;
volatile int turnRight = 0;

void setOutput_right();
void setOutput_left();
void setOutput_rear();

void collisionFront();
void collisionBack();


void __attribute__((__interrupt__,__auto_psv__)) _OC1Interrupt(void){
    _OC1IF = 0; //Boilerplate, clear the OC1 interrupt.
}

int getFrontStatus(){
    return detectFront;
}

int getBackStatus(){
    return detectBack;
}
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void){
    _T1IF = 0; //Boilerplate, clera the interrupt flag
    T1OV += 1; //Keep track of timer one overflows.
}
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(void){
    _T3IF = 0; //Boilerplate, necessary for operation of peripherals
}
void __attribute__((__interrupt__,__auto_psv__)) _T5Interrupt(void){
    T5CONbits.TON = 0; //Turn off timer5
    _T5IF = 0;
    detectFront = 0;
    detectBack = 0;
}
//Trigger Rear
void setupOC1(void){
    TRISBbits.TRISB6 = 0;
    AD1PCFG = 0x9fff;
    T3CONbits.TON = 0; //Turn off the timer to set it up
    TMR3 = 0;
    T3CONbits.TCKPS1 = 1;
    T3CONbits.TCKPS0 = 1;
    PR3 = TRIG_WIDTH;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR3bits.RP6R = 18;  // Use Pin RP6 for Output Compare 1 = "18" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC1CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC1R = TRIG_PAT;   // servo start position. We won?t touch OC1R again
    OC1RS = TRIG_PAT;  // We will only change this once PWM is turned on
    OC1CONbits.OCM = 0b110; // Output compare PWM w/o faults
    //OC1CONbits.OCM = 0b011; //Compare event toggles pins
    OC1CONbits.OCTSEL = 1; // Use Timer 3 for compare source
    //enable the PWM
    OC1CON = OC1CON | 0x0006;
    _T3IE = 1; // Enable the T3 Interrupt
    _OC1IE = 1; //Enable the OC1 Interrupt
    T3CONbits.TON = 1;
}
//Output compare timer: Runs at 17Hz
void setupTMR3(void){
    T3CONbits.TON = 0; //Turn on TMR3
    TMR3 = 0; //Clear TMR3
    T3CONbits.TCKPS1 = 1; //Set the prescaler to 256
    T3CONbits.TCKPS0 = 1;
    PR3 = TRIG_WIDTH; //Give TMR3 the proper width
    _T3IE = 1; // Enable the T3 Interrupt
    T3CONbits.TON = 1;
}
volatile unsigned int seconds; //Time context
void setupTMR5(void){
    //Create a one second timer
    T5CONbits.TON = 0;
    TMR5 = 0;
    T5CONbits.TCKPS0 = 1;
    T5CONbits.TCKPS1 = 1;
    PR5 = (62500)/2-1;
    _T5IF = 0;
    //_T2IP = 0b010;
    _T5IE = 1;
    T5CONbits.TON = 0;
}

//Trigger pulse timer

void setupTMR1(void){
    //Switched to maximum to prevent overflows and use another function to write
    //the pulse. This timer is used by the external interrupts to keep track of the time for rising and falling edge
    T1CONbits.TON = 0;
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TCKPS1 = 0;
    TMR1 = 0;
    PR1 = 0xffff;
    _T1IF = 0;
    //_T1IP = 0b001;
    _T1IE = 1;
    T1CONbits.TON = 1;
}
volatile unsigned int ic2 = 0; //count to tell us which edge, makes sure delta rises
volatile unsigned long long int delta; //Rising edge
volatile unsigned long long int sigma; //Falling edge
volatile unsigned long long int beta; //Falling edge
volatile unsigned long long int alpha; //Rising edge
volatile unsigned long long int rise; //Rising edge
volatile unsigned long long int fall; //Falling edge
//front right
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void){
    //Rising edge is the primary side.
    if(_INT0EP == 0){ //Read the rising edge
        rise = TMR1 + (T1OV * 0xffff); //store the time at rise
        sent_ov = T1OV; //Store the overflows at rise
    }
    else if(_INT0EP == 1){
        fall = TMR1 + (T1OV * 0xffff); //Store the time at fall
        sent_ov = 0; //Clear time contexts for next pulse
        T1OV = 0;
        _INT0IE = 0;
        _INT2IE = 1;
        setOutput_right();
        
    }
    _INT0EP ^= 1;//If you read a rise, read a fall next. Else read a rise next.
    _INT0IF = 0;
}
//rear
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(void){
    //Rising edge is the primary side.
    if(_INT1EP == 0){ //Read for the rise
        sigma = TMR1 + (T1OV * 0xffff); //Time of rising edge
        sent_ov = T1OV; //Time context of the rising edge
    }
    else if(_INT1EP == 1){ //Read for the fall
        delta = TMR1 + (T1OV * 0xffff); //Time of falling edge
        sent_ov = 0; //Clear time contexts
        T1OV = 0;
        setOutput_rear();
    }
    _INT1EP ^= 1; //If you read a rise, read a fall next. Else read a rise next.
    _INT1IE = 1;
    _INT1IF = 0;
}
//front left
void __attribute__((__interrupt__,__auto_psv__)) _INT2Interrupt(void){
    //Rising edge is the primary side.
    if(_INT2EP == 0){
        alpha = TMR1 + (T1OV * 0xffff); //Time of rising edge
        sent_ov = T1OV;
    }
    else if(_INT2EP == 1){
        beta = TMR1 + (T1OV * 0xffff); //Time of falling edge
        sent_ov = 0; //Clear time contexts
        T1OV = 0;
        _INT0IE = 1;
        _INT2IE = 0;
        setOutput_left();
    }
    _INT2EP ^= 1; //If you read a rise, read a fall next. Else read a rise next.
  //  _INT2IE = 1;

    _INT2IF = 0;
}
//front right -- Initialize the module that measures echo pulses from the front right sensor
void setupINT0(void){ 
    TRISBbits.TRISB7 = 1;
    //Turn on the interrupts
    IEC0bits.INT0IE = 1;
    //Set as high priority interrupts
    IPC0bits.INT0IP = 0b010;
    //Set edge polarity to rising edges.
    INTCON2bits.INT0EP = 0;
    //Turn off the interrupt flags.
    IFS0bits.INT0IF = 0;
}
//rear -- Initialize the module that measures echo pulses from the rear sensor
void setupINT1(void){
    TRISBbits.TRISB13 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR0bits.INT1R = 13;  // Use Pin RP13 for interrupt
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    //Turn on the interrupts
  //  IEC1bits.INT1IE = 1;
    //High priority interrupt
    IPC5bits.INT1IP = 0b011;
    //Set edge polarity to rising edges.
    INTCON2bits.INT1EP = 0;
    //Turn off the interrupt flags.
    IFS1bits.INT1IF = 0;
}
//front left -- Initialize the module that measures echo pulses from the front left sensor
void setupINT2(void){
    TRISBbits.TRISB10 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR1bits.INT2R = 10;  // Use Pin RP10 for interrupt
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    //Turn on the interrupts
   // IEC1bits.INT2IE = 1;
    //High priority interrupt
    IPC7bits.INT2IP = 0b011;
    //Set edge polarity to rising edges.
    INTCON2bits.INT2EP = 0;
    //Turn off the interrupt flags.
    IFS1bits.INT2IF = 0;
}

int getTurnRight(){
    return turnRight;
}

int getTurnLeft(){
    return turnLeft;
}

//This routine is for the front sensor
volatile unsigned int microseconds; // Number of microseconds from sigma to delta
volatile unsigned int readings[10] ={}; //Buffer for last twenty values we have seen
volatile unsigned int farReadings[10] = {};
volatile long readingVal[10] = {};
volatile unsigned int reading_pos = 0; //Place in the buffer
volatile unsigned int reading_pos_short = 0; //Place in the buffer
volatile int STOP_LEFT = 0; //Tells us whether to brake or not
volatile unsigned int read_sum; //Like a check sum, tell me how scary the last readings were
volatile long encroach_sum;
void setOutput_left(){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    //microseconds = (beta - alpha)/16; //clock cycles to microseconds
    if(beta>alpha){
        microseconds = (beta - alpha)/16; //clock cycles to microseconds rear
    }
    else{
        microseconds = 10000; //A virtual infinity. Has no meaning beyond making sure that we don't record a "hot" reading
    }
    if(microseconds < AUTOSTOP_DISTANCE && microseconds > NOISE_TOLERANCE){
        //OC2RS = 1000 ;
        readings[reading_pos] = 1; //Indicates a "hot" or "too close" event
    }
    else{
        readings[reading_pos] = 0; //Indicates a "situation normal" event
    }
    if(microseconds < TURNAWAY_DISTANCE && microseconds > NOISE_TOLERANCE){
        //OC2RS = 1000 ;
        farReadings[reading_pos] = 1;
    }
    else{
        farReadings[reading_pos] = 0;
    }
    //readingVal[reading_pos_short] = microseconds;
    int i = 0;
    //Sums the values in the buffer
    while(i < 10){
        read_sum += readings[i];
        encroach_sum += farReadings[i];
        i += 1;
    }
    //encroach_avg = (readingVal[0]+readingVal[1]+readingVal[2]+readingVal[3]+readingVal[4])/5;
    //Condition for "situation normal"
    if(encroach_sum < 7){
        PORTBbits.RB8 = 0;
        turnRight = 0;
       // STOP_LEFT = 0;
    }
    //Condition for "make a correction"
    else if(read_sum < 7&&encroach_sum >= 7){
        PORTBbits.RB8 = 0;
        turnRight =1;
    }
    //Condition for "stop"
    else{
        PORTBbits.RB8 = 1;
        stop();
        collisionFront();
      //  STOP_LEFT = 1;
        //right();
    }
    reading_pos += 1;
    //reading_pos_short += 1;
    if(reading_pos == 10){
        reading_pos = 0;
    }
    //if(reading_pos_short == 7){
      //  reading_pos = 0;
    //}
   // encroach_avg = 0;
    encroach_sum = 0;
    read_sum = 0;
    return;
}



//This routine is for the rear sensor
volatile unsigned int microseconds2; // Number of microseconds from sigma to delta
volatile unsigned int readings2[10] ={}; //Buffer for last twenty values we have seen
volatile unsigned int reading_pos2 = 0; //Place in the buffer
volatile int STOP_REAR = 0; //Tells us whether to brake or not
volatile unsigned int read_sum2; //Like a check sum, tell me how scary the last readings were
void setOutput_rear(){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    //microseconds = (beta - alpha)/16; //clock cycles to microseconds
    //microseconds2 = (delta - sigma)/16; //clock cycles to microseconds rear
    if(delta>sigma){
        microseconds2 = (delta - sigma)/16; //clock cycles to microseconds rear
    }
    else{
        microseconds2 = 10000;
    }
    if(microseconds2 < AUTOSTOP_DISTANCE && microseconds2 > NOISE_TOLERANCE){
        readings2[reading_pos2] = 1;
    }
    else{
        readings2[reading_pos2] = 0;
    }
    int i = 0;
    while(i < 10){
        read_sum2 += readings2[i];
        i += 1;
    }
    if(read_sum2 < 5){
        PORTBbits.RB8 = 0;
        STOP_REAR = 0;
    }
    else{
        PORTBbits.RB8 = 1;
        stop();
        collisionBack();
        STOP_REAR = 1;
        //forward();
    }
    reading_pos2 += 1;
    if(reading_pos2 == 10){
        reading_pos2 = 0;
    }
    read_sum2 = 0;
    return;
}
volatile unsigned int microseconds3; // Number of microseconds from sigma to delta
volatile unsigned int readings3[10] ={}; //Buffer for last twenty values we have seen
volatile unsigned int farReadings3[10] = {};
volatile unsigned int reading_pos3 = 0; //Place in the buffer
volatile unsigned int reading_pos3_short = 0;
volatile int STOP_RIGHT = 0; //Tells us whether to brake or not
volatile unsigned int read_sum3; //Like a check sum, tell me how scary the last readings were
volatile long readingVal3[10] = {};
volatile long encroach_sum3;
volatile long encroach_avg3;
void setOutput_right(){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    //microseconds = (beta - alpha)/16; //clock cycles to microseconds
   // microseconds3 = (fall - rise)/16; //clock cycles to microseconds rear
    if(fall>rise){
        microseconds3 = (fall - rise)/16; //clock cycles to microseconds rear
    }
    else{
        microseconds3 = 10000;
    }  
    if(microseconds3 < AUTOSTOP_DISTANCE && microseconds3 > NOISE_TOLERANCE){
        readings3[reading_pos3] = 1;
    }
    else{
        readings3[reading_pos3] = 0;
    }
    if(microseconds3 < TURNAWAY_DISTANCE && microseconds3 > NOISE_TOLERANCE){
        //OC2RS = 1000 ;
        farReadings3[reading_pos] = 1;
    }
    else{
        farReadings3[reading_pos3] = 0;
    }
    //readingVal3[reading_pos3_short] = microseconds3;
    int i = 0;
    while(i < 10){
        read_sum3 += readings3[i];
        encroach_sum3 += farReadings3[i];
        i += 1;
    }
    //encroach_avg3 = (readingVal3[0]+readingVal3[1]+readingVal3[2]+readingVal3[3]+readingVal3[4]);
    if(encroach_sum3 < 7){
        PORTBbits.RB8 = 0;
        turnLeft = 0;
      //  STOP_RIGHT = 0;
    }
    else if(read_sum3 < 7&&encroach_sum3 >= 7){
        PORTBbits.RB8 = 0;
        turnLeft =1;
    }
    else{
        PORTBbits.RB8 = 1;
        stop();
        collisionFront();
       // asm("nop");
      //  STOP_RIGHT = 1;
        //left();
    }
    reading_pos3 += 1;
   // reading_pos3_short += 1;
    if(reading_pos3 == 10){
        reading_pos3 = 0;
    }
    //if(reading_pos3_short == 7){
      //  reading_pos3_short = 0;
    //}
    encroach_sum3 = 0;
    read_sum3 = 0;
    return;
    
    
}

void collisionFront(){
    detectFront = 1;
    TMR5 = 0;
    T5CONbits.TON = 1;
}

void collisionBack(){
    detectBack = 1;
    TMR5 = 0;
    T5CONbits.TON = 1;
}

long getAvgDistLeft(){
   // return encroach_avg;
}

long getAvgDistRight(){
    //return encroach_avg3;
}


void setup_scan(void){
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    setupINT0();
    setupINT1();
    setupINT2();
    setupTMR1();
    setupOC1();
    setupTMR3();
    setupTMR5();
}

