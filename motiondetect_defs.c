/*
 * File:   rear_hc.c
 * Author: Grant
 *
 * Created on April 19, 2018, 5:44 PM
 */


#include "xc.h"
#include "p24Fxxxx.h"
#include "scanner.h"
#define TRIG_WIDTH 3759 //Value in PR3 for both trigger signals.
#define TRIG_PAT 3758
#define TRIG_PAT1 3756
#define START 20
#define MIN_DISTANCE 600
#define NOISE_TOLERANCE 50
volatile unsigned int T1OV; //current number of overflows
volatile unsigned int sent_ov; //number of overflows at delta
void __attribute__((__interrupt__,__auto_psv__)) _OC1Interrupt(void){
    _OC1IF = 0;
}
void __attribute__((__interrupt__,__auto_psv__)) _OC2Interrupt(void){
    _OC2IF = 0;
}
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void){
    _T1IF = 0;
    T1OV += 1;
}
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(void){
    _T3IF = 0;
}
volatile unsigned int seconds;
void __attribute__((__interrupt__,__auto_psv__)) _T5Interrupt(void){
    _T5IF = 0;
}
//Trigger Rear
void setupOC1(void){
    //Bind OC1 to RP6/RB6
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    TRISBbits.TRISB14 = 0;
    //TRISB = 0x0000;
    AD1PCFG = 0x9fff;
    T3CONbits.TON = 0;
    TMR3 = 0;
    T3CONbits.TCKPS1 = 1;
    T3CONbits.TCKPS0 = 1;
    //Set PR3 to be 20mS, this was before adding clear timers in the int func
    //and at TCKPS[1:0] = 11
    //PR3 = 149
    //Turned RCdiv to right channel
    PR3 = TRIG_WIDTH;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR3bits.RP6R = 18;  // Use Pin RP12 for Output Compare 1 = "18" (Table 10-3)
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
    T3CONbits.TON = 0;
    TMR3 = 0;
    T3CONbits.TCKPS1 = 1;
    T3CONbits.TCKPS0 = 1;
    PR3 = TRIG_WIDTH;
    _T3IE = 1; // Enable the T3 Interrupt
    T3CONbits.TON = 1;
}
volatile unsigned int seconds;
void setupTMR5(void){
    //Create a one second timer
    T5CONbits.TON = 0;
    TMR5 = 0;
    T5CONbits.TCKPS0 = 1;
    T5CONbits.TCKPS1 = 1;
    PR5 = 62499;
    _T5IF = 0;
    //_T2IP = 0b010;
    _T5IE = 1;
    T5CONbits.TON = 1;
}

//Trigger pulse timer

void setupTMR1(void){
    //Create a ten micro second timer for the trigger pulses with PR1 = 159.
    //Switched to maximum to prevent overflows and use another function to write
    //the pulse
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
volatile unsigned long long int gamma; //Falling edge
volatile unsigned long long int beta; //Falling edge
volatile unsigned long long int alpha; //Rising edge
volatile unsigned long long int rise;
volatile unsigned long long int fall;
//front right
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void){
    //Rising edge is the primary side.
    if(_INT0EP == 0){
        rise = TMR1 + (T1OV * 0xffff);
        sent_ov = T1OV;
    }
    else if(_INT0EP == 1){
        fall = TMR1 + (T1OV * 0xffff);
        sent_ov = 0;
        T1OV = 0;
        setOutput_right();
    }
    _INT0EP ^= 1;
    _INT0IE = 1;
    _INT0IF = 0;
}
//rear
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(void){
    //Rising edge is the primary side.
    if(_INT1EP == 0){
        gamma = TMR1 + (T1OV * 0xffff);
        sent_ov = T1OV;
    }
    else if(_INT1EP == 1){
        delta = TMR1 + (T1OV * 0xffff);
        sent_ov = 0;
        T1OV = 0;
        setOutput_rear();
    }
    _INT1EP ^= 1;
    _INT1IE = 1;
    _INT1IF = 0;
}
//front left
void __attribute__((__interrupt__,__auto_psv__)) _INT2Interrupt(void){
    //Rising edge is the primary side.
    if(_INT2EP == 0){
        alpha = TMR1 + (T1OV * 0xffff);
        sent_ov = T1OV;
    }
    else if(_INT2EP == 1){
        beta = TMR1 + (T1OV * 0xffff);
        sent_ov = 0;
        T1OV = 0;
        setOutput_left();
    }
    _INT2EP ^= 1;
    _INT2IE = 1;
    _INT2IF = 0;
}
//front right
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
//rear
void setupINT1(void){
    TRISBbits.TRISB13 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR0bits.INT1R = 13;  // Use Pin RP13 for interrupt
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    //Turn on the interrupts
    IEC1bits.INT1IE = 1;
    //High priority interrupt
    IPC5bits.INT1IP = 0b011;
    //Set edge polarity to rising edges.
    INTCON2bits.INT1EP = 0;
    //Turn off the interrupt flags.
    IFS1bits.INT1IF = 0;
}
//front left
void setupINT2(void){
    TRISBbits.TRISB10 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR1bits.INT2R = 10;  // Use Pin RP10 for interrupt
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    //Turn on the interrupts
    IEC1bits.INT2IE = 1;
    //High priority interrupt
    IPC7bits.INT2IP = 0b011;
    //Set edge polarity to rising edges.
    INTCON2bits.INT2EP = 0;
    //Turn off the interrupt flags.
    IFS1bits.INT2IF = 0;
}
//This routine is for the front sensor
volatile unsigned int microseconds; // Number of microseconds from gamma to delta
volatile unsigned int readings[10] ={}; //Buffer for last twenty values we have seen
volatile unsigned int reading_pos = 0; //Place in the buffer
volatile int STOP_LEFT = 0; //Tells us whether to brake or not
volatile unsigned int read_sum; //Like a check sum, tell me how scary the last readings were
void setOutput_left(void){
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
        microseconds = 10000;
    }
    if(microseconds < MIN_DISTANCE && microseconds > NOISE_TOLERANCE){
        //OC2RS = 1000 ;
        readings[reading_pos] = 1;
    }
    else{
        readings[reading_pos] = 0;
    }
    int i = 0;
    while(i < 10){
        read_sum += readings[i];
        i += 1;
    }
    if(read_sum < 1){
        PORTBbits.RB8 = 0;
        STOP_LEFT = 0;
    }
    else{
        PORTBbits.RB8 = 1;
        STOP_LEFT = 1;
        left_tooClose(1);
    }
    reading_pos += 1;
    if(reading_pos == 10){
        reading_pos = 0;
    }
    read_sum = 0;
    return;
}
//This routine is for the rear sensor
volatile unsigned int microseconds2; // Number of microseconds from gamma to delta
volatile unsigned int readings2[10] ={}; //Buffer for last twenty values we have seen
volatile unsigned int reading_pos2 = 0; //Place in the buffer
volatile int STOP_REAR = 0; //Tells us whether to brake or not
volatile unsigned int read_sum2; //Like a check sum, tell me how scary the last readings were
void setOutput_rear(void){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    //microseconds = (beta - alpha)/16; //clock cycles to microseconds
    microseconds2 = (delta - gamma)/16; //clock cycles to microseconds rear
    if(microseconds2 < MIN_DISTANCE && microseconds > NOISE_TOLERANCE){
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
    if(read_sum2 < 1){
        PORTBbits.RB8 = 0;
        STOP_REAR = 0;
    }
    else{
        PORTBbits.RB8 = 1;
        //stop();
        STOP_REAR = 1;
        rear_tooClose(1);
    }
    reading_pos2 += 1;
    if(reading_pos2 == 10){
        reading_pos2 = 0;
    }
    read_sum2 = 0;
    return;
}
volatile unsigned int microseconds3; // Number of microseconds from gamma to delta
volatile unsigned int readings3[10] ={}; //Buffer for last twenty values we have seen
volatile unsigned int reading_pos3 = 0; //Place in the buffer
volatile int STOP_RIGHT = 0; //Tells us whether to brake or not
volatile unsigned int read_sum3; //Like a check sum, tell me how scary the last readings were
void setOutput_right(void){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    //microseconds = (beta - alpha)/16; //clock cycles to microseconds
    microseconds3 = (fall - rise)/16; //clock cycles to microseconds rear
    if(microseconds3 < MIN_DISTANCE && microseconds3 > NOISE_TOLERANCE){
        readings3[reading_pos3] = 1;
    }
    else{
        readings3[reading_pos3] = 0;
    }
    int i = 0;
    while(i < 10){
        read_sum3 += readings3[i];
        i += 1;
    }
    if(read_sum3 < 2){
        PORTBbits.RB8 = 0;
        STOP_RIGHT = 0;
    }
    else{
        PORTBbits.RB8 = 1;
        //stop();
        STOP_RIGHT = 1;
        right_tooClose(1);
    }
    reading_pos3 += 1;
    if(reading_pos3 == 10){
        reading_pos3 = 0;
    }
    read_sum3 = 0;
    return;
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

