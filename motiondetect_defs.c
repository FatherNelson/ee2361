/*
 * File:   hc-sro4_defs.c
 * Author: Grant
 *
 * Created on March 10, 2018, 12:04 PM
 */

/*Currently this is a program that just adjusts the brightness of an LED to
 correspond to a hand. We will make output compare one responsible for the 
 trigger signal. The connections are as follows:
 RB6 = TRIG
 RB8 = LED
 RB7 = ECHO
 * Output compare one is going to handle the trigger sets and oc2 will handle
 * the output to indicate the operation of the circuit. OC2 will be bound to RB7 
 */
#include "xc.h"
#include "p24Fxxxx.h"
#include "robot.h"
#include "math.h"
volatile unsigned int seconds;

#define PWM_WIDTH 15000
#define PULSE_WIDTH 3748 //Should be one less than "one second', the rate of polling
#define TRIG_WIDTH 3749
#define ONE_SECOND 62499
#define BOOST_WIDTH 159
//#define WHEEL_TEST 1
//Global variables
//Sets output pulse
volatile unsigned long long int start_of_trig;
volatile unsigned long long int end_of_trig;
volatile unsigned long int alpha;
volatile unsigned long int beta;
volatile unsigned long int gamma;
volatile unsigned long int delta;
//record send and received
volatile unsigned long long int sent_trig;
volatile unsigned long long int received_echo;
volatile unsigned long int sent_ov;
volatile unsigned long int received_ov;
//Keep track of the wrap arounds of the micro second timer
volatile unsigned int T1OV; 
//Keeps track of how many seconds have passed since the start of the program.
volatile long unsigned int seconds_passed;
volatile unsigned long int microseconds;
volatile unsigned long int microseconds2;
//Variable that tells us we should stop
volatile int STOP;
//ISR's
//Boost
void __attribute__((__interrupt__,__auto_psv__)) _OC3Interrupt(void){
    _OC3IF = 0;
}
void __attribute__((__interrupt__,__auto_psv__)) _OC4Interrupt(void){
    _OC4IF = 0;
}
//Trigger pulse
void __attribute__((__interrupt__,__auto_psv__)) _OC1Interrupt(void){
    _OC1IF = 0;
}
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void){
    _T1IF = 0;
    T1OV += 1;
}
//Time context - measures every second
void __attribute__((__interrupt__,__auto_psv__)) _T5Interrupt(void){
    #ifdef WHEEL_TEST
    if(STOP != 1){
        if(seconds < 5){
            forward();
        }
        else if(seconds >= 5 && seconds < 10){
            reverse();
        }
        else if(seconds >= 10 && seconds < 15){
            left();
        }
        else if(seconds >= 15 && seconds < 20){
            right();
        }
        else{
            seconds = 0;
        }
    }
    else{
        stop();
    }
    #endif
    seconds += 1;
    _T5IF = 0;
    seconds_passed += 1;
}
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void){
    _T2IF = 0;
}
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(void){
    _T3IF = 0;
    sent_trig = TMR1;
}
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void){
    //Rising edge is the primary side.
    _INT0IF = 0;
    if(_INT0EP == 0){
        alpha = TMR1 + (T1OV * 0xffff);
    }
    else if(_INT0EP == 1){
        beta = TMR1 + (T1OV * 0xffff);
        T1OV = 0;
        setOutput();
    }
    _INT0EP ^= 1;
}
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(void){
    //Rising edge is the primary side.
    _INT1IF = 0;
    if(_INT1EP == 0){
        gamma = TMR1 + (T1OV * 0xffff);
    }
    else if(_INT1EP == 1){
        delta = TMR1 + (T1OV * 0xffff);
        T1OV = 0;
        setOutput();
    }
    _INT1EP ^= 1;
}
//Fires on receive signal - tied to echo. This is a bad use of an input capture, 
//but it allows us to use our timer2 for boost.
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void){
    //Timestamp for reception time
    received_echo = TMR1 + (0xffff * (T1OV-sent_ov));
    _IC1IF = 0;
    //This overflow should be at most one.
    T1OV = 0;
    sent_ov = 0;
    setOutput();
}
void __attribute__((__interrupt__,__auto_psv__)) _IC2Interrupt(void){
    //Timestamp for reception time
    received_echo = TMR1 + (0xffff * (T1OV-sent_ov));
    _IC2IF = 0;
    //This overflow should be at most one.
    T1OV = 0;
    sent_ov = 0;
    setOutput();
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
//Boost Timer
void setupTMR2(void){
    T2CONbits.TON = 0;
    T2CONbits.TCKPS0 = 0;
    T2CONbits.TCKPS1 = 0;
    TMR2 = 0;
    PR2 = 399;
    _T2IF = 0;
    _T2IE = 1;
    T2CONbits.TON = 1;
}
//Input Capture timer: We are using timer two to take care of boost
void setupTMR5(void){
        //Create a one second timer
    T5CONbits.TON = 0;
    TMR5 = 0;
    T5CONbits.TCKPS0 = 1;
    T5CONbits.TCKPS1 = 1;
    PR5 = ONE_SECOND;
    _T5IF = 0;
    //_T2IP = 0b010;
    _T5IE = 1;
    T5CONbits.TON = 1;
}
//Keep track of pulse width of echo (RB7)
void setupINT0(void){
    //Turn on the interrupts
    IEC0bits.INT0IE = 1;
    //Set as high priority interrupts
    IPC0bits.INT0IP = 0b010;
    //Set edge polarity to rising edges.
    INTCON2bits.INT0EP = 0;
    //Turn off the interrupt flags.
    IFS0bits.INT0IF = 0;
}
void setupINT1(void){
    TRISBbits.TRISB13 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR0bits.INT1R = 13;  // Use Pin RP13 for interrupt
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    //Turn on the interrupts
    IEC1bits.INT1IE = 1;
    //High priority interrupt
    IPC5bits.INT1IP = 0b010;
    //Set edge polarity to rising edges.
    INTCON2bits.INT1EP = 0;
    //Turn off the interrupt flags.
    IFS1bits.INT1IF = 0;
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
//Trigger
void setupOC1(void){
    CLKDIVbits.RCDIV = 0;
    //Bind OC1 to RP6/RB6
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    TRISBbits.TRISB6 = 1;
    //TRISB = 0x0000;
    AD1PCFG = 0x9fff;
    //Set PR3 to be 20mS, this was before adding clear timers in the int func
    //and at TCKPS[1:0] = 11
    //PR3 = 149
    //Turned RCdiv to right channel
    PR3 = TRIG_WIDTH;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR3bits.RP6R = 18;  // Use Pin RP6 for Output Compare 1 = "18" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC1CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC1R = PULSE_WIDTH;   // servo start position. We won?t touch OC1R again
    OC1RS = PULSE_WIDTH;  // We will only change this once PWM is turned on
    OC1CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC1CONbits.OCTSEL = 1; // Use Timer 3 for compare source
    //enable the PWM
    //OC1CON = OC1CON | 0x0006;
    _OC1IE = 1; //Enable the OC1 Interrupt
    T3CONbits.TON = 1;
}
//OUTPUT
void setupOC2(void){
    CLKDIVbits.RCDIV = 0;
    //Bind OC1 to RP6/RB6
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    TRISBbits.TRISB8 = 1;
    //TRISB = 0x0000;
    AD1PCFG = 0x9fff;
    //Set PR3 to be 20mS, this was before adding clear timers in the int func
    //and at TCKPS[1:0] = 11
    //PR3 = 149
    //Turned RCdiv to right channel
    PR3 = TRIG_WIDTH;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR4bits.RP8R = 19;  // Use Pin RP8 for Output Compare 2 = "19" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC2CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC2R = PULSE_WIDTH;   // servo start position. We won?t touch OC1R again
    OC2RS = PULSE_WIDTH;  // We will only change this once PWM is turned on
    OC2CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC2CONbits.OCTSEL = 1; // Use Timer 3 for compare source
    //enable the PWM
    //OC1CON = OC1CON | 0x0006;
    _T3IE = 1; // Enable the T3 Interrupt
    _OC2IE = 1; //Enable the OC1 Interrupt
}
//The BOOST converter
void setupOC3(void){
    CLKDIVbits.RCDIV = 0;
    //Bind OC1 to RP6/RB6
    TRISBbits.TRISB3 = 0;
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    //Turned RCdiv to right channel
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR5bits.RP10R = 20;  // Use Pin RP9 for Output Compare 3 = "20" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC1CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    PR3 = TRIG_WIDTH;
    OC3R = PULSE_WIDTH;  // servo start position. We won?t touch OC1R again
    OC3RS = PULSE_WIDTH;  // We will only change this once PWM is turned on
    OC3CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC3CONbits.OCTSEL = 0; // Use Timer 2 for compare source
    //enable the PWM
    //OC1CON = OC1CON | 0x0006;
    _T2IE = 1; // Enable the T3 Interrupt
    _T2IF = 0;
    _OC3IE = 1; //Enable the OC1 Interrupt
    _OC3IF = 0;
    T2CONbits.TON = 1;
}
//The rear view
void setupOC4(void){
    CLKDIVbits.RCDIV = 0;
    //Bind OC1 to RP6/RB6
    TRISBbits.TRISB12 = 0;
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    //Turned RCdiv to right channel
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR6bits.RP12R = 21;  // Use Pin RP12 for Output Compare 4 = "21" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC4CON = 0;    // turn off OC1 for now
    PR3 = TRIG_WIDTH;
    //These lines simply give a starting position. Not entirely essential
    OC4R = PULSE_WIDTH;  // servo start position. We won?t touch OC1R again
    OC4RS = PULSE_WIDTH;  // We will only change this once PWM is turned on
    OC4CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC4CONbits.OCTSEL = 1; // Use Timer 3 for compare source
    //enable the PWM
    //OC1CON = OC1CON | 0x0006;
    _T3IE = 1; // Enable the T3 Interrupt
    _T3IF = 0;
    _OC4IE = 1; //Enable the OC1 Interrupt
    _OC4IF = 0;
    T3CONbits.TON = 1;
}
volatile int readings[20] = {}; //the buffer of too close readings
volatile int reading_pos = 0; //position in the buffer
volatile int read_sum; //The sum of the entries in the buffer
void setOutput(void){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    microseconds = (beta - alpha)/16; //clock cycles to microseconds
    microseconds2 = abs((gamma - delta)/16); //clock cycles to microseconds rear
    if(microseconds < 800){
        OC2RS = 2000; //microseconds to centimeters
        readings[reading_pos] = 1;
    }
    else{
        OC2RS = 0;
        readings[reading_pos] = 0;
        STOP = 0;
    }
    int i = 0;
    while(readings[i] < 21){
        read_sum += readings[reading_pos];
        i += 1;
    }
    if(read_sum > 15){
        STOP = 1;
    }
    if(reading_pos == 20){
        reading_pos = 0;
    }
    reading_pos += 1;
    read_sum = 0;
    return;
}
//setup
void setup_scan(void){
    //Make RB3 the output for the HC-SR04 Trigger, RB6 the output for the LED
    //Make RB8 the input to receive the echo signal off of the hc-sr04
    //RB3 is going to be the output
    CLKDIVbits.RCDIV = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB8 = 0;
    //PORTBbits.RB6 = 0;
    //setupIC1();  // Engage Input capture (read for echos)
    setupINT0();
    setupINT1();
    setupTMR5();
    setupTMR3();
    setupTMR2();
    setupOC4(); //Engage OC4 (TRIGGer for the rearview cam)
    setupOC1();  // Engage OC1 (setup the front trigger)
    setupOC2();  //Setup OC2 (the output for indicating circuit operation)
    //setupOC3(); //Setup OC3 (The boost converter)
    setupTMR1(); // TMR for keeping track of start and stop signal
}