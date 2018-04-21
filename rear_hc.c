/*
 * File:   rear_hc.c
 * Author: Grant
 *
 * Created on April 19, 2018, 5:44 PM
 */


#include "xc.h"
#include "p24Fxxxx.h"
#define TRIG_WIDTH 3759 //Value in PR3 for both trigger signals.
#define TRIG_PAT 3758
#define TRIG_PAT1 3756
#define START 20
//Needs this to reset 
// PIC24FJ64GA002 Configuration Bit Settings
// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary oscillator disabled. 
					// Primary Oscillator refers to an external osc connected to the OSC1 and OSC2 pins)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // OSC2/CLKO/RC15 functions as port I/O (RC15)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))
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
    TRISBbits.TRISB12 = 0;
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
    RPOR6bits.RP12R = 18;  // Use Pin RP12 for Output Compare 1 = "18" (Table 10-3)
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
//Trigger Front
void setupOC2(void){
    CLKDIVbits.RCDIV = 0;
    //Bind OC1 to RP6/RB6
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    TRISBbits.TRISB6 = 0;
    //TRISB = 0x0000;
    AD1PCFG = 0x9fff;
    //Set PR3 to be 20mS, this was before adding clear timers in the int func
    //and at TCKPS[1:0] = 11
    //PR3 = 149
    //Turned RCdiv to right channel
    PR3 = TRIG_WIDTH;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR3bits.RP6R = 19;  // Use Pin RP6 for Output Compare 2 = "19" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC2CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC2R = TRIG_PAT1;   // servo start position. We won?t touch OC1R again
    OC2RS = TRIG_PAT1;  // We will only change this once PWM is turned on
    OC2CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC2CONbits.OCTSEL = 1; // Use Timer 3 for compare source
    //enable the PWM
    //OC1CON = OC1CON | 0x0006;
    _OC2IE = 1; //Enable the OC1 Interrupt
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
//void setupIC2(void){
//    TRISBbits.TRISB13 = 1;
//    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
//    RPINR7bits.IC2R = 13;  // Use Pin RP13 for interrupt
//    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
//    IC2CONbits.ICTMR = 0b100;//Selected timer one
//    IC2CONbits.ICI = 00; //interrupt on every capture event
//    IC2CONbits.ICM = 011; //Interrupt one every rising edge
//    _IC2IF = 0;
//    _IC2IE = 1;
//}
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
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void){
    //Rising edge is the primary side.
    if(_INT0EP == 0){
        alpha = TMR1 + (T1OV * 0xffff);
        sent_ov = T1OV;
    }
    else if(_INT0EP == 1){
        beta = TMR1 + (T1OV * 0xffff);
        sent_ov = 0;
        T1OV = 0;
        setOutput();
    }
    _INT0EP ^= 1;
    _INT0IE = 1;
    _INT0IF = 0;
}
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
        setOutput2();
    }
    _INT1EP ^= 1;
    _INT1IE = 1;
    _INT1IF = 0;
}
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
        setOutput();
    }
    _INT2EP ^= 1;
    _INT2IE = 1;
    _INT2IF = 0;
}
//Keep track of pulse width of echo (RB7)
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
volatile unsigned int STOP = 0; //Tells us whether to brake or not
volatile unsigned int read_sum; //Like a check sum, tell me how scary the last readings were
void setOutput(void){
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
    if(microseconds < 800 && microseconds > 50){
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
    if(read_sum < 3){
        PORTBbits.RB7 = 0;
        STOP = 1;
    }
    else{
        PORTBbits.RB7 = 1;
        STOP = 0;
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
volatile unsigned int STOP2 = 0; //Tells us whether to brake or not
volatile unsigned int read_sum2; //Like a check sum, tell me how scary the last readings were
void setOutput2(void){
    //Times three was added so that the light gets brighter at greater distances.
    //A value of zero is 2cm or less and a value of 23200 (RAW) should be 
    //400cm. Microseconds is the delay in microseconds between TX and Rx.
    // Dividing by 58 gives the distance in centimeters.
    /*We get seventeen readings a second, if the last 4 are all bad, we may assume something is wrong*/
    //microseconds = (beta - alpha)/16; //clock cycles to microseconds
    microseconds2 = (delta - gamma)/16; //clock cycles to microseconds rear
    if(microseconds2 < 800 && microseconds > 50){
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
    if(read_sum2 < 3){
        PORTBbits.RB8 = 0;
        STOP2 = 1;
    }
    else{
        PORTBbits.RB8 = 1;
        STOP2 = 0;
    }
    reading_pos2 += 1;
    if(reading_pos2 == 10){
        reading_pos2 = 0;
    }
    read_sum2 = 0;
    return;
}
void setup(void){
    setupINT2();
    setupINT1();
    setupTMR1();
    //setupOC1();
    setupOC2();
    setupTMR3();
    setupTMR5();
}
int main(void) {
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;
    PORTBbits.RB12 = 1;
    PORTBbits.RB14 = 1;
    PORTBbits.RB4 = 1;
    setup();
    while(1){

    }
    return 0;
}
