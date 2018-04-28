/*
 * File:   servo_main.c
 * Author: Grant
 *
 * Created on April 27, 2018, 9:43 PM
 */


/*
 * File:   udstr005_lab04_defs_v001.c
 * Author: Grant
 *
 * Created on February 21, 2018, 1:33 AM
 */
#include "xc.h"
//1249 gives 20mS pulses
#define PWM_WIDTH 1249
#define CALIBRATION_WIDTH 50
#define START 625
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
volatile unsigned int servo_periods = 0;
#define SERVO_POSITIONS 50
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(void){
    if(servo_periods< 2*CALIBRATION_WIDTH){
        OC1RS = servo_periods;
    }
    else if(servo_periods < 4*CALIBRATION_WIDTH){
        OC1RS = (4*CALIBRATION_WIDTH) - servo_periods;
    }
    else{
        OC1RS = 0;
        servo_periods = 1;
    }
    servo_periods+=1;
    _T3IF = 0;
}
void setupTMR3(void){
    T3CONbits.TON = 0;
    TMR3 = 0;
    T3CONbits.TCKPS1 = 1;
    T3CONbits.TCKPS0 = 1;
    PR3 = PWM_WIDTH;
    _T3IE = 1; // Enable the T3 Interrupt
    T3CONbits.TON = 1;
}
void initServo(void){
    CLKDIVbits.RCDIV = 0;
    //Bind OC1 to RP6/RB6
    //Set timer to have period 20ms
    //Set to digital and make 6,8,9 outputs
    TRISBbits.TRISB2 = 0;
    AD1PCFG = 0x9fff;
    T3CONbits.TON = 0;
    TMR3 = 0;
    PR3 = PWM_WIDTH;
    T3CONbits.TCKPS1 = 1;
    T3CONbits.TCKPS0 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR1bits.RP2R = 18;  // Use Pin RP6 for Output Compare 1 = "18" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    // Timer 3 setup should happen before this line
    OC1CON = 0;    // turn off OC1 for now
    //These lines simply give a starting position. Not entirely essential
    OC1R = START;   // servo start position. We won?t touch OC1R again
    OC1RS = START;  // We will only change this once PWM is turned on
    OC1CONbits.OCM = 0b110; // Output compare PWM w/o faults
    OC1CONbits.OCTSEL = 1; // Use Timer 3 for compare source
    //enable the PWM
    OC1CON = OC1CON | 0x0006;
    _T3IE = 1; // Enable the T3 Interrupt
    _OC1IE = 1; //Enable the OC1 Interrupt
    T3CONbits.TON = 1;
}
int main(void) {
    setupTMR3();
    initServo();
    while(1){
        //setServo(85); //85 is center of the current servo, use this to find the
        //correct calibration for ninety degree turn radius
        //setServo(CALIBRATION_WIDTH);
    }
    return 0;
}
