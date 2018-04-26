#include "bluetooth.h" 
#include "p24Fxxxx.h"
#include "xc.h"
#include "robot_asm2.h"
volatile char i = 0;
void delay(int ms){
    int j = 0;
    for(j = 0; j<ms; j++){
        delay_ms();
    }
}
void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void){ 
    i = U1RXREG;
    int j = seconds;
    if(!STOP){
        switch(i){
            case 'w':
                forward();
                break;
            case 'a':
                left();
                break;
            case 's':
                reverse();
                break;
            case 'd':
                right();
                break;
            case 'x':
                stop();
                break;
        }
    }
    if(STOP){
        stop();
    }
    _U1RXIF = 0;
}
void setup_uart(void){
    TRISBbits.TRISB2 = 0;
    U1MODE = 0x800;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR18bits.U1RXR = 11;  // Use Pin RP11 = "11", for receive
    RPOR5bits.RP10R = 3; // Set the transmit line
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    U1STA = 0x00; // Interrupt flag set when RX buffer is full
    U1BRG = 103; //Baudrate of 9600
    U1STAbits.UTXEN = 1;
    _U1RXIE = 1;
    _U1RXIF = 0;
    _U1TXIE = 1;
    _U1TXIF = 0;
    U1MODE = 0x8000; //TUrn on the uart
}
