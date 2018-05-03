#include "robot.h" 
#include "p24Fxxxx.h"
#include "xc.h"
#include "robot_asm2.h"
volatile char i = 0; //A place holder for the contents sent to the PIC

static unsigned int userMode = 0; //If 1, in assisted driving mode, if 0, not

void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void){ 
    i = U1RXREG;
        switch(i){
            case 'w':
                if(!userMode){
                _INT0IE = 1; //Front right sensor
                _INT1IE = 0; //Rear sensor
                _INT2IE = 0; //Front left sensor
                }
                if(getFrontStatus()&&(!userMode)){ //Check the front sensors, if they are "hot" and we are not in userMode
                    //then break, that is, do not proceed forward.
                    break;
                }
                forward(); //Go to OC_MOTO and adjust the H-bridge for a forward motion
                break;
            case 'e':
                if(!userMode){ //Engages the front sensors if the user is in assisted mode. Note that the stable version
                    //could only handle one sensor in the front being on due to hardware constraints.
                _INT0IE = 1;
                _INT1IE = 0;
                _INT2IE = 0;
                }
                autoForward(); //OC_MOTO function that says "continue forward until you see an obstacle"
                break; 
            case 'q':
                if(!userMode){
                _INT0IE = 1;
                _INT1IE = 0;
                _INT2IE = 0;
                }
                autoReverse(); //OC_MOTO function that says "continue reverse until you see an obstacle"
                break;      
            case 'a':
                _INT0IE = 0; //Turns off the sensors due to the nature of slip steer making the user turn with a zero degree 
                //radius
                _INT1IE = 0;
                _INT2IE = 0;
                left();  //OC_MOTO function "turn left"
                //stop();
                break;
            case 's': 
                if(!userMode){ //Same checks as above, only this time check the rear sensor if being assisted
                _INT0IE = 0;
                _INT1IE = 1;
                _INT2IE = 0;
                }
                if(getBackStatus()&&(!userMode)){
                    break;
                }
                reverse(); //OC_MOTO function to drive in reverse relative the front grill.
                break;
            case 'd':
                _INT0IE = 0;
                _INT1IE = 0;
                _INT2IE = 0;
                right(); //OC_MOTO function, turn right. No sensors because of the nature of slip-steering
                //stop();
                break;
            case 'u':
                _INT0IE = 0;
                _INT1IE = 0;
                _INT2IE = 0;
                userMode = 1; //Set the userMode variable to a value that dictates the driving assistance as on
                setUserMode(userMode);
                stop(); //stop the vehicle
                stop();
                break;
            case 'h':
                _INT0IE = 1;
                _INT1IE = 0;
                _INT2IE = 0;
                userMode = 0; //Set the userMode to a value that says "user mode off (assistance on)"
                setUserMode(userMode);
                stop();
                stop();
                break;
            case 'p':
                speedUp(); //Adjust the speed in the OC_MOTO file to a faster one
              //  stop();
                //stop();
                break;
            case 'o':
                speedDown(); //Adjust the speed in the OC_MOTO file to a slower one
                //stop();
                //stop();
                break;  
                
            case 'x':
                stop();  //Pure old vanilla "stop". Causes all tire rotation to end.
                break;
        }
    

    _U1RXIF = 0; //Boiler plate "clear the interrupt"
}
void setup_uart(void){
    TRISBbits.TRISB2 = 0;
    U1MODE = 0x800;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR18bits.U1RXR = 11;  // Use Pin RP11 = "11", for receive
    RPOR5bits.RP10R = 3; // Set the transmit line as RP10
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    U1STA = 0x00; // Interrupt flag set when RX buffer is full
    U1BRG = 103; //Baudrate of 9600
    U1STAbits.UTXEN = 1; //Engage uart
   // forward();
    _U1RXIE = 1; //Engage uart interrupts
    _U1RXIF = 0; //Clear the uart interrupt flag
    _U1TXIE = 1; //Engae the transmit interrupts
    _U1TXIF = 0; //Clear the transmission flag
    U1MODE = 0x8000; //TUrn on the uart
}
