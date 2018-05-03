/* 
 * File:   robot.h
 * Author: Grant
 *
 * Created on April 12, 2018, 4:29 PM
 */

#ifndef ROBOT_H
#define	ROBOT_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define MIN_DISTANCE 4000 //if a reading is greater than this, it is non-contextual and set to this value
    
    int getFrontStatus(); // Establish the state of the front sensors, i.e. if an obstacle has been determined.
    int getBackStatus(); // Establish the state of the rear sensors, i.e. if an obstacle has been determined.

    void setUserMode(int i); //global function that simply defines whether the user is in assist mode or not
    
    extern volatile unsigned int seconds; //global time variable that tracks seconds, not pertinent to 1.0, but usable for 
    //runtime statistics in later iterations
    //extern volatile int STOP; //variable to define if currently stopped. Not used in 1.0
    void setup_motors(void); //define motor connections
    void forward(void); //drive forward
    void autoReverse(void); //cruise control forward
    void autoForward(void); //cruise control rearward     
    void reverse(void); //drive droid in rear direction
    void right(void); //turn the droid to the right
    void left(void); //turn the droid to the left
    void stop(void); //stop the droid
    void setup_scan(void); //Setup the pinout defs for sensors per the definitions in scanner.h
    void setup_uart(void); //Setup the uart pinout definitions per the definitions in bluetooth_defs.c

    long getAvgDistLeft(); //Return the average distance on the left side, not used due to hardware limitations. 
    long getAvgDistRight();//Return the average distance on the right side, not used due to hardware limitations. 

    
    void speedUp(void); //Increase the speed of the droid
    void speedDown(void); //Decrease the speed of the droid

    
    
#ifdef	__cplusplus
}
#endif

#endif	/* ROBOT_H */

