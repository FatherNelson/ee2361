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
    //#define WHEEL_TEST 1
    
#define MIN_DISTANCE 4000
    
    int getFrontStatus();

    int getBackStatus();

    
    void setUserMode(int i);
    
    extern volatile unsigned int seconds;
    extern volatile int STOP;
    void setup_motors(void);
    void forward(void);
    void autoReverse(void);
    void autoForward(void);      
    void reverse(void);
    void right(void);
    void left(void);
    void stop(void);
    void setup_scan(void);
    void setup_uart(void);

    long getAvgDistLeft();
    
    long getAvgDistRight();

    
    void speedUp(void);
    void speedDown(void);

    
    
#ifdef	__cplusplus
}
#endif

#endif	/* ROBOT_H */

