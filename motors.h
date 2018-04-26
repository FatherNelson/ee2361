/*
 * File:   motors.h
 * Author: Webster Wing 
 *
 * Created April 26, 2018, 2:48 PM
 */

#ifndef MOTORS_H
#define	MOTORS_H

#ifdef	__cplusplus
extern "C" {
#endif

    void setup_motors(void);
    void forward(void);
    void reverse(void);
    void right(void);
    void left(void);
    void stop(void);

#ifdef	__cplusplus
}
#endif

#endif	

