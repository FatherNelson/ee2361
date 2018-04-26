/*
 * File:   motors.h
 * Author: Webster Wing 
 *
 * Created April 26, 2018, 2:50 PM
 */

#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

#ifdef	__cplusplus
extern "C" {
#endif

    void setup_uart(void);
    extern void (*process_function)(char);

#ifdef	__cplusplus
}
#endif

#endif

