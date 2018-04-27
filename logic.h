#ifndef LOGIC_H
#define LOGIC_H


#ifdef	__cplusplus
extern "C" {
#endif

    void bluetooth_react(char);

    // each of these functions take a boolean value
    // to determine if the sensor is too close.
    // pass 0 if an obstacle is not detected and 1
    // if one is visible
    void left_tooClose(char);
    void right_tooClose(char);
    void rear_tooClose(char);

#ifdef	__cplusplus
}
#endif

#endif
