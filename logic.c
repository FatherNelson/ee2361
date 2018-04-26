#include "logic.h"
#include "motors.h"

int STOP;

void bluetooth_react(char c) {
    STOP = 0;
    if(STOP){
        stop();
	return;
    }

    switch(c){
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
    return;
}
