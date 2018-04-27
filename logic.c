#include "logic.h"
#include "motors.h"

char hazard_right = 0;
char hazard_left = 0;
char hazard_rear = 0;

void bluetooth_react(char c) {
    hazard_right = 0;
    hazard_left = 0;
    hazard_rear = 0;   
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

void left_tooClose(char c) {
    if(c) {
	if(!hazard_left) {
	    stop();
	    hazard_left = 1;
	}
    } else {
	hazard_left = 0;
    }
}

void right_tooClose(char c) {
    if(c) {
	if(!hazard_right) {
	    stop();
	    hazard_right = 1;
	}
    } else {
	hazard_right = 0;
    }
}

void rear_tooClose(char c) {
    if(c) {
	if(!hazard_rear) {
	    stop();
	    hazard_rear = 1;
	}
    } else {
	hazard_rear = 0;
    }
}
