#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "control.h"

extern Mutex kalman_st_mut;
extern fix16Exc kalman_state_global[7];

void orientation();

extern struct control_inputs<fix16Exc> current_control;
extern Mutex control_mut;
extern struct gains<fix16Sat> current_gains;

#endif
