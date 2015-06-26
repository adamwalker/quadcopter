#ifndef ORIENTATION_H
#define ORIENTATION_H

extern Mutex kalman_st_mut;
extern fix16Exc kalman_state_global[7];

void orientation();

#endif
