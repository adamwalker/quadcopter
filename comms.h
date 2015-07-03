#ifndef COMMS_H
#define COMMS_H

void communication();
void receive();

#define TIMER_EVT     0x01
#define ACK_EVT       0x02
#define EXCEPTION_EVT 0x04

extern Thread *comms_tp;

#endif
