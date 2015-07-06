#include <stdint.h>
#include <stddef.h>
#include <fix.hpp>
#include <ChibiOS_ARM.h>

#include "WProgram.h"
#include "orientation.h"
#include "xbee.h"
#include "comms.h"

#define ACK       0
#define TELEMETRY 1
#define EXCEPTION 2

#define PING      0
#define CONTROL   1
#define GAINS     2
#define RESET     3
#define ARM       4
#define DISARM    5

size_t makeAckPkt(uint8_t *pkt){
    pkt[0] = ACK;
    return 1;
}

size_t makeExcPkt(uint8_t *pkt){
    pkt[0] = EXCEPTION;
    return 1;
}

size_t makeTelemetryPkt(uint8_t *pkt){
    pkt[0] = TELEMETRY;
    chMtxLock(&kalman_st_mut);
    memcpy(pkt+1, &kalman_state_global, 7*sizeof(fix16Exc));
    chMtxUnlock();
    return 1 + 7*sizeof(fix16Exc);
}

Thread *comms_tp = NULL;

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

extern unsigned int last_command;
extern bool armed;

void parsePacket(size_t len, uint8_t *data){
    switch(data[0]){
        case PING:
            if(comms_tp) chEvtSignal(comms_tp, (eventmask_t) ACK_EVT);
            break;
        case CONTROL:
            chMtxLock(&control_mut);
            memcpy(&current_control, data+1, sizeof(struct control_inputs<fix16Exc>));
            last_command = millis();
            chMtxUnlock();
            break;
        case GAINS:
            chMtxLock(&control_mut);
            memcpy(&current_gains, data+1, sizeof(struct gains<fix16Exc>));
            chMtxUnlock();
            break;
        case RESET:
            CPU_RESTART;
            break;
        case ARM:
            armed = true;
            break;
        case DISARM:
            armed = false;
            break;
    }
}

VirtualTimer vt;

void timer_callback(void *p){
    chSysLockFromIsr();
    chVTSetI(&vt, MS2ST(100), timer_callback, p);
    if(comms_tp != NULL){
        chEvtSignalI(comms_tp, (eventmask_t)1);
    }
    chSysUnlockFromIsr();
}

void communication(){
    comms_tp = chThdSelf();
    chSysLock();
    chVTSetI(&vt, MS2ST(100), timer_callback, NULL);
    chSysUnlock();

    while(true){
        
        eventmask_t event = chEvtWaitAny((eventmask_t) 3);

        if(event & TIMER_EVT){
            uint8_t telem_pkt[256];
            size_t len = makeTelemetryPkt(telem_pkt);
            uint8_t tmp[256];
            int i;
            len = xbeeTransmitPacket(0x1234, 0x00, 0, len, telem_pkt, tmp);
            for(i=0; i<len; i++){
                Serial1.write(tmp[i]);
            }
        }

        if(event & ACK_EVT){
            uint8_t ack_pkt[256];
            size_t len = makeAckPkt(ack_pkt);
            uint8_t tmp[256];
            int i;
            len = xbeeTransmitPacket(0x1234, 0x00, 0, len, ack_pkt, tmp);
            for(i=0; i<len; i++){
                Serial1.write(tmp[i]);
            }
        }

        if(event & EXCEPTION_EVT){
            uint8_t exc_pkt[256];
            size_t len = makeExcPkt(exc_pkt);
            uint8_t tmp[256];
            int i;
            len = xbeeTransmitPacket(0x1234, 0x00, 0, len, exc_pkt, tmp);
            for(i=0; i<len; i++){
                Serial1.write(tmp[i]);
            }
        }
    }
}

void receive(){
    while(true){
        struct RecvdPacket pkt;
        uint8_t buffer[256];
        pkt.data = buffer;
        xbeeRecvPacket(&pkt);
        parsePacket(pkt.len, buffer);
    }
}
