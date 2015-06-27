#include <stdint.h>
#include <stddef.h>
#include <fix.hpp>
#include <ChibiOS_ARM.h>

#include "WProgram.h"
#include "orientation.h"
#include "xbee.h"

#define ACK       0
#define TELEMETRY 1
#define EXCEPTION 2

#define PING      0
#define ARM       1

//void decodePacket(size_t len, uint8_t *pkt){
//    if(pkt[0] == PING){
//    } else if (pkt[0] == ARM) {
//    } else {
//    }
//}

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

VirtualTimer vt;
Thread *comms_tp = NULL;

void timer_callback(void *p){
    chSysLockFromIsr();
    chVTSetI(&vt, MS2ST(100), timer_callback, p);
    if(comms_tp != NULL){
        chEvtSignalI(comms_tp, (eventmask_t)1);
    }
    chSysUnlockFromIsr();
}

void communication(){
    Serial1.begin(38400, SERIAL_8N1);
    delay(3000);

    comms_tp = chThdSelf();
    chSysLock();
    chVTSetI(&vt, MS2ST(100), timer_callback, NULL);
    chSysUnlock();

    while(true){
        
        eventmask_t event = chEvtWaitAny((eventmask_t) 3);

        if(event & 0x1){
            uint8_t telem_pkt[256];
            size_t len = makeTelemetryPkt(telem_pkt);
            uint8_t tmp[256];
            int i;
            len = xbeeTransmitPacket(0x1234, 0x00, 0, len, telem_pkt, tmp);
            for(i=0; i<len; i++){
                Serial1.write(tmp[i]);
            }
        }

        if(event & 0x2){
            uint8_t ack_pkt[256];
            size_t len = makeAckPkt(ack_pkt);
            uint8_t tmp[256];
            int i;
            len = xbeeTransmitPacket(0x1234, 0x00, 0, len, ack_pkt, tmp);
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

        if(comms_tp) chEvtSignal(comms_tp, (eventmask_t) 2);
    }
}
