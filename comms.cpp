#include <stdint.h>
#include <stddef.h>
#include <fix.hpp>
#include <ChibiOS_ARM.h>

#include "WProgram.h"
#include "orientation.h"
#include "xbee.h"

#define ACK       0
#define TELEMETRY 1

void makeTelemetryPkt(uint8_t *pkt){
    pkt[0] = TELEMETRY;
    chMtxLock(&kalman_st_mut);
    memcpy(pkt+1, &kalman_state_global, 7*sizeof(fix16Exc));
    chMtxUnlock();
}

void communication(){
    Serial1.begin(38400, SERIAL_8N1);
    delay(3000);
    while(true){
        struct RecvdPacket pkt;
        uint8_t buffer[256];
        pkt.data = buffer;
        xbeeRecvPacket(&pkt);
    
        uint8_t telem_pkt[256];
        makeTelemetryPkt(telem_pkt);
        uint8_t tmp[256];
        size_t len = xbeeTransmitPacket(0x1234, 0x00, 0, pkt.len, pkt.data, tmp);
        int i;
        //for(i=0; i<len; i++){
        //    Serial1.write(tmp[i]);
        //}
        len = xbeeTransmitPacket(0x1234, 0x00, 0, 1 + 7*sizeof(fix16Exc), telem_pkt, tmp);
        for(i=0; i<len; i++){
            Serial1.write(tmp[i]);
        }
    }
}

