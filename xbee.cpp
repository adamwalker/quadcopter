#include <stdint.h>
#include <stddef.h>

#include <utility/ch.h>
#include "WProgram.h"
#include "xbee.h"

//Assumes the output buffer is large enough
size_t xbeeEscape(size_t len_in, uint8_t *in, uint8_t *out){
    size_t i;
    size_t j;
    for(i=0, j=0; i<len_in; i++, j++){
        if(in[i]==0x7e || in[i]==0x7d || in[i]==0x11 || in[i]==0x13){
            out[j] = 0x7d;
            j++;
            out[j] = 0x20 ^ in[i];
        } else {
            out[j] = in[i];
        }
    }
    return j;
}

uint8_t xbeeChecksum(size_t len, uint8_t *data){
    uint8_t sum = 0;
    size_t i;
    for(i=0; i<len; i++){
        sum += data[i];
    }
    return sum;
}

//Assumes the output buffer is large enough
size_t xbeeTransmitPacket(uint16_t address, uint8_t options, uint8_t frame_id, size_t data_len, uint8_t *data, uint8_t *packet){
    packet[0] = 0x7e;

    uint8_t tmp[256];
    tmp[0] = 0x00;
    tmp[1] = 5 + data_len;
    tmp[2] = 0x01;
    tmp[3] = frame_id;
    tmp[4] = (address >> 8);
    tmp[5] = (address & 0xff);
    tmp[6] = options;

    size_t i;
    for(i=0; i<data_len; i++){
        tmp[7+i] = data[i];
    }

    tmp[7+data_len] = 255 - xbeeChecksum(5 + data_len, tmp + 2);

    int length = xbeeEscape(8 + data_len, tmp, packet + 1);
    return length + 1;
}

uint8_t getByte(){
    while(Serial1.available() <= 0){
        chEvtWaitAny((eventmask_t) 1);
    };
    return Serial1.read();
}

void xbeeRecvPacket(struct RecvdPacket *pkt){
    int byte;

    while(true){

        byte = getByte();
        if(byte != 0x7e) continue;

got_start:

        byte = getByte();
        if(byte == 0x7e) goto got_start;
        if(byte == 0x7d) byte = 0x20 ^ getByte();
        if(byte != 0x00) continue;

        uint8_t lsb = getByte();
        if(lsb == 0x7e) goto got_start;
        if(lsb == 0x7d) lsb = 0x20 ^ getByte();

        uint8_t tmp[256];
        int i;
        for(i=0; i<lsb+1; i++){
            byte = getByte();
            if(byte == 0x7e) goto got_start;
            if(byte == 0x7d) byte = 0x20 ^ getByte();
            tmp[i] = byte;
        }

        if(tmp[0] != 0x81) continue;

        if(xbeeChecksum(lsb+1, tmp) != 255) continue;

        pkt->sourceAddr = (tmp[1] << 8) | tmp[2];
        pkt->rssi       = tmp[3];
        pkt->options    = tmp[4];

        pkt->len = lsb - 5;
        for(i=0; i<lsb - 5; i++){
            pkt->data[i] = tmp[i+5];
        }

        return;
    }
}

