#ifndef XBEE_H
#define XBEE_H

struct RecvdPacket {
    uint16_t sourceAddr;
    uint8_t  rssi;
    uint8_t  options;
    size_t   len;
    uint8_t  *data;
};

void xbeeRecvPacket(struct RecvdPacket *pkt);
size_t xbeeTransmitPacket(uint16_t address, uint8_t options, uint8_t frame_id, size_t data_len, uint8_t *data, uint8_t *packet);

#endif
