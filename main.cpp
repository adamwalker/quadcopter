#include <stdint.h>
#include "WProgram.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

#include <ChibiOS_ARM.h>

#include <fix.hpp>
#include <printing.hpp>
#include <imu.hpp>

Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12346);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(12347);

template <class T>
void get_measurements(struct measurements<T> *meas){
    jmp_buf jb;
    jmp_buf *last_jb = overflow_exc;
    overflow_exc = &jb;
    int exc;

    if(!(exc = setjmp(jb))){
        sensors_event_t event;
        int i;

        while(!mag.getEvent(&event)){
            Serial.print("mag.getEvent failed\r\n");
        }
        for(i=0; i<3; i++){
            meas->body_mag[i] = event.magnetic.v[i];
        }

        while(!accel.getEvent(&event)){
            Serial.print("accel.getEvent failed\r\n");
        }
        for(i=0; i<3; i++){
            meas->body_grav[i] = event.acceleration.v[i];
        }

        while(!gyro.getEvent(&event)){
            Serial.print("gyro.getEvent failed\r\n");
        }
        for(i=0; i<3; i++){
            meas->body_gyro[i] = event.gyro.v[i];
        }

        normalize(3, meas->body_mag,  meas->body_mag);
        normalize(3, meas->body_grav, meas->body_grav);
    } else {
        Serial.printf("getMeasurements exception\r\n");
        overflow_exc = last_jb;
        longjmp(*overflow_exc, exc);
    }

    overflow_exc = (jmp_buf *)last_jb;
}

template <class T>
void calibrate(struct calibration<T> *calib){
    sensors_event_t event;
    int i, j;

    for(j=0; j<3; j++){
        calib->earth_mag[j] = 0;
        calib->earth_grav[j] = 0;
        calib->stat_gyro[j] = 0;
    }

    for(i=0; i<100; i++){
        while(!mag.getEvent(&event)){
            Serial.print("mag.getEvent failed\r\n");
        }
        normalize(3, event.magnetic.v, event.magnetic.v);
        for(j=0; j<3; j++)
            calib->earth_mag[j] += event.magnetic.v[j];

        while(!accel.getEvent(&event)){
            Serial.print("accel.getEvent failed\r\n");
        }
        normalize(3, event.acceleration.v, event.acceleration.v);
        for(j=0; j<3; j++)
            calib->earth_grav[j] += event.acceleration.v[j];

        while(!gyro.getEvent(&event)){
            Serial.print("gyro.getEvent failed\r\n");
        }
        normalize(3, event.gyro.v, event.gyro.v);
        for(j=0; j<3; j++)
            calib->stat_gyro[j] += event.gyro.v[j];

        chThdSleepMilliseconds(30);
    }

    normalize(3, calib->earth_mag, calib->earth_mag);
    normalize(3, calib->earth_grav, calib->earth_grav);
    divv((T)100, 3, calib->stat_gyro, calib->stat_gyro);
}

void orientation(){
    //Wire.begin();
    sensors_event_t event;

    if(!accel.begin()){
        Serial.print("Accelerometer not detected\n");
    }

    if(!mag.begin()){
        Serial.print("Magnetometer not detected\n");
    }

    if(!gyro.begin()){
        Serial.print("Gyro not detected\n");
    }
    
    jmp_buf jb;
    overflow_exc = &jb;
    int exc;
    if(!(exc = setjmp(jb))){
        struct calibration<fix16Exc> calibration;
        calibrate(&calibration);

        Serial.printf("Mag:   %f %f %f\r\n", calibration.earth_mag[0], calibration.earth_mag[1], calibration.earth_mag[2]);
        Serial.printf("Accel: %f %f %f\r\n", calibration.earth_grav[0], calibration.earth_grav[1], calibration.earth_grav[2]); 
        Serial.printf("Gyro:  %f %f %f\r\n", calibration.stat_gyro[0], calibration.stat_gyro[1], calibration.stat_gyro[2]); 

        struct params<fix16Exc> parameters = {1, 10, 0.3, 0.1};
        unsigned int last_millis = millis() - 100;

        struct kalman_state<fix16Exc> kalman_state;
        init_kalman_state(fix16Exc(0), fix16Exc(0), &kalman_state);

        while(true){
            struct measurements<fix16Exc> measurements;
            get_measurements(&measurements);

            /*
            int i;
            for(i=0; i<3; i++){
                measurements.body_gyro[i] = measurements.body_gyro[i] - calibration.stat_gyro[i];
            }
            */

            int millis_now = millis();
            int diff       = millis_now - last_millis;
            last_millis    = millis_now;

            imu(&calibration, &parameters, &measurements, &kalman_state, fix16Exc(diff) / fix16Exc(1000));

            //print_vect(4, kalman_state.state);
            //Serial.printf("\r\n%d\r\n", millis_now);
            Serial.printf("$%f,%f,%f,%f\r\n", fix16_to_float(kalman_state.state[0].val), fix16_to_float(kalman_state.state[1].val), fix16_to_float(kalman_state.state[2].val), fix16_to_float(kalman_state.state[3].val));
        }
    } else {
        Serial.printf("overflow exception %d\n", exc);
    }
}

static WORKING_AREA(wa_flash_thread, 128);
static msg_t flash_thread(void *arg){
	pinMode(13, OUTPUT);
	while (1) {
		digitalWriteFast(13, HIGH);
        chThdSleepMilliseconds(100);
		digitalWriteFast(13, LOW);
        chThdSleepMilliseconds(100);
	}
}

static WORKING_AREA(wa_orientation_thread, 8192);
static msg_t orientation_thread(void *arg){
    Serial.printf("Starting orientation task\r\n");
    orientation();
}

extern "C" void mainFunc(){
    Serial.printf("mainFunc\r\n");

    chThdCreateStatic(wa_flash_thread, sizeof(wa_flash_thread), NORMALPRIO, flash_thread, NULL);
    chThdCreateStatic(wa_orientation_thread, sizeof(wa_orientation_thread), NORMALPRIO, orientation_thread, NULL);
}

extern "C" void loop(){
    chThdSleepMilliseconds(100);
}

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
    while(Serial1.available() <= 0){};
    return Serial1.read();
}

struct RecvdPacket {
    uint16_t sourceAddr;
    uint8_t  rssi;
    uint8_t  options;
    size_t   len;
    uint8_t  *data;
};

void recvPacket(struct RecvdPacket *pkt){
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

extern "C" int main(void) {
    Serial.begin(38400);
    delay(3000);
    Serial.printf("Hello World\r\n");

    Serial1.begin(38400, SERIAL_8N1);
    delay(3000);
    while(true){
        struct RecvdPacket pkt;
        uint8_t buffer[256];
        pkt.data = buffer;
        recvPacket(&pkt);
    
        Serial.println(pkt.sourceAddr, HEX);
        Serial.println(pkt.rssi, HEX);
        Serial.println(pkt.options, HEX);
        
        int i;
        for(i=0; i<pkt.len; i++){
            Serial.printf("%c", pkt.data[i]);
        }

        uint8_t tmp[256];
        size_t len = xbeeTransmitPacket(0xFFFF, 0x01, 0, pkt.len, pkt.data, tmp);
        for(i=0; i<len; i++){
            Serial1.write(tmp[i]);
        }
    }

    chBegin(mainFunc);
}

