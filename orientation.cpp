#include <stdint.h>
#include "WProgram.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

#include <ChibiOS_ARM.h>

#include <fix.hpp>
#include <printing.hpp>
#include <imu.hpp>

#include "orientation.h"
#include "led.h"
#include "control.h"
#include "comms.h"

Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12346);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(12347);

Mutex kalman_st_mut;
fix16Exc kalman_state_global[7];

bool armed = false;

Mutex control_mut;
struct control_inputs<fix16Exc> current_control;
struct gains<fix16Sat>          current_gains;
unsigned int last_command = 0;

Thread *i2c_tp = NULL;

Thread *orient_tp = NULL;
struct measurements<float> meas;

void get_sensors(){
    i2c_tp = chThdSelf();
    struct measurements<float> meas_local;

    Serial.print("accel begin\r\n");
    if(!accel.begin()){
        Serial.print("Accelerometer not detected\n");
    }
    Serial.print("sensors\r\n");

    if(!mag.begin()){
        Serial.print("Magnetometer not detected\n");
    }

    if(!gyro.begin()){
        Serial.print("Gyro not detected\n");
    }

    while(true){
        sensors_event_t event;
        int i;

        while(!mag.getEvent(&event));
        for(i=0; i<3; i++){
            meas_local.body_mag[i] = event.magnetic.v[i];
        }

        while(!accel.getEvent(&event));
        for(i=0; i<3; i++){
            meas_local.body_grav[i] = event.acceleration.v[i];
        }

        while(!gyro.getEvent(&event));
        for(i=0; i<3; i++){
            meas_local.body_gyro[i] = event.gyro.v[i];
        }

        if(orient_tp != NULL) {
            memcpy(&meas, &meas_local, sizeof(struct measurements<float>));
            //Serial.printf("%f %f %f\r\n", meas.body_gyro[0], meas.body_gyro[1],meas.body_gyro[2]);
            chMsgSend(orient_tp, NULL);
        } else {
            chThdSleepMilliseconds(100);
        }
    }
}

template <class T>
void get_measurements(struct measurements<T> *meass){
    jmp_buf jb;
    jmp_buf *last_jb = overflow_exc;
    overflow_exc = &jb;
    int exc;

    struct measurements<float> meas_local;

    chMsgWait();
    memcpy(&meas_local, &meas, sizeof (struct measurements<float>));
    chMsgRelease(i2c_tp, NULL);

    if(!(exc = setjmp(jb))){
        int i;

        for(i=0; i<3; i++){
            meass->body_mag[i] = meas_local.body_mag[i];
        }
        for(i=0; i<3; i++){
            meass->body_grav[i] = meas_local.body_grav[i];
        }
        for(i=0; i<3; i++){
            meass->body_gyro[i] = meas_local.body_gyro[i];
        }

        normalize(3, meass->body_mag,  meass->body_mag);
        normalize(3, meass->body_grav, meass->body_grav);
    } else {
        Serial.printf("getMeasurements exception\r\n");
        overflow_exc = last_jb;
        longjmp(*overflow_exc, exc);
    }

    overflow_exc = (jmp_buf *)last_jb;
}

template <class T>
void calibrate(struct calibration<T> *calib){
    int i, j;
    struct measurements<T> meas;

    for(j=0; j<3; j++){
        calib->earth_mag[j] = 0;
        calib->earth_grav[j] = 0;
        calib->stat_gyro[j] = 0;
    }

    for(i=0; i<100; i++){
        get_measurements(&meas);

        for(j=0; j<3; j++)
            calib->earth_mag[j] += meas.body_mag[j];

        for(j=0; j<3; j++)
            calib->earth_grav[j] += meas.body_grav[j];

        for(j=0; j<3; j++)
            calib->stat_gyro[j] += meas.body_gyro[j];

        chThdSleepMilliseconds(30);
    }

    normalize(3, calib->earth_mag, calib->earth_mag);
    normalize(3, calib->earth_grav, calib->earth_grav);
    divv((T)100, 3, calib->stat_gyro, calib->stat_gyro);
}

void set_motors(fix16Sat out[4]){
    analogWrite(20, out[0].val >> 23);
    analogWrite(21, out[1].val >> 23);
    analogWrite(22, out[2].val >> 23);
    analogWrite(23, out[3].val >> 23);
}

void orientation(){
    orient_tp = chThdSelf();

    //Wire.begin();
    sensors_event_t event;

    current_control.throttle = 0;
    current_control.orientation[0] = 1;
    current_control.orientation[1] = 0;
    current_control.orientation[2] = 0;
    current_control.orientation[3] = 0;

    current_gains.throttle_gain = 16386;
    current_gains.horiz_plane_p = 16386;
    current_gains.yaw_p         = 16386;
    current_gains.horiz_plane_d = 16386;
    current_gains.yaw_d         = 16386;
    
    jmp_buf jb;
    overflow_exc = &jb;
    int exc;

    struct calibration<fix16Exc> calibration;
    while(true){
        if(!(exc = setjmp(jb))){
            calibrate(&calibration);
            break;
        } else {
            Serial.printf("overflow exception %d\n", exc);
            if(comms_tp) chEvtSignal(comms_tp, (eventmask_t) EXCEPTION_EVT);
        }
    }

    chEvtSignal(led_tp, (eventmask_t)1);

    while(true){
        if(!(exc = setjmp(jb))){
            struct params<fix16Exc> parameters = {2, 40, 0.3, 0.05};
            struct gn_weights<fix16Exc> weights = {20, 1};

            unsigned int last_millis = micros() - 100000;

            struct kalman_state<fix16Exc> kalman_state;
            init_kalman_state(fix16Exc(0), fix16Exc(0), &kalman_state);

            while(true){
                struct measurements<fix16Exc> measurements;
                get_measurements(&measurements);

                int i;
                for(i=0; i<3; i++){
                    measurements.body_gyro[i] = measurements.body_gyro[i] - calibration.stat_gyro[i];
                }

                unsigned int millis_now = micros();
                unsigned int diff       = millis_now - last_millis;
                last_millis    = millis_now;

                Serial.printf("rate: %d\r\n", 1000000 / diff);

                imu(&calibration, &parameters, &weights, &measurements, &kalman_state, fix16Exc((int)diff / 100) / fix16Exc(10000));

                //print_vect(4, kalman_state.state);
                //Serial.printf("\r\n%d\r\n", millis_now);
                //Serial.printf("$%f,%f,%f,%f\r\n", fix16_to_float(kalman_state.state[0].val), fix16_to_float(kalman_state.state[1].val), fix16_to_float(kalman_state.state[2].val), fix16_to_float(kalman_state.state[3].val));

                chMtxLock(&kalman_st_mut);
                memcpy(&kalman_state_global, &kalman_state.state, 7*sizeof(fix16Exc));
                chMtxUnlock();

                //Control the motors
                struct gains<fix16Sat>          the_gains;
                struct control_inputs<fix16Sat> control_req;

                chMtxLock(&control_mut);
                memcpy(&the_gains,   &current_gains,   sizeof(struct gains<fix16Sat>));
                memcpy(&control_req, &current_control, sizeof(struct control_inputs<fix16Sat>));
                chMtxUnlock();

                fix16Sat out[4];
                control(&the_gains, (fix16Sat *)kalman_state.state, &control_req, out);

                for(i=0; i<4; i++){
                    if(out[i].val < 0) out[i] = 0;
                }
                //Serial.printf("$%x,%x,%x,%x\r\n", out[0].val >> 23, out[1].val >> 23, out[2].val >> 23, out[3].val >> 23);

                if((millis() > last_command + 1000) || !armed){
                    for(i=0; i<4; i++) out[i] = 0;
                } 
                set_motors(out);
            }
        } else {
            Serial.printf("overflow exception %d\n", exc);
            if(comms_tp) chEvtSignal(comms_tp, (eventmask_t) EXCEPTION_EVT);
        }
    }
}
