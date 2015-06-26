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

Mutex kalman_st_mut;
fix16Exc kalman_state_global[7];

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

            chMtxLock(&kalman_st_mut);
            memcpy(&kalman_state_global, &kalman_state.state, 7*sizeof(fix16Exc));
            chMtxUnlock();
        }
    } else {
        Serial.printf("overflow exception %d\n", exc);
    }
}
