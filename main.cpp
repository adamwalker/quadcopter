#include <stdint.h>
#include "WProgram.h"
#include <ChibiOS_ARM.h>
#include <fix.hpp>

#include "xbee.h"
#include "orientation.h"
#include "comms.h"
#include "led.h"

static WORKING_AREA(wa_orientation_thread, 8192);
static msg_t orientation_thread(void *arg){
    Serial.begin(38400);
    delay(3000);
    orientation();
}

Thread *tp = NULL;

static WORKING_AREA(wa_communication_thread, 4096);
static msg_t communication_thread(void *arg){
    //tp = chThdSelf();
    communication();
}

static WORKING_AREA(wa_receive_thread, 4096);
static msg_t receive_thread(void *arg){
    tp = chThdSelf();
    receive();
}

static WORKING_AREA(wa_flash_thread, 128);
static msg_t flash_thread(void *arg){
    led();
}

static WORKING_AREA(wa_led_thread, 128);
static msg_t led_thread(void *arg){
    int val = 0;
    while(1){
        if(val > 255) val = 0;
        analogWrite(23, val);
        analogWrite(22, val);
        analogWrite(21, val);
        analogWrite(20, val);
        val+=1;
        chThdSleepMilliseconds(10);
    }
}

extern "C" void mainFunc(){
    Serial.printf("mainFunc\r\n");

    chMtxInit(&kalman_st_mut);

    //chThdCreateStatic(wa_led_thread,           sizeof(wa_led_thread),           NORMALPRIO+3, led_thread,           NULL);
    chThdCreateStatic(wa_flash_thread,         sizeof(wa_flash_thread),         NORMALPRIO+3, flash_thread,         NULL);
    chThdCreateStatic(wa_communication_thread, sizeof(wa_communication_thread), NORMALPRIO+2, communication_thread, NULL);
    chThdCreateStatic(wa_receive_thread,       sizeof(wa_receive_thread),       NORMALPRIO+1, receive_thread,       NULL);
    chThdCreateStatic(wa_orientation_thread,   sizeof(wa_orientation_thread),   NORMALPRIO,   orientation_thread,   NULL);
}

extern "C" void loop(){
    chThdSleepMilliseconds(10000);
}


extern "C" int main(void) {
    chBegin(mainFunc);
}

