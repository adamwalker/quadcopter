#include <stdint.h>
#include "WProgram.h"
#include <ChibiOS_ARM.h>
#include <fix.hpp>

#include "xbee.h"
#include "orientation.h"
#include "comms.h"

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

Thread *tp = NULL;

static WORKING_AREA(wa_communication_thread, 2048);
static msg_t communication_thread(void *arg){
    tp = chThdSelf();
    communication();
}

extern "C" void mainFunc(){
    Serial.printf("mainFunc\r\n");

    chMtxInit(&kalman_st_mut);

    chThdCreateStatic(wa_flash_thread,         sizeof(wa_flash_thread),         NORMALPRIO+2, flash_thread,         NULL);
    chThdCreateStatic(wa_communication_thread, sizeof(wa_communication_thread), NORMALPRIO+1, communication_thread, NULL);
    chThdCreateStatic(wa_orientation_thread,   sizeof(wa_orientation_thread),   NORMALPRIO,   orientation_thread,   NULL);
}

extern "C" void loop(){
    chThdSleepMilliseconds(100);
}


extern "C" int main(void) {
    Serial.begin(38400);
    delay(3000);
    Serial.printf("Hello World\r\n");

    chBegin(mainFunc);
}

