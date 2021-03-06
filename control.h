#ifndef CONTROL_H
#define CONTROL_H

template <class T>
struct control_inputs {
    T throttle;
    T orientation[4];
};

template <class T>
struct gains {
    T throttle_gain;
    T horiz_plane_p;
    T yaw_p;
    T horiz_plane_d;
    T yaw_d;
};

/*
 * Motor Layout:
 *
 * 1 2
 * 3 4
 *
 * Sensor axes:
 * X is up
 * Y is right
 * Z is out of screen
 *
 * Moment about X axis: k_h (1 + 2 - 3 - 4)
 * Moment about Y axis: k_h (1 - 2 + 3 - 4)
 * Moment about Z axis: k_y (1 - 2 - 3 + 4)
 * Throttle:            k_t (1 + 2 + 3 + 4)
 *
 * Increase X, keep others constant: add(1, 2), sub(3, 4)
 * Increase Y, keep others constant: add(1, 3), sub(2, 4)
 * Increase Z, keep others constant: add(1, 4), sub(2, 3)
 * Increase T, keep others constant: add(1, 2, 3, 4)
 *
 * rotate_error -> rotate_control = rotation
 * rotation     = rotate_control * rotate_error
 * rotate_error = rotate_control^-1 * rotation 
 *
 */

template <class T>
void control(struct gains<T> *gains, T *state, struct control_inputs<fix16Sat> *c_in, T *out){
    T error[3];
    T control_inv[4];

    control_inv[0] = c_in->orientation[0];
    int i;
    for(i=1; i<4; i++){
        control_inv[i] = -c_in->orientation[i];
    }

    mul_q_no_1st_out(control_inv, state, error);
    
    //X moment
    out[0] -= gains->horiz_plane_p * error[0];
    out[1] -= gains->horiz_plane_p * error[0];
    out[2] += gains->horiz_plane_p * error[0];
    out[3] += gains->horiz_plane_p * error[0];

    //Y moment
    out[0] -= gains->horiz_plane_p * error[1];
    out[1] += gains->horiz_plane_p * error[1];
    out[2] -= gains->horiz_plane_p * error[1];
    out[3] += gains->horiz_plane_p * error[1];
    
    //Z moment
    out[0] -= gains->yaw_p * error[2];
    out[1] += gains->yaw_p * error[2];
    out[2] += gains->yaw_p * error[2];
    out[3] -= gains->yaw_p * error[2];

    //Throttle
    out[0] += gains->throttle_gain * c_in->throttle;
    out[1] += gains->throttle_gain * c_in->throttle;
    out[2] += gains->throttle_gain * c_in->throttle;
    out[3] += gains->throttle_gain * c_in->throttle;
    
    //Serial.printf("$%f,%f,%f,%f\r\n", fix16_to_float(out[0].val), fix16_to_float(out[1].val), fix16_to_float(out[2].val), fix16_to_float(out[3].val));
}


#endif
