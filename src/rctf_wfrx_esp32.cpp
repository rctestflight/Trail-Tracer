
#include "Arduino.h"
#include "rctf_wfrx_exp32.h"

StuckVehicle::StuckVehicle(float* Pos_y, float Threshold, float Timeout, float Cooldown){
    pos_y_pointer = Pos_y;
    threshold = Threshold;
    timeout = Timeout;
    cooldown = Cooldown;
    stuck_timer_reset = true;
}

/*
Check if the vehicle is stuck based on the minumum and maximum Y position values over a given period
return 0 => not stuck
return 1 => stuck
*/
int StuckVehicle::GetStatus(){
    float pos_y = *pos_y_pointer;  // get the latest y position value from the pointer
    if(stuck_timer_reset == true){
        // return 'not stuck' on first run, start millis timier
        stuck_timer_reset = false;
        max_y = pos_y;
        min_y = pos_y;
        last_millis = millis();
        stuck_status = 0;
        return stuck_status;
    }else{
        if((millis() - last_millis) > (timeout + cooldown) || (millis() - last_millis) < timeout){
            // if timer has not tripepd, or has tripped and cooldown has elapsed
            max_y = (pos_y > max_y) ? pos_y : max_y;  // update the maximum y position value of this period
            min_y = (pos_y < min_y) ? pos_y : min_y;  // update the minimum y position value of this period
        }else{
            // if timer has tripped and cooldown has not elapsed, gurantee nothing happens
            max_y = pos_y;
            min_y = pos_y;
        }

        if((max_y - min_y) > threshold){
            // reset timer if threshold is exceeded
            stuck_timer_reset = true;
            return stuck_status;
        }

        if((millis() - last_millis) > timeout){
            // If timeout expires without reset...
            stuck_status = 1;
            return stuck_status;
        }

        // default to last status
        return stuck_status;
    }
}

void StuckVehicle::ResetStatus(){
    stuck_timer_reset = true;
}

void StuckVehicle::SetThreshold(float Threshold){
    threshold = Threshold;
}

void StuckVehicle::SetTimeout(float Timeout){
    timeout = Timeout;
}

