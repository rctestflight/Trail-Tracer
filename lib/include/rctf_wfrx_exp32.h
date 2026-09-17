#ifndef INC_RCTF_WFRX_ESP32_H_
#define INC_RCTF_WFRX_ESP32_H_

class StuckVehicle{
    
    public:
        StuckVehicle(float*, float, float, float);
        int GetStatus();
        void ResetStatus();
        void SetThreshold(float);
        void SetTimeout(float);

    private:
        float *pos_y_pointer;  // pointer to y position variable
        float threshold;
        float timeout;
        float cooldown;
        unsigned long last_millis;
        float max_y;
        float min_y;
        bool stuck_timer_reset;
        int stuck_status;
};

#endif // INC_RCTF_WFRX_ESP32_H_