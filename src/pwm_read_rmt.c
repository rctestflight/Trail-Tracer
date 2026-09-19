#include "pwm_read_rmt.h"
#include "esp_attr.h"

int32_t *pwm_read_dur = NULL;
uint8_t pwm_in_num_channels = 0;

static void IRAM_ATTR rmt_isr_handler(void* arg)
{
    uint32_t intr_st = RMT.int_st.val;

    for(int i = 0; i < pwm_in_num_channels; i++)
    {
        uint8_t channel = i;
        uint32_t channel_mask = BIT(channel*3+1);

        if (!(intr_st & channel_mask)) continue;

        RMT.conf_ch[channel].conf1.rx_en = 0;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
        volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;
        if (item) {
            pwm_read_dur[i] = item->duration0;
        }

        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[channel].conf1.rx_en = 1;

        //clear RMT interrupt status.
        RMT.int_clr.val = channel_mask;
    }
}

void pwm_read_rmt_init(uint8_t channelPins[], uint8_t numberOfPins)
{
    assert(numberOfPins <= 8);
    pwm_in_num_channels = numberOfPins;
    pwm_read_dur = (int32_t *) malloc(numberOfPins * sizeof(int32_t));

    for(int i = 0; i < pwm_in_num_channels; i++)
    {
        rmt_config_t rmt_configuration = RMT_DEFAULT_CONFIG_RX(channelPins[i], i);
        rmt_config(&rmt_configuration);
        rmt_set_clk_div(i, 8);
        rmt_set_rx_idle_thresh(i, 25000);
        rmt_set_rx_intr_en(i, true);
        rmt_rx_start(i, 1);
    }

    rmt_isr_register(rmt_isr_handler, NULL, 0, NULL);
}

int32_t pwm_read_rmt_dur(uint8_t channel)
{   
    return (pwm_read_dur[channel] - PWM_INPUT_OFFSET) / 10;
}

void pwm_reset_readings(){
    for(int i = 0; i < pwm_in_num_channels; i++)
    {
        pwm_read_dur[i] = PWM_INPUT_OFFSET;
    }
}