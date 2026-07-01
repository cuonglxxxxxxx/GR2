#include "robot_encoder.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
RobotEncoder::RobotEncoder(int pin_a, int pin_b, pcnt_unit_handle_t* handle, float gear_ratio)
    : _pin_a(pin_a), _pin_b(pin_b), _handle(handle), _gear_ratio(gear_ratio) {}
void RobotEncoder::init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << _pin_a) | (1ULL << _pin_b);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, _handle));
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = _pin_a,
        .level_gpio_num = _pin_b,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*_handle, &chan_config, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, 
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, 
        PCNT_CHANNEL_EDGE_ACTION_DECREASE  
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, 
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE, 
        PCNT_CHANNEL_LEVEL_ACTION_KEEP     
    ));
    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*_handle, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_enable(*_handle));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*_handle));
    ESP_ERROR_CHECK(pcnt_unit_start(*_handle));
}
float RobotEncoder::getRPM(float dt_s) {
    int cur_pulse = 0;
    pcnt_unit_get_count(*_handle, &cur_pulse);
    pcnt_unit_clear_count(*_handle); 
    float ticks = (float)cur_pulse;
    return (ticks / ENCODER_PPR) * (60.0f / dt_s);
}

