#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC
#include "AP_HAL_EXTERNALFC.h"
#include <AP_ESC_Telem/AP_ESC_Telem_EXTERNALFC.h>

class HALEXTERNALFC::RCOutput : public AP_HAL::RCOutput {
public:
    explicit RCOutput(SITL_State *sitlState): _sitlState(sitlState), _freq_hz(50) {}
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t* period_us, uint8_t len) override;
    void cork(void) override;
    void push(void) override;

    /*
      force the safety switch on, disabling PWM output from the IO board
     */
    bool force_safety_on(void) override {
        safety_state = AP_HAL::Util::SAFETY_DISARMED;
        return true;
    }
    /*
      force the safety switch off, enabling PWM output from the IO board
     */
    void force_safety_off(void) override {
        safety_state = AP_HAL::Util::SAFETY_ARMED;
    }

    /*
      get safety switch state, used by Util.cpp
    */
    AP_HAL::Util::safety_state _safety_switch_state(void) { return safety_state; }

    /*
      Serial LED emulation
     */
    bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) override;
    bool set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) override;
    bool serial_led_send(const uint16_t chan) override;
    
private:
    SITL_State *_sitlState;
    AP_ESC_Telem_SITL *esc_telem;

    uint16_t _freq_hz;
    uint32_t _enable_mask;
    bool _corked;
    uint16_t _pending[SITL_NUM_CHANNELS];

    AP_HAL::Util::safety_state safety_state = AP_HAL::Util::safety_state::SAFETY_DISARMED;
};

#endif
