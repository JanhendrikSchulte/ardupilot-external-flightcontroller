#pragma once

#include "SIM_config.h"

#if AP_SIM_MAXSONAR_I2C_XL_ENABLED

#include "SIM_I2CDevice.h"

namespace EXTERNALFC {

class MaxSonarI2CXL : public I2CDevice, public I2CCommandResponseDevice
{
public:

    // methods to make I2CCommandResponseDevice happy:
    uint8_t command_take_reading() const override { return 0x51; }
    uint16_t reading() const override {
        return rangefinder_range * 100;
    };

    void update(const class Aircraft &aircraft) override {
        rangefinder_range = aircraft.rangefinder_range();
    }

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CCommandResponseDevice::rdwr(data);
    }


private:

    float rangefinder_range;
};

} // namespace EXTERNALFC

#endif  // AP_SIM_MAXSONAR_I2C_XL_ENABLED
