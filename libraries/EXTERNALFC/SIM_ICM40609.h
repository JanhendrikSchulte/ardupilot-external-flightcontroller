#include "SIM_config.h"

#if AP_SIM_ICM40609_ENABLED

#include "SIM_Invensense_v3.h"

namespace EXTERNALFC {

class ICM40609DevReg : public InvensenseV3DevReg {
public:
};

class ICM40609 : public InvensenseV3
{
public:
    void init() override {
        InvensenseV3::init();

        set_register(ICM40609DevReg::WHOAMI, (uint8_t)0x3b);
    }

    float accel_scale() const override { return (GRAVITY_MSS / 1024); }

private:
};

} // namespace EXTERNALFC

#endif  // AP_SIM_ICM40609_ENABLED
