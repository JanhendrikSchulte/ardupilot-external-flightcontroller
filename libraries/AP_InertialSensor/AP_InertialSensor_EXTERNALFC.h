#pragma once

#include "AP_InertialSensor.h"

#if AP_SIM_INS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC

#include "AP_InertialSensor_Backend.h"

// simulated sensor rates in Hz. This matches a pixhawk1
const uint16_t INS_SITL_SENSOR_A[] = { 1000, 1000 };
const uint16_t INS_SITL_SENSOR_B[] = { 760, 800 };

#include <EXTERNALFC/EXTERNALFC.h>

class AP_InertialSensor_SITL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_SITL(AP_InertialSensor &imu, const uint16_t sample_rates[]);

    /* update accel and gyro state */
    bool update() override;
    void start() override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu, const uint16_t sample_rates[]);

private:
    bool init_sensor(void);
    void timer_update();
    float gyro_drift(void) const;
    void generate_accel();
    void generate_gyro();
    float get_temperature(void);
    void update_file();
#if AP_SIM_INS_FILE_ENABLED
    void read_gyro(const float* buf, uint8_t nsamples);
    void read_gyro_from_file();
    void write_gyro_to_file(const Vector3f& gyro);
    void read_accel(const float* buf, uint8_t nsamples);
    void read_accel_from_file();
    void write_accel_to_file(const Vector3f& accel);
#endif
	EXTERNALFC::SIM *sitl;

    const uint16_t gyro_sample_hz;
    const uint16_t accel_sample_hz;

    uint64_t next_gyro_sample;
    uint64_t next_accel_sample;
    float gyro_time;
    float accel_time;
    float gyro_motor_phase[32];
    float accel_motor_phase[32];
    uint32_t temp_start_ms;
#if AP_SIM_INS_FILE_ENABLED
    int gyro_fd = -1;
    int accel_fd = -1;
#endif

    static uint8_t bus_id;
};
#endif // AP_SIM_INS_ENABLED
