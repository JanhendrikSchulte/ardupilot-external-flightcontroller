#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC

#define SITL_MCAST_IP "239.255.145.51"
#define SITL_MCAST_PORT 20721
#define SITL_SERVO_PORT 20722

#include <AP_HAL/utility/Socket_native.h>
#include <EXTERNALFC/SIM_SoloGimbal.h>
#include <EXTERNALFC/SIM_ADSB.h>
#include <EXTERNALFC/SIM_ADSB_Sagetech_MXS.h>
#include <EXTERNALFC/SIM_EFI_Hirth.h>
#include <EXTERNALFC/SIM_Vicon.h>
#include <EXTERNALFC/SIM_RF_Ainstein_LR_D1.h>
#include <EXTERNALFC/SIM_RF_Benewake_TF02.h>
#include <EXTERNALFC/SIM_RF_Benewake_TF03.h>
#include <EXTERNALFC/SIM_RF_Benewake_TFmini.h>
#include <EXTERNALFC/SIM_RF_NoopLoop.h>
#include <EXTERNALFC/SIM_RF_TeraRanger_Serial.h>
#include <EXTERNALFC/SIM_RF_JRE.h>
#include <EXTERNALFC/SIM_RF_LightWareSerial.h>
#include <EXTERNALFC/SIM_RF_LightWareSerialBinary.h>
#include <EXTERNALFC/SIM_RF_Lanbao.h>
#include <EXTERNALFC/SIM_RF_BLping.h>
#include <EXTERNALFC/SIM_RF_LeddarOne.h>
#include <EXTERNALFC/SIM_RF_RDS02UF.h>
#include <EXTERNALFC/SIM_RF_USD1_v0.h>
#include <EXTERNALFC/SIM_RF_USD1_v1.h>
#include <EXTERNALFC/SIM_RF_MaxsonarSerialLV.h>
#include <EXTERNALFC/SIM_RF_Wasp.h>
#include <EXTERNALFC/SIM_RF_NMEA.h>
#include <EXTERNALFC/SIM_RF_MAVLink.h>
#include <EXTERNALFC/SIM_RF_GYUS42v2.h>
#include <EXTERNALFC/SIM_VectorNav.h>
#include <EXTERNALFC/SIM_MicroStrain.h>
#include <EXTERNALFC/SIM_InertialLabs.h>
#include <EXTERNALFC/SIM_AIS.h>
#include <EXTERNALFC/SIM_GPS.h>

#include <EXTERNALFC/SIM_Frsky_D.h>
#include <EXTERNALFC/SIM_CRSF.h>
// #include <SITL/SIM_Frsky_SPort.h>
// #include <SITL/SIM_Frsky_SPortPassthrough.h>
#include <EXTERNALFC/SIM_PS_RPLidarA2.h>
#include <EXTERNALFC/SIM_PS_RPLidarA1.h>
#include <EXTERNALFC/SIM_PS_TeraRangerTower.h>
#include <EXTERNALFC/SIM_PS_LightWare_SF45B.h>

#include <EXTERNALFC/SIM_RichenPower.h>
#include <EXTERNALFC/SIM_Loweheiser.h>
#include <EXTERNALFC/SIM_FETtecOneWireESC.h>

#include <EXTERNALFC/SIM_ELRS.h>

#include "AP_HAL_EXTERNALFC.h"
#include "AP_HAL_EXTERNALFC_Namespace.h"
#include "HAL_EXTERNALFC_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <vector>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <EXTERNALFC/EXTERNALFC.h>
#include <EXTERNALFC/EXTERNALFC_Input.h>

class HAL_SITL;

class HALEXTERNALFC::SITL_State_Common {
    friend class HALEXTERNALFC::Scheduler;
    friend class HALEXTERNALFC::Util;
    friend class HALEXTERNALFC::GPIO;
public:
    virtual void init(int argc, char * const argv[]) = 0;

    enum vehicle_type {
        NONE,
        ArduCopter,
        Rover,
        ArduPlane,
        ArduSub,
        Blimp
    };

    // create a simulated serial device; type of device is given by
    // name parameter
    EXTERNALFC::SerialDevice *create_serial_sim(const char *name, const char *arg, const uint8_t portNumber);

    // simulated airspeed, sonar and battery monitor
    float sonar_pin_voltage;    // pin 0
    float airspeed_pin_voltage[AIRSPEED_MAX_SENSORS]; // pin 1
    float voltage_pin_voltage;  // pin 13
    float current_pin_voltage;  // pin 12
    float voltage2_pin_voltage;  // pin 15
    float current2_pin_voltage;  // pin 14

    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    bool new_rc_input;
    uint16_t pwm_output[SITL_NUM_CHANNELS];
    bool output_ready = false;

#if AP_SIM_SOLOGIMBAL_ENABLED
    // simulated gimbal
    bool enable_gimbal;
    EXTERNALFC::SoloGimbal *gimbal;
#endif

#if HAL_SIM_ADSB_ENABLED
    // simulated ADSb
    EXTERNALFC::ADSB *adsb;
#endif

#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED
    EXTERNALFC::ADSB_Sagetech_MXS *sagetech_mxs;
#endif

#if !defined(HAL_BUILD_AP_PERIPH)
    // simulated vicon system:
    EXTERNALFC::Vicon *vicon;
#endif

    // simulated Ainstein LR-D1 rangefinder:
    EXTERNALFC::RF_Ainstein_LR_D1 *ainsteinlrd1;
    // simulated Benewake tf02 rangefinder:
    EXTERNALFC::RF_Benewake_TF02 *benewake_tf02;
    // simulated Benewake tf03 rangefinder:
    EXTERNALFC::RF_Benewake_TF03 *benewake_tf03;
    //simulated JAE JRE rangefinder:
    EXTERNALFC::RF_JRE *jre;
    // simulated Benewake tfmini rangefinder:
    EXTERNALFC::RF_Benewake_TFmini *benewake_tfmini;
    //simulated NoopLoop TOFSense rangefinder:
    EXTERNALFC::RF_Nooploop *nooploop;
    // simulated TeraRanger Serial:
    EXTERNALFC::RF_TeraRanger_Serial *teraranger_serial;

    // simulated LightWareSerial rangefinder - legacy protocol::
    EXTERNALFC::RF_LightWareSerial *lightwareserial;
    // simulated LightWareSerial rangefinder - binary protocol:
    EXTERNALFC::RF_LightWareSerialBinary *lightwareserial_binary;
    // simulated Lanbao rangefinder:
    EXTERNALFC::RF_Lanbao *lanbao;
    // simulated BLping rangefinder:
    EXTERNALFC::RF_BLping *blping;
    // simulated LeddarOne rangefinder:
    EXTERNALFC::RF_LeddarOne *leddarone;
    // simulated RDS02UF rangefinder:
    EXTERNALFC::RF_RDS02UF *rds02uf;
    // simulated USD1 v0 rangefinder:
    EXTERNALFC::RF_USD1_v0 *USD1_v0;
    // simulated USD1 v1 rangefinder:
    EXTERNALFC::RF_USD1_v1 *USD1_v1;
    // simulated MaxsonarSerialLV rangefinder:
    EXTERNALFC::RF_MaxsonarSerialLV *maxsonarseriallv;
    // simulated Wasp rangefinder:
    EXTERNALFC::RF_Wasp *wasp;
    // simulated NMEA rangefinder:
    EXTERNALFC::RF_NMEA *nmea;
    // simulated MAVLink rangefinder:
    EXTERNALFC::RF_MAVLink *rf_mavlink;
    // simulated GYUS42v2 rangefinder:
    EXTERNALFC::RF_GYUS42v2 *gyus42v2;

    // simulated Frsky devices
    EXTERNALFC::Frsky_D *frsky_d;
    // EXTERNALFC::Frsky_SPort *frsky_sport;
    // EXTERNALFC::Frsky_SPortPassthrough *frsky_sportpassthrough;

#if HAL_SIM_PS_RPLIDARA2_ENABLED
    // simulated RPLidarA2:
    EXTERNALFC::PS_RPLidarA2 *rplidara2;
#endif

    // simulated FETtec OneWire ESCs:
    EXTERNALFC::FETtecOneWireESC *fetteconewireesc;
#if HAL_SIM_PS_RPLIDARA1_ENABLED
    // simulated RPLidarA1:
    EXTERNALFC::PS_RPLidarA1 *rplidara1;
#endif
#if HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED
    // simulated SF45B proximity sensor:
    EXTERNALFC::PS_LightWare_SF45B *sf45b;
#endif

#if HAL_SIM_PS_TERARANGERTOWER_ENABLED
    EXTERNALFC::PS_TeraRangerTower *terarangertower;
#endif

#if AP_SIM_CRSF_ENABLED
    // simulated CRSF devices
    EXTERNALFC::CRSF *crsf;
#endif

    // simulated VectorNav system:
    EXTERNALFC::VectorNav *vectornav;

    // simulated MicroStrain system
    EXTERNALFC::MicroStrain5 *microstrain5;

    // simulated MicroStrain system
    EXTERNALFC::MicroStrain7 *microstrain7;

    // simulated InertialLabs INS
    EXTERNALFC::InertialLabs *inertiallabs;
    
#if HAL_SIM_JSON_MASTER_ENABLED
    // Ride along instances via JSON SITL backend
    EXTERNALFC::JSON_Master ride_along;
#endif

#if HAL_SIM_AIS_ENABLED
    // simulated AIS stream
    EXTERNALFC::AIS *ais;
#endif

    // simulated EFI MegaSquirt device:
    EXTERNALFC::EFI_MegaSquirt *efi_ms;

    // simulated EFI MegaSquirt device:
    EXTERNALFC::EFI_Hirth *efi_hirth;

    // output socket for flightgear viewing
    SocketAPM_native fg_socket{true};
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    // simulated GPS devices
    EXTERNALFC::GPS *gps[AP_SIM_MAX_GPS_SENSORS];  // constrained by # of parameter sets

    // Simulated ELRS radio
    EXTERNALFC::ELRS *elrs;

    // returns a voltage between 0V to 5V which should appear as the
    // voltage from the sensor
    float _sonar_pin_voltage() const;

    // multicast state
    int mc_out_fd = -1;
    
    // send out SITL state as UDP multicast
    void multicast_state_open(void);
    void multicast_state_send(void);

    // number of times we have paused the simulation for 1ms because
    // the TCP queue is full:
    uint32_t _serial_0_outqueue_full_count;

protected:
    enum vehicle_type _vehicle;

    void sim_update(void);

    // internal SITL model
    EXTERNALFC::Aircraft *sitl_model;

    EXTERNALFC::SIM *_sitl;

    void update_voltage_current(struct sitl_input &input, float throttle);
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC
