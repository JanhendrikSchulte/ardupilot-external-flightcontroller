/*
  parent class for CAN transports in ArduPilot SITL
 */
#pragma once

#include "AP_HAL_EXTERNALFC.h"

#if HAL_NUM_CAN_IFACES

#include <AP_HAL/CANIface.h>

class CAN_Transport {
public:
    virtual ~CAN_Transport() {}
    virtual bool init(uint8_t instance) = 0;
    virtual bool send(const AP_HAL::CANFrame &frame) = 0;
    virtual bool receive(AP_HAL::CANFrame &frame) = 0;
    virtual int get_read_fd(void) const = 0;

    void set_event_handle(AP_HAL::BinarySemaphore *handle) {
        sem_handle = handle;
    }

protected:
    AP_HAL::BinarySemaphore *sem_handle;
};

#endif // HAL_NUM_CAN_IFACES
