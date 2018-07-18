#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define PACK_SIZE_OATF 11

// OATF gimbal direction enum
enum OATF_Gimbal_Dir {
    OATF_FORWARD = 0,
    OATF_BACKWARD = 1
};

class AP_RangeFinder_OATF : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_OATF(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager,
                                   uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    // get a reading
    bool get_reading(uint8_t direction, int16_t att100_singleorien, uint16_t &oadist_cm, uint16_t &tfdist_cm);
    void prompt_reading(uint8_t direction, int16_t att100_singleorien);
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    uint8_t linebuf[PACK_SIZE_OATF];
    uint8_t check = 0;

};

