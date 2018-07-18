/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_OATF.h"

#define OATF_SERIAL_LV_BAUD_RATE 115200

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_OATF::AP_RangeFinder_OATF(RangeFinder::RangeFinder_State &_state,
                                                                 AP_SerialManager &serial_manager,
                                                                 uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
    else
        hal.console->printf("AP_RangeFinder_OATF: uart==nullptr.\n");
}

/*
   detect if a OATF rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_OATF::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    hal.console->printf("in OATF detect\n");
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

//send a command to prompt oatf ack witch data
void AP_RangeFinder_OATF::prompt_reading(uint8_t direction, int16_t att100_singleorien)
{
    // we need to write a command to prompt another reading
    uint8_t cmd[8] = {0x55, 0x0c, 0, 0, 0, 0, 0, 0x61};
    uint8_t sumchk = 0x61;
    if(direction)
        cmd[4] = 0x01;
    cmd[5] = att100_singleorien>>8;
    cmd[6] = (uint8_t)att100_singleorien;

    sumchk += cmd[3];
    sumchk += cmd[4];
    sumchk += cmd[5];
    sumchk += cmd[6];
    cmd[7] = sumchk;
    uart->write(cmd,8);

}

// read - return last value measured by sensor
bool AP_RangeFinder_OATF::get_reading(uint8_t direction, int16_t att100_singleorien, uint16_t &oadist_cm, uint16_t &tfdist_cm)
{
    if (uart == nullptr) {
        return false;
    }

    uint8_t sum = 0x6c; //presum 0x55+0x0c+0x0b;
    int16_t nbytes = uart->available();
    uint16_t count = 0;
    hal.console->printf("nbytes: %d.\n", nbytes);

    while (nbytes-- > 0) {
        char c = uart->read();

        if(check==3){
             linebuf[count++] = c;

             if(count==PACK_SIZE_OATF-3){
                check = 4;
                break;
             }
             else{
                //sumcheck
                 sum += (uint8_t)c;
             }
        }
        //head check process
        else if((c==0x0b) && check==2){
                check = 3;
        }
        else if((c==0x0c) && check==1){
                check = 2;
        }
        else if((c==0x55) && check==0){
            if(nbytes>=10)
                check = 1;
            else
                break;
        }
        else {
            check = 0;
            //if the rest of the bytes size equal to PACK_SIZE_OATF or more, we continue.
            if(nbytes>=PACK_SIZE_OATF)
                continue;
            else
                break;
        }
    }

    //after read send a command to prompt oatf ack witch data
    prompt_reading(direction, att100_singleorien);

    //it can be check=0, 1, 2
    if (check != 4){
        return false;
    }
    if(sum!= linebuf[PACK_SIZE_OATF-4]) {
        return false;
    }
    // This OATF has multi distance output, so we have to choose wich we want.
    if(direction){
        oadist_cm = (uint16_t)linebuf[4]<<8 | linebuf[5];
    }else{
        oadist_cm = (uint16_t)linebuf[2]<<8 | linebuf[3];
    }
    tfdist_cm = (uint16_t)linebuf[1]<<8 | linebuf[0];

    check = 0;
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_OATF::update(void)
{
    OATF_Gimbal_Dir direction = OATF_FORWARD;
    int16_t pitch=0;
    uint16_t ground_clearance_cm;
    if (get_reading(direction, pitch, state.distance_cm, ground_clearance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 500) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

