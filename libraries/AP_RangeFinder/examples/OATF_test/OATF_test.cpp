/*
 * mwradar oatf test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static RangeFinder mwradar{serial_manager, ROTATION_NONE};

void setup()
{
    // print welcome message
    hal.console->printf("mwradar test\n");

    //init low level driver[] with uart port,must be here
    serial_manager.init();
    // setup for some necessary para ,only use on examples
    AP_Param::set_object_value(&mwradar, mwradar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_OATF);
    AP_Param::set_object_value(&mwradar, mwradar.var_info, "_PIN", -1.0f);
    AP_Param::set_object_value(&mwradar, mwradar.var_info, "_SCALING", 1.0f);
    AP_Param::set_object_value(&mwradar, mwradar.var_info, "_MIN_CM", 0);
    AP_Param::set_object_value(&mwradar, mwradar.var_info, "_MAX_CM", 5000);
    AP_Param::set_object_value(&serial_manager,serial_manager.var_info,"5_PROTOCOL",9);
    AP_Param::set_object_value(&serial_manager,serial_manager.var_info,"5_BAUD",115);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    mwradar.init();
    hal.console->printf("mwradar: %d devices detected\n", mwradar.num_sensors());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(200);
    mwradar.update();

    bool had_data = false;
    for (uint8_t i=0; i<mwradar.num_sensors(); i++) {
      AP_RangeFinder_Backend *sensor = mwradar.get_backend(i);
        if (sensor == nullptr) {
            hal.console->printf("oatf: sensor == nullptr.\n");
            continue;
        }
        if (!sensor->has_data()) {
            hal.console->printf("oatf: sensor has no data.\n");
            continue;
        }
        hal.console->printf("All: device_%u type %d status %d distance_cm %d\n",
                            i,
                            (int)sensor->type(),
                            (int)sensor->status(),
                            sensor->distance_cm());
        had_data = true;
    }
    if (!had_data) {
        hal.console->printf("All: no data on any sensor\n");
    }

}
AP_HAL_MAIN();

