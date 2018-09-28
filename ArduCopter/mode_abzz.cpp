#include "Copter.h"

#if MODE_ABZZ_ENABLED == ENABLED

/*
 * Init and run calls for abzz flight mode
 */

// abzz_init - initialise abzz controller flight mode
bool Copter::ModeABZz::init(bool ignore_checks)
{
    if ((copter.position_ok()) || ignore_checks) {

        //set abwp status
        _abwp_sta = Start;

        // reject switching to abzz mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && ap.land_complete) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "abzz: auto takeoff not support yet.");
            return false;
        }

        if (_point_ab_sta!=AB_POINT_CMPLT) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "abzz: AB point sample not complete.");
            return false;
        }

         // initialise waypoint and spline controller
         wp_nav->wp_and_spline_init();

         // initialise yaw
        if(!_flags.ab_bearing_set){
            _ab_bearing_deg = get_bearing_cd(_point_a, _point_b)/100.0f;
            _flags.ab_bearing_set = true;
            gcs().send_text(MAV_SEVERITY_INFO, "set AB bearing:%f", _ab_bearing_deg);
        }

         return true;
    } else {
        return false;
    }
}

// initialise abzz mode's position controller
//void Copter::ModeABZz::ab__start()
//{
//    // set to position control mode
//    _mode = Abzz_WP;

//    // initialise waypoint and spline controller
//    wp_nav->wp_and_spline_init();

//    // initialise wpnav to stopping point
//    Vector3f stopping_point;
//    wp_nav->get_wp_stopping_point(stopping_point);

//    // no need to check return status because terrain data is not used
//    wp_nav->set_wp_destination(stopping_point, false);

//}


// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::ModeABZz::run()
{
    //update_abwp_sta
    update_abwp_sta();

    // call the correct auto controller
    switch (_mode) {

    case Abzz_WP:
        wp_run();
        break;

    case Abzz_Spline:
        spline_run();
        break;

    default:
        break;
    }
}

void Copter::ModeABZz::generate_next_abline()
{
    _shift_count++;
    generate_abline(_shift_count);
}


void Copter::ModeABZz::generate_abline(uint16_t shift_cnt)
{

    if(shift_cnt%2==0){
        _point_shift_a = _point_a;
        _point_shift_b = _point_b;
        if(shift_cnt==0)
            return ;
    }else{
        _point_shift_a = _point_b;
        _point_shift_b = _point_a;
    }

    if(!_flags.ab_bearing_set){
        _ab_bearing_deg = get_bearing_cd(_point_a, _point_b)/100.0f;
        _flags.ab_bearing_set = true;
    }

    float shift_bearing_deg;
    if(shift_direction_cw){
        shift_bearing_deg = wrap_360(_ab_bearing_deg+90.0f);
    }else{
        shift_bearing_deg = wrap_360(_ab_bearing_deg-90.0f);
    }

    location_update(_point_shift_a, shift_bearing_deg, _shift_width_cm*shift_cnt/100.0f);
    location_update(_point_shift_b, shift_bearing_deg, _shift_width_cm*shift_cnt/100.0f);

}


void Copter::ModeABZz::update_abwp_sta()
{
    bool wp_complete = false;
    switch(_abwp_sta){

    case Start:
        gcs().send_text(MAV_SEVERITY_INFO, "ab start");
        auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);

        _shift_count = 0;
        generate_abline(_shift_count);
        copter.wp_nav->set_wp_destination(_point_shift_a);
        _abwp_sta = GotoA;
        break;

    case AToB:
        //destination is point B
        //it must be normal wp nav
        if(verify_nav_wp()){
            gcs().send_text(MAV_SEVERITY_INFO, "AtoB finish");
            generate_next_abline();

            auto_yaw.set_mode(AUTO_YAW_FIXED);

            if(_shift_count%2){
                copter.wp_nav->set_speed_xy(500);
            }else{
                copter.wp_nav->set_speed_xy(200);
            }
            copter.wp_nav->set_wp_destination(_point_shift_a);
            _abwp_sta = BToA;
        }
        break;

    case BToA:
        if(_mode==Abzz_WP){
            wp_complete = verify_nav_wp();
        }else{
            wp_complete = true;
        }

        if(wp_complete){
            auto_yaw.set_mode(AUTO_YAW_FIXED);
            gcs().send_text(MAV_SEVERITY_INFO, "BtoA finish");
            copter.wp_nav->set_wp_destination(_point_shift_b);
            _abwp_sta = AToB;
        }
        break;

    case GotoA:
         if(verify_nav_wp()){
            loiter_time_max = 1;
//             auto_yaw.set_mode(AUTO_YAW_FIXED);
         //set fixed sepcific yaw angle according to way point direction
         auto_yaw.set_fixed_yaw(_ab_bearing_deg, 10.0f, 0, false);


             copter.wp_nav->set_wp_destination(_point_shift_b);
             _abwp_sta = AToB;
         }
         break;

    default:
        break;
    }

}


// auto_spline_start - initialises waypoint controller to implement flying to a particular destination using the spline controller
//  seg_end_type can be SEGMENT_END_STOP, SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE.  If Straight or Spline the next_destination should be provided
void Copter::ModeABZz::spline_start(const Location_Class& destination, bool stopped_at_start,
                               AC_WPNav::spline_segment_end_type seg_end_type,
                               const Location_Class& next_destination)
{
    _mode = Abzz_Spline;

    // initialise wpnav
    if (!wp_nav->set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
//    if (auto_yaw.mode() != AUTO_YAW_ROI) {
//        auto_yaw.set_mode_to_default(false);
//    }
}

uint32_t Copter::ModeABZz::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t Copter::ModeABZz::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

bool Copter::ModeABZz::get_wp(Location_Class& destination)
{
    switch (_mode) {
    case Abzz_WP:
        return wp_nav->get_wp_destination(destination);
    default:
        return false;
    }
}


// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::ModeABZz::wp_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        zero_throttle_and_relax_ac();
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(),true);
    }
}

// auto_spline_run - runs the auto spline controller
//      called by auto_run at 100hz or more
void Copter::ModeABZz::spline_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        zero_throttle_and_relax_ac();
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rat
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    wp_nav->update_spline();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}


// terrain_adjusted_location: returns a Location with lat/lon from cmd
// and altitude from our current altitude adjusted for location
//Location_Class Copter::ModeABZz::terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const
//{
//    // convert to location class
//    Location_Class target_loc(cmd.content.location);
//    const Location_Class &current_loc = copter.current_loc;

//    // decide if we will use terrain following
//    int32_t curr_terr_alt_cm, target_terr_alt_cm;
//    if (current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, curr_terr_alt_cm) &&
//        target_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, target_terr_alt_cm)) {
//        curr_terr_alt_cm = MAX(curr_terr_alt_cm,200);
//        // if using terrain, set target altitude to current altitude above terrain
//        target_loc.set_alt_cm(curr_terr_alt_cm, Location_Class::ALT_FRAME_ABOVE_TERRAIN);
//    } else {
//        // set target altitude to current altitude above home
//        target_loc.set_alt_cm(current_loc.alt, Location_Class::ALT_FRAME_ABOVE_HOME);
//    }
//    return target_loc;
//}


////do_spline_wp - initiate move to next waypoint
//void Copter::ModeABZz::do_spline_wp(const AP_Mission::Mission_Command& cmd)
//{
//    Location_Class target_loc(cmd.content.location);
//    const Location_Class &current_loc = copter.current_loc;

//    // use current lat, lon if zero
//    if (target_loc.lat == 0 && target_loc.lng == 0) {
//        target_loc.lat = current_loc.lat;
//        target_loc.lng = current_loc.lng;
//    }
//    // use current altitude if not provided
//    if (target_loc.alt == 0) {
//        // set to current altitude but in command's alt frame
//        int32_t curr_alt;
//        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
//            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
//        } else {
//            // default to current altitude as alt-above-home
//            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
//        }
//    }

//    // this will be used to remember the time in millis after we reach or pass the WP.
//    loiter_time = 0;
//    // this is the delay, stored in seconds
//    //    loiter_time_max = cmd.p1;
//    loiter_time_max = 1;

//    // determine segment start and end type
//    bool stopped_at_start = true;
//    AC_WPNav::spline_segment_end_type seg_end_type = AC_WPNav::SEGMENT_END_STOP;
//    AP_Mission::Mission_Command temp_cmd;

//    // if previous command was a wp_nav command with no delay set stopped_at_start to false
//    // To-Do: move processing of delay into wp-nav controller to allow it to determine the stopped_at_start value itself?
//    uint16_t prev_cmd_idx = copter.mission.get_prev_nav_cmd_index();
//    if (prev_cmd_idx != AP_MISSION_CMD_INDEX_NONE) {
//        if (copter.mission.read_cmd_from_storage(prev_cmd_idx, temp_cmd)) {
//            if ((temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) && temp_cmd.p1 == 0) {
//                stopped_at_start = false;
//            }
//        }
//    }

//    // if there is no delay at the end of this segment get next nav command
//    Location_Class next_loc;
//    if (cmd.p1 == 0 && copter.mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
//        next_loc = temp_cmd.content.location;
//        // default lat, lon to first waypoint's lat, lon
//        if (next_loc.lat == 0 && next_loc.lng == 0) {
//            next_loc.lat = target_loc.lat;
//            next_loc.lng = target_loc.lng;
//        }
//        // default alt to first waypoint's alt but in next waypoint's alt frame
//        if (next_loc.alt == 0) {
//            int32_t next_alt;
//            if (target_loc.get_alt_cm(next_loc.get_alt_frame(), next_alt)) {
//                next_loc.set_alt_cm(next_alt, next_loc.get_alt_frame());
//            } else {
//                // default to first waypoints altitude
//                next_loc.set_alt_cm(target_loc.alt, target_loc.get_alt_frame());
//            }
//        }
//        // if the next nav command is a waypoint set end type to spline or straight
//        if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT) {
//            seg_end_type = AC_WPNav::SEGMENT_END_STRAIGHT;
//        } else if (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
//            seg_end_type = AC_WPNav::SEGMENT_END_SPLINE;
//        }
//    }

//    // set spline navigation target
//    spline_start(target_loc, stopped_at_start, seg_end_type, next_loc);
//}


/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/



//void Copter::ModeABZz::do_change_speed(const AP_Mission::Mission_Command& cmd)
//{
//    if (cmd.content.speed.target_ms > 0) {
//        copter.wp_nav->set_speed_xy(cmd.content.speed.target_ms * 100.0f);
//    }
//}

//void Copter::ModeABZz::do_set_home(const AP_Mission::Mission_Command& cmd)
//{
//    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
//        copter.set_home_to_current_location(false);
//    } else {
//        copter.set_home(cmd.content.location, false);
//    }
//}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

// verify_nav_wp - check if we have reached the next way point
bool Copter::ModeABZz::verify_nav_wp()
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
		if (loiter_time_max > 0) {
			// play a tone
			AP_Notify::events.waypoint_complete = 1;
			}
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
		if (loiter_time_max == 0) {
			// play a tone
			AP_Notify::events.waypoint_complete = 1;
			}
        gcs().send_text(MAV_SEVERITY_DEBUG, "loiter_time_max= %i",loiter_time_max);
        return true;
    } else {
        return false;
    }
}


// verify_spline_wp - check if we have reached the next way point using spline
//bool Copter::ModeABZz::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
//{
//    // check if we have reached the waypoint
//    if ( !copter.wp_nav->reached_wp_destination() ) {
//        return false;
//    }

//    // start timer if necessary
//    if (loiter_time == 0) {
//        loiter_time = millis();
//    }

//    // check if timer has run out
//    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
//        gcs().send_text(MAV_SEVERITY_DEBUG, "loiter_time_max= %i",loiter_time_max);
//        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
//        return true;
//    } else {
//        return false;
//    }
//}

#endif
