#include "Copter.h"

#if MODE_ABZZ_ENABLED == ENABLED

/*
 * Init and run calls for abzz flight mode
 */

#define ABZZ_WP_RADIUS_CM   300


// abzz_init - initialise abzz controller flight mode
bool Copter::ModeABZz::init(bool ignore_checks)
{
    //TODO: we need to reject into this mode when the tank is empty
    if ((copter.position_ok())) {

        // initialize's loiter position and velocity on xy-axes from current pos and velocity
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // initialise position_z and desired velocity_z
        if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

//        //TODO: we also need to check if there has a valid break point
//        if (_sta_absetting=AB_POINT_CMPLT) {
//             _sta_abzz = Resume;
//        }else{
//             gcs().send_text(MAV_SEVERITY_CRITICAL, "abzz: AB point sample not complete.");

            reset_mission();
            _mode = Abzz_Manual;
            _sta_abzz = Start;
//        }

         return true;
    } else {
        gcs().send_text(MAV_SEVERITY_ERROR, "ABZZ: ekf position not lock.");
        return false;
    }
}


// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::ModeABZz::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // call the correct auto controller
    switch (_mode) {

    case Abzz_Manual:
        manual_control_run();
        break;

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

// update auto procession status  SCHED _TASK 50Hz  100us
void Copter::ModeABZz::run_autopilot()
{
    //update_abwp_sta
    update_abwp_sta();
}



void Copter::ModeABZz::update_abwp_sta()
{
    bool wp_complete = false;
    //TODO: we need to prevent the AB point location where is far away from home
    switch(_sta_abzz){

    case Start:
        if(_sta_absetting==AB_POINT_CMPLT && !_point_break.is_zero()){

            if(!_flags.ab_bearing_set){
                calc_ab_bearing();
            }
            _sta_abzz = Suspend;
            return;
        }else if(_sta_absetting==AB_POINT_CMPLT && _cmd_auto){

            //now we can start auto flight
            gcs().send_text(MAV_SEVERITY_INFO, "ab start");

            if(!_flags.ab_bearing_set){
                calc_ab_bearing();
            }
            //update loiter timer
            if(get_distance_cm(copter.current_loc, _point_shift_a)< ABZZ_WP_RADIUS_CM){
                loiter_time_max = 2;
            }
            else{
                loiter_time_max = 1;
            }
            loiter_time = 0;

             auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);

            //set start point and line cnt
            _shift_count = 0;
            generate_abline(_shift_count);

            // initialise waypoint controller
            wp_nav->wp_and_spline_init();

            if (wp_start(_point_shift_a)){
                _sta_abzz = GotoWork;
            }else{
                gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: fail to start, check AB point.");
                set_cmd_auto(false);
            }
            return;
        }


        //if in AB sample status
        save_ab_shiftdir();
        //keep in Manual flight mode
        if(_mode!=Abzz_Manual){
            manual_start();
        }
        break;

    case Suspend:
        //keep in Manual flight mode
        if(_mode!=Abzz_Manual){
            manual_start();
        }

        //app require finish
        if(_cmd_endmission){
            _cmd_endmission = false;

            //reset abzz mode and point sampling status
            _sta_absetting = SAMPLE_A;
            _sta_abzz = Start;
            return;
       }

        //execute app start or resume command is update
        if(_cmd_auto){

             if(get_distance_cm(copter.current_loc, _point_break)< ABZZ_WP_RADIUS_CM){
                 loiter_time_max = 2;
             }
             else{
                 loiter_time_max = 1;
             }
             loiter_time = 0;

             auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);

             wp_nav->wp_and_spline_init();
            //set start point to last break point
            if (wp_start(_point_break)){
                _sta_abzz = GotoWork;
            }else{
                gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: fail to start, check break point.");
                set_cmd_auto(false);
            }
            return;
        }
        break;

    case GotoWork:
         if(verify_nav_wp()){

            // convert circle_edge_neu to Location_Class
            Location_Class dest(copter.wp_nav->get_wp_destination());

            if(get_distance_cm(copter.current_loc, dest)< ABZZ_WP_RADIUS_CM){
                loiter_time_max = 2;
            }
            else{
                loiter_time_max = 1;
            }
            loiter_time = 0;
         //set fixed sepcific yaw angle according to way point direction
            auto_yaw.set_fixed_yaw(_ab_bearing_deg, 20.0f, 0, false);
            copter.wp_nav->set_speed_xy(_speed_last);
            copter.wp_nav->set_wp_destination(_point_shift_b);
            //set flight mode
            _mode = Abzz_WP;

            _sta_abzz = AToB;
         }
         break;

    case AToB:
        //destination is point B
        //it must be normal wp nav
        if(verify_nav_wp()){
            gcs().send_text(MAV_SEVERITY_INFO, "AtoB finish");
            generate_next_abline();

            auto_yaw.set_mode(AUTO_YAW_FIXED);

            loiter_time_max = 1;
            loiter_time = 0;
            copter.wp_nav->set_wp_destination(_point_shift_a);
            _sta_abzz = BToA;
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
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: BtoA finish");
            loiter_time_max = 1;
            loiter_time = 0;
            copter.wp_nav->set_wp_destination(_point_shift_b);
            _sta_abzz = AToB;
        }
        break;

    default:
        break;
    }


    //if in auto flight mode any roll or pitch stick action will cause suspend
    if(_sta_abzz >Suspend){
        if(!_cmd_auto || channel_pitch->get_control_in() > 50 ||channel_pitch->get_control_in() < -50
            ||channel_roll->get_control_in() > 50 ||channel_roll->get_control_in() < -50){
            do_exit_auto();
         }

        int16_t ch_spd = channel_speed->get_control_in();
        int16_t section;
        if(ch_spd<250){
            section = 100;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: speed=100");
        }else if(ch_spd>=250 && ch_spd <500){
            section = 300;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: speed=300");
        }else if(ch_spd>=500 && ch_spd <750){
            section = 500;
//            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: speed=500");
        }else if(ch_spd>=750){
            section = 700;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: speed=700");
        }
        if(section!=_speed_last){
            copter.wp_nav->set_speed_xy(section);
            _speed_last = section;
        }

    }


}

void Copter::ModeABZz::do_exit_auto()
{
    //record break point if necessary
    if(_sta_abzz == AToB){
        _point_break = copter.current_loc;
    }else if(_sta_abzz == BToA){
        _point_break = _point_shift_a;
    }

    _sta_abzz = Suspend;
    gcs().send_text(MAV_SEVERITY_INFO, "ABZz: Suspend");
}


void Copter::ModeABZz::set_cmd_auto(int8_t flag_auto)
{
    if(flag_auto){
        if(_sta_abzz<=Suspend && _sta_absetting==AB_POINT_CMPLT){
            _cmd_auto = true;
             gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: auto start.");
            return ;
        }
        else if(_sta_absetting!=AB_POINT_CMPLT){
            _cmd_auto = false;
            gcs().send_text(MAV_SEVERITY_ERROR, "ABZZ: AB setting not complete.");
        }
    }else{
        _cmd_auto = false;
    }
}

void Copter::ModeABZz::set_cmd_endmission(int8_t flag_end)
{
    if(flag_end){
        reset_mission();
        _sta_abzz = Start;
    }
}


void Copter::ModeABZz::save_ab_point(uint8_t get_b)
{
    //save AB point in above ekf origin altitude frame
    if(!get_b && _sta_absetting == SAMPLE_A){
        _point_a = copter.current_loc;
//        _point_a.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_ORIGIN);
        _sta_absetting = SAMPLE_B;
        gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: point A stored");

    }else if(get_b && _sta_absetting == SAMPLE_B){
        _point_b = copter.current_loc;
//        _point_b.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_ORIGIN);
        _sta_absetting = SEL_SHIFT_DIR;
        gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: point B stored");
    }
}

void Copter::ModeABZz::save_ab_shiftdir()
{
    //choose direction by roll stick moving direction
    if( _sta_absetting == SEL_SHIFT_DIR){
        if(channel_roll->get_control_in() > 300){
            _shift_direction_cw = 1;
            _sta_absetting = AB_POINT_CMPLT;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: shift right");
        }else if(channel_roll->get_control_in() < -300){
            _shift_direction_cw = 0;
            _sta_absetting = AB_POINT_CMPLT;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: shift left");
        }
    }
}


void Copter::ModeABZz::calc_ab_bearing()
{
    if(_flags.ab_brearing_reverse){
        _ab_bearing_deg = get_bearing_cd(_point_b, _point_a)/100.0f;
    }else{
        _ab_bearing_deg = get_bearing_cd(_point_a, _point_b)/100.0f;
    }
    _flags.ab_bearing_set = true;
    gcs().send_text(MAV_SEVERITY_INFO, "set AB bearing:%f", _ab_bearing_deg);
}

void Copter::ModeABZz::change_shiftwidth(uint16_t new_width_cm)
{
    //update shift width parameter
    _shift_width_cm = new_width_cm;

    if(_shift_count%2==0){
        _point_a = _point_shift_a;
        _point_b = _point_shift_b;
    }else{
         //if we reverse the AB shift point to BA, use flag to mark this exhange.
         //_flag.ab_brearing_reverse is used to track the original sample AB point.
        _point_a = _point_shift_b;
        _point_b = _point_shift_a;
        _flags.ab_brearing_reverse  = !_flags.ab_brearing_reverse;
    }

    //reset shift count after update the base AB point.
    _shift_count = 0;

}


void Copter::ModeABZz::generate_next_abline()
{
    _shift_count++;
    generate_abline(_shift_count);
}


void Copter::ModeABZz::generate_abline(uint16_t shift_cnt)
{
    //
    float shift_bearing_deg;
    if(_shift_direction_cw){
        shift_bearing_deg = wrap_360(_ab_bearing_deg+90.0f);
    }else{
        shift_bearing_deg = wrap_360(_ab_bearing_deg-90.0f);
    }

    if(shift_cnt%2==0){
        _point_shift_a = _point_a;
        _point_shift_b = _point_b;
        if(shift_cnt==0)
            return ;
    }else{
        _point_shift_a = _point_b;
        _point_shift_b = _point_a;
    }

    location_update(_point_shift_a, shift_bearing_deg, _shift_width_cm*shift_cnt/100.0f);
    location_update(_point_shift_b, shift_bearing_deg, _shift_width_cm*shift_cnt/100.0f);

}


//reset necessary mission data
void Copter::ModeABZz::reset_mission()
{
    _point_break.zero();
    _shift_count = 0;
    _sta_absetting = SAMPLE_A;
    _flags.ab_bearing_set = false;
    _flags.ab_brearing_reverse = false;
    _shift_direction_cw = 1;
    _cmd_auto = false;
    _cmd_endmission = false;

}

// manual control to the pilot
void Copter::ModeABZz::manual_start()
{
    _mode = Abzz_Manual;

    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
bool Copter::ModeABZz::wp_start(const Location_Class& dest_loc)
{

    copter.wp_nav->set_speed_xy(500);

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return false;
    }

    _mode = Abzz_WP;
    return true;


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

// manual_control - process manual control
void Copter::ModeABZz::manual_control_run()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run loiter controller
    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

    // adjust climb rate using rangefinder
    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

    // adjusts target up or down using a climb rate
    pos_control->update_z_controller();
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
