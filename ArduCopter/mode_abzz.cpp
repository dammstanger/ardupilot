#include "Copter.h"

#if MODE_ABZZ_ENABLED == ENABLED

/*
 * Init and run calls for abzz flight mode
 */

#define ABZZ_WP_RADIUS_CM   300
#define ABZZ_YAW_LOOK_DISTANCE_MIN_CM   800



// abzz_init - initialise abzz controller flight mode
bool Copter::ModeABZz::init(bool ignore_checks)
{
    // TODO: we need to reject into this mode when the AB point and necessary work set is not complete
    if(!copter.position_ok()){
         gcs().send_text(MAV_SEVERITY_ERROR, "[05300]ABZZ: position is not OK.");
         _sta_abzz = StandBy;
         return false;
    }

    // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
    if (!motors->armed() || ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_ERROR, "[05301]ABZZ: need Takeoff.");
        _sta_abzz = StandBy;
        return false;
    }

    if(_sta_absetting!=AB_POINT_CMPLT){
        switch(_sta_absetting){
            case SAMPLE_A:
                gcs().send_text(MAV_SEVERITY_ERROR, "[05302]ABZZ: need A and B point.");
                break;
            case SAMPLE_B:
                gcs().send_text(MAV_SEVERITY_ERROR, "[05303]ABZZ: need B point.");
                break;
            case SEL_SHIFT_DIR:
                gcs().send_text(MAV_SEVERITY_ERROR, "[05304]ABZZ: need shift direction.");
                break;
            case GET_AB_BEAR_REV:
                gcs().send_text(MAV_SEVERITY_ERROR, "[05305]ABZZ: need bear rev to recover.");
                break;

            default:
                break;
            }
        _sta_abzz = StandBy;
        return false;
    }

    // TODO: we also need to check if there has a valid break point
    if(!check_ab_point_validity()) return false;

    gcs().send_text(MAV_SEVERITY_INFO, "A:=%d, B=%d, cnt=%d", _point_a.lng,_point_b.lng, _shift_count);

    //if break point is valid means we should continue the task
    if(check_break_point_validity()){
        _sta_abzz = Resume;
    }else{
        start_mission();
        _sta_abzz = Start;
        _sta_abzz_last = Start;
    }

    //we need to set _flags.save_when_exit to false every time we begin abzzmode
    //it will enable work status being saved when exit abzz mode in unexpected situation.
    _flags.save_when_exit = true;

    //calculate yaw
    if(!_flags.ab_bearing_set){
        calc_ab_bearing();
    }

    // initialize's loiter position and velocity on xy-axes from current pos and velocity
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise position_z and desired velocity_z
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    //init sprayer system to adapt auto mode
    set_sprayer_auto();

    // initialise waypoint and spline controller
    _maxspeed_cms = wp_nav->get_speed_xy();
    wp_nav->wp_and_spline_init();

    //controller mode should be set to loiter to prevent run in unexperted status
    //before the low **run_autopilot** loop  initialise the WP_Nav
    _mode = Abzz_Loiter;

    return true;
}


// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 50hz which handles decision making and non-navigation related commands
void Copter::ModeABZz::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // call the correct auto controller
    switch (_mode) {

    case Abzz_Loiter:
        loiter_run();
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

// update auto processing status  SCHED _TASK 50Hz  100us
void Copter::ModeABZz::run_autopilot()
{
    //update_abwp_sta
    update_abwp_sta();

    //update_work_speed
    update_work_speed();
}

void Copter::ModeABZz::update_abwp_sta()
{
    bool wp_complete = false;
    switch(_sta_abzz){

    case StandBy:
        handle_RC_exit_ab_mode();
    break;

    case Start:{
            //now we can start auto flight
            gcs().send_text(MAV_SEVERITY_INFO, "[05600]ab start");

            //
            if(!generate_next_abline()){
                brake_and_exit(false);
                _sta_abzz = StandBy;
                _sta_abzz_last = Start;
                return ;
            }

            if (wp_start(_point_shift_a)){
                //update loiter timer with destination distance
                if(wp_distance()< ABZZ_WP_RADIUS_CM){
                    loiter_time_max = 2;
                }
                else{
                    loiter_time_max = 1;
                }
                loiter_time = 0;

                //update yaw
                auto_yaw.set_fixed_yaw(wp_bearing()/100.0f, 20.0f, 0, false);
                
                _sta_abzz = GotoWork;
                _mode = Abzz_WP;
            }else{
                gcs().send_text(MAV_SEVERITY_ERROR, "[05306]ABZZ: terrain follow failed.");
                brake_and_exit(true);
                _sta_abzz = StandBy;
            }
            _sta_abzz_last = Start;
        }
        break;

    case Resume:{
            //set start point to last break point
            if (wp_start(_point_break)){
                //update loiter timer with destination distance
                if(wp_distance()< ABZZ_WP_RADIUS_CM){
                    loiter_time_max = 2;
                }
                else{
                    loiter_time_max = 1;
                }
                loiter_time = 0;

                //update yaw
                auto_yaw.set_fixed_yaw(wp_bearing()/100.0f, 20.0f, 0, false);

                _sta_abzz = GotoWork;
                _mode = Abzz_WP;
            }else{
                gcs().send_text(MAV_SEVERITY_ERROR, "[05306]ABZZ: terrain follow failed.");
                brake_and_exit(true);
                _sta_abzz = StandBy;
            }
            gcs().send_text(MAV_SEVERITY_DEBUG, "ABZZ: Resume.");
            _sta_abzz_last = Resume;
        }
        break;

    case GotoWork:
         if(verify_nav_wp()){
            if(set_next_wp(_point_shift_b)){
                //update loiter timer with destination distance
                if(wp_distance()< ABZZ_WP_RADIUS_CM){
                    loiter_time_max = 2;
                }
                else{
                    loiter_time_max = 1;
                }
                loiter_time = 0;

                //update yaw
                auto_yaw.set_fixed_yaw(_ab_bearing_deg, 20.0f, 0, false);

                _sta_abzz = AToB;
                //enable sprayer
                operate_sprayer(true);
            }else{
                gcs().send_text(MAV_SEVERITY_ERROR, "[05306]ABZZ: terrain follow failed.");
                brake_and_exit(true);
                _sta_abzz = StandBy;
            }
            _sta_abzz_last = GotoWork;

            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: GotoWork.");
         }else{
            if(handle_RC_exit_ab_mode()){
                 _sta_abzz = StandBy;
                 _sta_abzz_last = GotoWork;
            }
         }

         break;

    case AToB:
        //destination is point B
        //it must be normal wp nav
        if(verify_nav_wp()){
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: AtoB finish");
            
            if(!generate_next_abline()){
                brake_and_exit(false);
                _sta_abzz = StandBy;
                _sta_abzz_last = AToB;
                return ;
            }

            auto_yaw.set_mode(AUTO_YAW_FIXED);

            loiter_time_max = 1;
            loiter_time = 0;

            if(set_next_wp(_point_shift_a)){
                _sta_abzz = BToA;
                //enable sprayer
                operate_sprayer(false);
            }else{
                gcs().send_text(MAV_SEVERITY_ERROR, "[05306]ABZZ: terrain follow failed.");
                brake_and_exit(true);
                _sta_abzz = StandBy;
            }
            _sta_abzz_last = AToB;
        }else{
            if(handle_RC_exit_ab_mode()){
                 _sta_abzz = StandBy;
                 _sta_abzz_last = GotoWork;
                 gcs().send_text(MAV_SEVERITY_DEBUG, "ABZZ: exit from AToB");
            }
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
            gcs().send_text(MAV_SEVERITY_DEBUG, "ABZZ: BtoA finish");
            loiter_time_max = 1;
            loiter_time = 0;

            if(set_next_wp(_point_shift_b)){
                _sta_abzz = AToB;
                //enable sprayer
                operate_sprayer(false);
            }else{
                gcs().send_text(MAV_SEVERITY_ERROR, "[05306]ABZZ: terrain follow failed.");
                brake_and_exit(true);
                _sta_abzz = StandBy;
            }
            _sta_abzz_last = BToA;
        }else{
            if(handle_RC_exit_ab_mode()){
                 _sta_abzz = StandBy;
                 _sta_abzz_last = GotoWork;
                 gcs().send_text(MAV_SEVERITY_DEBUG, "ABZZ: exit from BToA");
            }
         }
        break;

    default:
        break;
    }

//        copter.wp_nav->set_speed_xy(section);
//----------------------------------
        //change speed
//        int16_t ch_spd = channel_speed->get_control_in();
//        int16_t section;
//        if(ch_spd<250){
//            section = 100;
//        }else if(ch_spd>=250 && ch_spd <480){
//            section = 300;
//        }else if(ch_spd>=480 && ch_spd <750){
//            section = 500;
//        }else if(ch_spd>=750){
//            section = 800;
//        }
//        if(section!=_pilot_desired_speed_cms){
//            _pilot_desired_speed_cms = section;
//        }

//----------------------------------


}

//exit from normal work flow, we need to save the breakpoint
void Copter::ModeABZz::record_breakpoint()
{
    //record break point if necessary
    if(_sta_abzz == AToB){
        _point_break = copter.current_loc;
    }else if(_sta_abzz == BToA){
        Location_Class temp_loc(_point_shift_a);
        _point_break = temp_loc;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "[05601]ABZz: break_point recorded");
}

void Copter::ModeABZz::set_breakpoint(Location_Class& brk_point)
{
    _point_break = brk_point;
}

//exit from normal work flow, we need to save the work info
void Copter::ModeABZz::record_workinfo_for_beacon_msg()
{
    copter.beaconParams.breakPointLongitude = _point_break.lng;
    copter.beaconParams.breakPointLatitude = _point_break.lat;
    
    if(_point_shift_a.is_zero() || _point_shift_b.is_zero()){
        copter.beaconParams.aPointLongitude = _point_a.lng;
        copter.beaconParams.aPointLatitude = _point_a.lat;
        copter.beaconParams.bPointLongitude = _point_b.lng;
        copter.beaconParams.bPointLatitude = _point_b.lat;
    }else{
        Location_Class temp_loc(_point_shift_a);
        copter.beaconParams.aPointLongitude = temp_loc.lng;
        copter.beaconParams.aPointLatitude = temp_loc.lat;
        Location_Class temp_loc2(_point_shift_b);
        copter.beaconParams.bPointLongitude = temp_loc2.lng;
        copter.beaconParams.bPointLatitude = temp_loc2.lat;
    }

    if(_flags.ab_bearing_reverse)
        copter.beaconParams.breakDirection = 1;
    else copter.beaconParams.breakDirection = 0;
}

void Copter::ModeABZz::send_beacon_msg()
{
    gcs().send_message(MSG_BEACON_A_POINT);
    gcs().send_message(MSG_BEACON_B_POINT);
    gcs().send_message(MSG_BEACON_BREAKPOINT);
}

bool Copter::ModeABZz::handle_RC_exit_ab_mode()
{
    // any roll or pitch stick action will cause suspend
    if(channel_pitch->get_control_in() > 100 ||channel_pitch->get_control_in() < -100
        ||channel_roll->get_control_in() > 100 ||channel_roll->get_control_in() < -100){
        if(brake_and_exit(true)) return true;
        else return false;
    }
    return false;
}

//
void Copter::ModeABZz::exit_ab_mode()
{
    //if we don't end mission we must run record_workinfo and break point
    if(!_flags.save_when_exit){
        reset_mission();
    }else{
        //record break point
        record_breakpoint();
        //update ab info
        if(!update_ab_status_to_current(UPDATE_AB_REASON_BREAK)){
            gcs().send_text(MAV_SEVERITY_ERROR, "[05305]ABZZ: update ab point failed when exit.");
        }
        //fill the work information to beacon message
        record_workinfo_for_beacon_msg();
        send_beacon_msg();
    }
    set_sprayer_manual();
}

bool Copter::ModeABZz::brake_and_exit(bool save_sta)
{
    //tell exit processor if we need to save work status
    _flags.save_when_exit = save_sta;

    if (copter.set_mode(BRAKE, MODE_REASON_UNKNOWN)) {
        copter.mode_brake.timeout_to_loiter_ms(500);
        return true;
    }else return false;
}

bool Copter::ModeABZz::check_ab_point_validity()
{
    float dist_limit_m;
#if AC_FENCE == ENABLED
    dist_limit_m = copter.fence.get_radius();
#else
    //we limit the distance in 2000m as default
    dist_limit_m = 2000.0f;
#endif
    Location_Class home(ahrs.get_home());
    if(_point_a.get_distance(home) > dist_limit_m || _point_b.get_distance(home) > dist_limit_m){
        gcs().send_text(MAV_SEVERITY_ERROR, "[05312]ABZZ: A / B point location is exceed distance limit");
        return false;
    }
    return true;
}

bool Copter::ModeABZz::check_break_point_validity()
{
    float dist_limit_m;
#if AC_FENCE == ENABLED
    dist_limit_m = copter.fence.get_radius();
#else
    //we limit the distance in 2000m as default
    dist_limit_m = 2000.0f;
#endif

    if(!_point_break.is_zero()){
        Location_Class home(ahrs.get_home());
        float dist_brk2a = _point_break.get_distance(_point_a);
        float dist_brk2b = _point_break.get_distance(_point_b);
        float dist_a2b    = _point_a.get_distance(_point_b);

        //we give 10m and 20m as margin to judge if break point is unreasonable
        if(dist_brk2a + dist_brk2b >  dist_a2b+10.0f && dist_brk2a > 20.0f && dist_brk2b > 20.0f){
            gcs().send_text(MAV_SEVERITY_ERROR, "[05313]ABZZ: break point location is unreasonable");
            return false;
        }else if(_point_break.get_distance(home) > dist_limit_m){
            gcs().send_text(MAV_SEVERITY_ERROR, "[05314]ABZZ: break point location is exceed distance limit");
            return false;
        }
        return true;
    }
    else{
        return false;
    }
}

bool Copter::ModeABZz::clear_ab_point()
{
    //we clear ab point by reset mission
    reset_mission();
    return true;
}

bool Copter::ModeABZz::set_ab_point(Location_Class &loc_to_a, Location_Class &loc_to_b, uint8_t reason, uint8_t update_mask)
{
    //someone is using ab data
    if(_mutex_ab_param) return false;
    //indicate we are using ab point
    _mutex_ab_param = true;

    if(update_mask&0x01){
        _point_a = loc_to_a;
        _sta_absetting = SAMPLE_B;
    }

    if(update_mask&0x02 && _sta_absetting == SAMPLE_B){
        _point_b = loc_to_b;
        _sta_absetting = SEL_SHIFT_DIR;
    }

    if(reason == UPDATE_AB_REASON_CHGWD || reason == UPDATE_AB_REASON_BREAK){
        _sta_absetting = AB_POINT_CMPLT;

    //for recover purpose we also need to get ab point bearing reverse flag
    }else if(reason == UPDATE_AB_REASON_GCSSET){
        _sta_absetting = GET_AB_BEAR_REV;
    }

    //reset shift count
    _shift_count = 0;

    gcs().send_text(MAV_SEVERITY_DEBUG, "new A:=%d, B=%d, cnt=%d", _point_a.lng,_point_b.lng, _shift_count);
    //release mutex
    _mutex_ab_param = false;

    return true;
}

//update ab status to current work status, including ab point, ab bearing reverse flag and shift count.
bool Copter::ModeABZz::update_ab_status_to_current(ABUpdate_Reason_eu reason)
{
    if(_mutex_ab_param) return false;

    if(_shift_count%2!=0){
         //if we reverse the AB shift point to BA, use flag to mark this exhange.
         //_flag.ab_bearing_reverse is used to track the original sample AB point.
        _flags.ab_bearing_reverse  = !_flags.ab_bearing_reverse;
    }

    //means we are working in AB mode or be paused from the mode
    if(_sta_absetting == AB_POINT_CMPLT && _shift_count!=0){
        Location_Class curr_a(_point_shift_a);
        Location_Class curr_b(_point_shift_b);
        if(!set_ab_point(curr_a, curr_b, reason, 3)) return false;

    }
    return true;
}

//design only run in the current flight mode is other flight mode
bool Copter::ModeABZz::sample_ab_point(uint8_t get_b, Location_Class& loc)
{
    bool result = false;
    //save AB point in above ekf origin altitude frame
    if(_sta_absetting == SAMPLE_B || _sta_absetting == SEL_SHIFT_DIR){
        if(get_b){
            _point_b = loc;
            _sta_absetting = SEL_SHIFT_DIR;
            gcs().send_text(MAV_SEVERITY_INFO, "[05602]ABZZ: point B stored");
            result = true;
        }else{
            gcs().send_text(MAV_SEVERITY_ERROR, "[05303]ABZZ: need point B");
        }
    }else if(_sta_absetting == SAMPLE_A || _sta_absetting == SEL_SHIFT_DIR){
        if(!get_b){
            _point_a = loc;
            _sta_absetting = SAMPLE_B;
            gcs().send_text(MAV_SEVERITY_INFO, "[05603]ABZZ: point A stored");
            result = true;
        }else{
            gcs().send_text(MAV_SEVERITY_ERROR, "[05308]ABZZ: need point A");
        }
    }else {
        gcs().send_text(MAV_SEVERITY_ERROR, "[05309]ABZZ: clear AB point first");
    }
    return result;
}

bool Copter::ModeABZz::save_ab_shiftdir(int16_t direction)
{
    //choose direction by roll stick moving direction
    if( _sta_absetting == SEL_SHIFT_DIR){
        _shift_direction_cw = direction;
        _sta_absetting = AB_POINT_CMPLT;
        if(_shift_direction_cw>0){
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: shift right");
        }else{
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: shift left");
        }
        return true;
    }
    return false;
}

void Copter::ModeABZz::save_ab_shiftdir_RC()
{
    //choose direction by roll stick moving direction
    if( _sta_absetting == SEL_SHIFT_DIR){
        if(channel_roll->get_control_in() > 300){
            _shift_direction_cw = 1;
            _sta_absetting = AB_POINT_CMPLT;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: shift right");
        }else if(channel_roll->get_control_in() < -300){
            _shift_direction_cw = -1;
            _sta_absetting = AB_POINT_CMPLT;
            gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: shift left");
        }
    }
}

void Copter::ModeABZz::set_ab_bearing_reverse_flag(uint8_t flag)
{
    if(flag) 
        _flags.ab_bearing_reverse = true;
    else _flags.ab_bearing_reverse = false;
}

bool Copter::ModeABZz::get_ab_bearing_reverse_flag()
{
    return _flags.ab_bearing_reverse;
}

void Copter::ModeABZz::calc_ab_bearing()
{
    if(_flags.ab_bearing_reverse){
        _ab_bearing_deg = get_bearing_cd(_point_b, _point_a)/100.0f;
    }else{
        _ab_bearing_deg = get_bearing_cd(_point_a, _point_b)/100.0f;
    }
    _flags.ab_bearing_set = true;
    gcs().send_text(MAV_SEVERITY_INFO, "set AB bearing:%f", _ab_bearing_deg);
}

//
bool Copter::ModeABZz::change_shiftwidth(uint16_t new_width_cm)
{
    if(update_ab_status_to_current(UPDATE_AB_REASON_CHGWD)){
        //update shift width parameter
        _shift_width_cm = new_width_cm;
         gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: set shift width: %d cm", _shift_width_cm);
        return true;
    }else{
        gcs().send_text(MAV_SEVERITY_ERROR, "[05315]ABZZ:change width failed");
        return false;
    }
}

void Copter::ModeABZz::set_ab_desired_terrain_alt(uint16_t desired_alt_cm)
{
    if(150>desired_alt_cm) desired_alt_cm = 150;
    else if(600<desired_alt_cm) desired_alt_cm = 600;

    copter.wp_nav->set_wp_desired_terrain_alt(desired_alt_cm);
    gcs().send_text(MAV_SEVERITY_INFO, "ABZZ: set terrain alt: %d cm", desired_alt_cm);
}

bool Copter::ModeABZz::generate_next_abline()
{
    //if someone is using ab point data return right now
    if(_mutex_ab_param) return false;
    _mutex_ab_param = true;

    bool result=false;

    _shift_count++;
    if(generate_abline(_shift_count)) result = true;
    else _shift_count--;
    gcs().send_text(MAV_SEVERITY_DEBUG, "_shift_count = %d", _shift_count);

    _mutex_ab_param = false;

    return result;
}


//this function only use in main state machine. cannot be used in any handler progress
bool Copter::ModeABZz::generate_abline(uint16_t shift_cnt)
{
    // TODO:check if ab points are valid.
    if(_point_a.is_zero() || _point_b.is_zero()) return false;

    float shift_bearing_deg;
    if(_shift_direction_cw>=0){
        shift_bearing_deg = wrap_360(_ab_bearing_deg+90.0f);
    }else{
        shift_bearing_deg = wrap_360(_ab_bearing_deg-90.0f);
    }

    Vector3f vet_a;
    Vector3f vet_b;
    if(!_point_a.get_vector_from_origin_NEU(vet_a)){
        gcs().send_text(MAV_SEVERITY_ERROR, "[05310]ABZZ: point a position error.");
        return false;
    }
    if(!_point_b.get_vector_from_origin_NEU(vet_b)){
        gcs().send_text(MAV_SEVERITY_ERROR, "[05311]ABZZ: point b position error.");
        return false;
    }

    if(shift_cnt%2==0){
        _point_shift_a = vet_a;
        _point_shift_b = vet_b;

    }else{
        _point_shift_a = vet_b;
        _point_shift_b = vet_a;
    }

    if(shift_cnt==0)
        return true;

    //update shift point by shift vector
    float ofs_x = cosf(radians(shift_bearing_deg))*_shift_width_cm*shift_cnt;
    float ofs_y  = sinf(radians(shift_bearing_deg))*_shift_width_cm*shift_cnt;

    gcs().send_text(MAV_SEVERITY_DEBUG, "ofs_x=%f,ofs_y=%f", ofs_x, ofs_y);
    _point_shift_a += Vector3f(ofs_x,ofs_y,0);
    _point_shift_b += Vector3f(ofs_x,ofs_y,0);

//    location_update(_point_shift_a, );
//    location_update(_point_shift_b, );

    return true;
}

bool Copter::ModeABZz::set_work_speed(const uint16_t speed)
{
    if(speed >=WPNAV_WP_SPEED_MIN && speed <= 1000){
        _pilot_desired_speed_cms = speed;
        return true;
    }else return false;
}

void Copter::ModeABZz::update_work_speed()
{
    bool val_change=false;
    //slow down speed limit change when speed limit decrease
    int16_t speed_limit_err = _pilot_desired_speed_cms - _maxspeed_cms;

    if(speed_limit_err < -5){
        _maxspeed_cms -= 5;
        val_change = true;
    }else if(speed_limit_err!=0){//if(!is_equal(speed_limit_err, 0.0f)){
        _maxspeed_cms += speed_limit_err;
        val_change = true;
    }

    if(val_change){
        copter.wp_nav->set_speed_xy(_maxspeed_cms);
    }
}

//reset necessary mission data
void Copter::ModeABZz::start_mission()
{
    _point_break.zero();
    _point_shift_b.zero();
    _point_shift_a.zero();
    _shift_count = 0;
    _flags.ab_bearing_set = false;
    _flags.ab_bearing_reverse = false;
}

//reset necessary mission data
void Copter::ModeABZz::reset_mission()
{
    _point_break.zero();
    _point_a.zero();
    _point_b.zero();
    _point_shift_b.zero();
    _point_shift_a.zero();
    _sta_absetting = SAMPLE_A;
    _sta_abzz = Start;
    _sta_abzz_last = Start;
    _shift_direction_cw = 1;
    _shift_count = 0;
    _flags.ab_bearing_set = false;
    _flags.ab_bearing_reverse = false;
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
bool Copter::ModeABZz::wp_start(Vector3f& dest_vect)
{

    copter.wp_nav->set_speed_xy(_pilot_desired_speed_cms);

    // send target to waypoint controller
    if (!set_next_wp(dest_vect)) {
        return false;
    }

    _mode = Abzz_WP;
    return true;

}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
bool Copter::ModeABZz::wp_start(const Location_Class& dest_loc)
{

    copter.wp_nav->set_speed_xy(_pilot_desired_speed_cms);

    //convert to vector with x,y from ekf_origin and alt above ground if terrain is enable,
    //if not, the vector is from ekf_origin
    Vector2f vect2d;
    dest_loc.get_vector_xy_from_origin_NE(vect2d);

    Vector3f dest_vct(vect2d.x, vect2d.y, 0);
    if (!set_next_wp(dest_vct)) {
        return false;
    }

    _mode = Abzz_WP;
    return true;

}

bool Copter::ModeABZz::set_next_wp(Vector3f& dest_vect)
{

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination_xy(dest_vect)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return false;
    }
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


// loiter_run - loiter in BAZZ flight mode
//      called by auto_run at 100hz or more
void Copter::ModeABZz::loiter_run()
{

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    loiter_nav->clear_pilot_desired_acceleration();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run loiter controller
    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

    // adjusts target up or down using a climb rate
    pos_control->update_z_controller();
}


//uint16_t _testcnt=0;
// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::ModeABZz::wp_run()
{
    float target_climb_rate = 0.0f;
//    bool 
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

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);


    // run waypoint controller
//    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // adjust climb rate using rangefinder
//    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

    // update altitude target and call position controller
    if(!is_zero(target_climb_rate)){
        copter.failsafe_terrain_set_status(wp_nav->update_wpnav_agr(true));
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    }else{
        copter.failsafe_terrain_set_status(wp_nav->update_wpnav_agr(false));
    }
    
//    if(_testcnt++>200){
//        _testcnt = 0;
//        gcs().send_text(MAV_SEVERITY_INFO, " dest_vect.z=%f, _pos_target.z=%f",_testdat, _testdat2);
//    }

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
        return true;
    } else {
        return false;
    }
}

//operate_sprayer according to force disable spraying flag
void Copter::ModeABZz::operate_sprayer(const bool enable)
{
    copter.sprayer.handle_cmd_auto(enable);
}


void Copter::ModeABZz::set_sprayer_auto()
{
    copter.sprayer.set_pump_mode(AC_Sprayer::Auto);
}

void Copter::ModeABZz::set_sprayer_manual()
{
    copter.sprayer.set_pump_mode(AC_Sprayer::Manual);
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

void Copter::ModeABZz::handle_set_pecial_point_Info(uint8_t info_type, BeaconParams& param)
{
    if(copter.flightmode == &copter.mode_loiter){
        switch(info_type){
            case 4:{
                Location_Class loc_break(param.aPointLatitude, param.aPointLongitude, param.height, Location_Class::ALT_FRAME_ABOVE_TERRAIN);
                set_breakpoint(loc_break);
                set_ab_bearing_reverse_flag(param.breakDirection);
                gcs().send_text(MAV_SEVERITY_DEBUG, "set break point.");
            }break;

            case 2:{
                Location_Class loc_a(param.aPointLatitude, param.aPointLongitude, param.height, Location_Class::ALT_FRAME_ABOVE_TERRAIN);
                Location_Class loc_b;
                set_ab_point(loc_a, loc_b, UPDATE_AB_REASON_GCSSET, 0x01);
                gcs().send_text(MAV_SEVERITY_DEBUG, "set a point.");
            }break;

            case 3:{
                Location_Class loc_b(param.aPointLatitude, param.aPointLongitude, param.height, Location_Class::ALT_FRAME_ABOVE_TERRAIN);
                Location_Class loc_a;
                set_ab_point(loc_a, loc_b, UPDATE_AB_REASON_GCSSET, 0x02);
                gcs().send_text(MAV_SEVERITY_DEBUG, "set b point.");
            }break;

            default:break;
        }
    }
}

#endif
