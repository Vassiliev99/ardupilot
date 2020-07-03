#include "Copter.h"


bool ModeRTLNoGPS::init(bool ignore_checks)
{
    // --------------- FROM ALTHOLD ---------------
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    // --------------- FROM ALTHOLD ---------------



    if (!ahrs.home_is_set()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Home is not set");
        return false;
    }

    if (!copter.gps_last_good_update_ms) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No valid location");
        return false;
    }

    Location home_loc = ahrs.get_home();
    const float int_loc_to_rad = M_PI / 10000000.0f / 180.0f;
    float home_lat = home_loc.lat * int_loc_to_rad;
    float home_lng = home_loc.lng * int_loc_to_rad;
    float curr_lat = copter.gps_last_good_loc.lat * int_loc_to_rad;
    float curr_lng = copter.gps_last_good_loc.lng * int_loc_to_rad;
    _azimuth = wrap_360(atan2f(sinf(home_lng - curr_lng) * cosf(home_lat), cosf(curr_lat) * sinf(home_lat) - sinf(curr_lat) * cosf(home_lat) * cosf(home_lng - curr_lng)) / M_PI * 180.0f);
    _yaw_ready_ms = 0;

    //auto_yaw.set_fixed_yaw(_azimuth, 0.0f, 0, false);

    return true;
}


// should be called at 100hz or more
void ModeRTLNoGPS::run()
{
    // -------------- FROM ALTHOLD START --------------
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);
    // --------------- FROM ALTHOLD END ---------------


    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw = _azimuth * 100.0f;
    float target_climb_rate = 0.0f;

    float curr_yaw = wrap_360(ahrs.get_yaw() / M_PI * 180.0f);

    // check if copter turned to correct yaw (azimuth to home) and ready to fly by pitch 
    float yaw_error = abs(_azimuth - curr_yaw);
    if (yaw_error > 180.0f) {
        yaw_error = 360.0f - yaw_error;
    } 
    if (!_yaw_ready_ms && yaw_error < g.rtl_nogps_curr_yaw_delta) {
        _yaw_ready_ms = millis();
    }

    if (_yaw_ready_ms && millis() - _yaw_ready_ms > 5000) {
        target_pitch = g.rtl_nogps_pitch;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    static int counter = 0;
    counter++;
    if (counter > 100) {
        counter = 0;
        //gcs().send_text(MAV_SEVERITY_WARNING, "az cur y p %5.5f %5.5f %5.5f %5.5f", _azimuth, curr_yaw, target_yaw_rate, target_pitch);
        //gcs().send_text(MAV_SEVERITY_INFO, "py %5.5f %5.5f", target_pitch, target_yaw_rate);
        //gcs().send_text(MAV_SEVERITY_INFO, "%5.5f %5.5f %5.5f", _azimuth, curr_yaw, target_climb_rate);
        //gcs().send_text(MAV_SEVERITY_INFO, "%d %d", millis(), _yaw_ready_ms);
    }

    


    // -------------- FROM ALTHOLD START --------------

    //TODO check??
#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        //copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif


    // adjust climb rate using rangefinder
    //target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

    // get avoidance adjusted climb rate
    //target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // --------------- FROM ALTHOLD END ---------------

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

    // -------------- FROM ALTHOLD START --------------
    // call z-axis position controller
    pos_control->update_z_controller();

    // --------------- FROM ALTHOLD END ---------------

}
