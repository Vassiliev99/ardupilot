#include "Copter.h"

//#define _USE_MATH_DEFINES
//#define ALLOW_DOUBLE_MATH_FUNCTIONS
//#include <cmath>


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeRTLNoGPS::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    Location home_loc, curr_loc;
    home_loc = AP::ahrs().get_home();
    AP::ahrs().get_position(curr_loc); // change to last valid position

    float home_lat = home_loc.lat / 10000000.0 * M_PI / 180.0;
    float home_lng = home_loc.lng / 10000000.0 * M_PI / 180.0;
    float curr_lat = curr_loc.lat / 10000000.0 * M_PI / 180.0;
    float curr_lng = curr_loc.lng / 10000000.0 * M_PI / 180.0;
    _azimuth = atan2f(sinf(home_lng - curr_lng) * cosf(home_lat), cosf(curr_lat) * sinf(home_lat) - sinf(curr_lat) * cosf(home_lat) * cosf(home_lng - curr_lng)) / M_PI * 180.0;
    if (_azimuth < 0) {
        _azimuth += 360.0;
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeRTLNoGPS::run()
{

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);



    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0.0f, target_climb_rate = 0.0f;
    //get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    //float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    //target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "%5.5f %5.5f %5.5f %5.5f", target_roll, target_pitch, target_yaw_rate, target_climb_rate);

    float curr_yaw = AP::ahrs().get_yaw() / M_PI * 180.0;
    if (curr_yaw < 0) {
        curr_yaw += 360.0;
    }

    if (abs(_azimuth - curr_yaw) < 1.0f || abs(_azimuth - curr_yaw - 360.0f) < 1.0f) {
        target_pitch = 1000.0f;
    }
    else {
        if (_azimuth > curr_yaw) {
            target_yaw_rate = 3000.0f;
        }
        else {

        }
    }
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "%8.8f %8.8f", _azimuth, curr_yaw);



    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

    // adjust climb rate using rangefinder
    target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();


    /*static int counter = 0;
    counter++;
    if (counter > 50) {
        counter = 0;
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "%ld %ld %ld", AP::ahrs().get_home().lat, AP::ahrs().get_home().lng, AP::ahrs().get_home().alt);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "(%ld %ld) (%ld %ld)", home_loc.lat, home_loc.lng, curr_loc.lat, curr_loc.lng);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%8.8f %8.8f", _azimuth, (AP::ahrs().get_yaw() / M_PI + 1.0) * 180.0);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "(%7.7f %7.7f) (%7.7f %7.7f)", home_lat, home_lng, curr_lat, curr_lng);
    }*/
}
