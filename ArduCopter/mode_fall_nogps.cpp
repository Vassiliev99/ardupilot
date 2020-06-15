#include "Copter.h"


bool ModeFallNoGPS::init(bool ignore_checks)
{
    //attitude_control->relax_attitude_controllers();
    //attitude_control->set_attitude_target_to_current_attitude();
    //attitude_control->set_yaw_target_to_current_heading();
    //attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, 0.0f, false);
    //motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    return true;
}

/*
 * Init and run calls for stabilize flight mode
 */

// should be called at 100hz or more
void ModeFallNoGPS::run()
{
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0);
    //attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, 0.0f, false);

    //attitude_control->set_throttle_out(0.4f, true, g.throttle_filt);

    //attitude_control->reset_rate_controller_I_terms();

    
    //pos_control->get_alt

    int parachute_pwm = 1000;
    if (motors->armed() && pos_control->get_current_alt() < 10000) {
        parachute_pwm = 2000;
    }
    SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, parachute_pwm);

    static int counter = 0;
    counter++;
    if (counter > 50) {
        counter = 0;
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "%5.5f %5.5f %5.5f %5.5f", throttle_thrust_best_rpy, thr_adj, rpy_scale, _thrust_rpyt_out[0]);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "%5.5f", pos_control->get_current_alt());
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "%5.3f %5.3f %5.3f %5.3f", _thrust_rpyt_out[0], _thrust_rpyt_out[1], _thrust_rpyt_out[2], _thrust_rpyt_out[3]);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "t %5.3f %5.3f %5.3f", target_roll, target_pitch, target_yaw_rate);
        AP::logger().Write("PRCT", "TimeUS,Pwm,Alt", "Qif",
                                        AP_HAL::micros64(),
                                        parachute_pwm,
                                        pos_control->get_current_alt());
    }


    
}
