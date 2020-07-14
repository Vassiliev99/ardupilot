#include "Copter.h"

/**
 *
 * Detects failures of the ekf or inertial nav system triggers an alert
 * to the pilot and helps take countermeasures
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check structure
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    uint8_t bad_variance : 1;   // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void Copter::ekf_check()
{
    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if motors are not armed, or ekf check is disabled
    if (!motors->armed() || (g.fs_ekf_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }


    check_gps_position();
    check_gps_failsafe();
    accum_wind();
    calc_wind();



    // compare compass and velocity variance vs threshold
    if (ekf_over_threshold()) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
#if EKF_CHECK_ITERATIONS_MAX > 2
            if (ekf_check_state.fail_count == EKF_CHECK_ITERATIONS_MAX-1) {
                // we are just about to declare a EKF failsafe, ask the EKF if we can change lanes
                // to resolve the issue
                ahrs.check_lane_switch();
            }
#endif
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

// ekf_over_threshold - returns true if the ekf's variance are over the tolerance
bool Copter::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // use EKF to get variance
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance, offset);

    // return true if two of compass, velocity and position variances are over the threshold OR velocity variance is twice the threshold
    uint8_t over_thresh_count = 0;
    if (mag_variance.length() >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    if (!optflow.healthy() && (vel_variance >= (2.0f * g.fs_ekf_thresh))) {
        over_thresh_count += 2;
    } else if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    if (position_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if (over_thresh_count >= 2) {
        return true;
    }

    // either optflow relative or absolute position estimate OK
    if (optflow_position_ok() || ekf_position_ok()) {
        return false;
    }
    return true;
}


// failsafe_ekf_event - perform ekf failsafe
void Copter::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // sometimes LAND *does* require GPS so ensure we are in non-GPS land
    if (control_mode == Mode::Number::LAND && landing_with_GPS()) {
        mode_land.do_not_use_GPS();
        return;
    }

    // does this mode require position?
    if (!copter.flightmode->requires_GPS() && (g.fs_ekf_action != FS_EKF_ACTION_LAND_EVEN_STABILIZE)) {
        return;
    }

    // take action based on fs_ekf_action parameter
    switch (g.fs_ekf_action) {
        case FS_EKF_ACTION_RTL_NOGPS:
            {
                Location home_loc = ahrs.get_home();
                if (copter.gps_last_good_loc.get_distance(home_loc) < 500) {
                    if (failsafe.radio || !set_mode(Mode::Number::ALT_HOLD, ModeReason::EKF_FAILSAFE)) {
                        set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
                    }
                }
                else {
                    set_mode(Mode::Number::RTL_NOGPS, ModeReason::EKF_FAILSAFE);
                }
                break;
            }
        case FS_EKF_ACTION_ALTHOLD:
            // AltHold
            if (failsafe.radio || !set_mode(Mode::Number::ALT_HOLD, ModeReason::EKF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            }
            break;
        case FS_EKF_ACTION_LAND:
        case FS_EKF_ACTION_LAND_EVEN_STABILIZE:
        default:
            set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            break;
    }
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
void Copter::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    failsafe.ekf = false;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}

// check for ekf yaw reset and adjust target heading, also log position reset
void Copter::check_ekf_reset()
{
    // check for yaw reset
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        Log_Write_Event(DATA_EKF_YAW_RESET);
    }

#if AP_AHRS_NAVEKF_AVAILABLE
    // check for change in primary EKF (log only, AC_WPNav handles position target adjustment)
    if ((EKF2.getPrimaryCoreIndex() != ekf_primary_core) && (EKF2.getPrimaryCoreIndex() != -1)) {
        attitude_control->inertial_frame_reset();
        ekf_primary_core = EKF2.getPrimaryCoreIndex();
        AP::logger().Write_Error(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(ekf_primary_core));
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF primary changed:%d", (unsigned)ekf_primary_core);
    }
#endif
}

// check for high vibrations affecting altitude control
void Copter::check_vibration()
{
    uint32_t now = AP_HAL::millis();

    // assume checks will succeed
    bool checks_succeeded = true;

    // check if vertical velocity and position innovations are positive (NKF3.IVD & NKF3.IPD are both positive)
    Vector3f vel_innovation;
    Vector3f pos_innovation;
    Vector3f mag_innovation;
    float tas_innovation;
    float yaw_innovation;
    if (!ahrs.get_innovations(vel_innovation, pos_innovation, mag_innovation, tas_innovation, yaw_innovation)) {
        checks_succeeded = false;
    }
    const bool innov_velD_posD_positive = is_positive(vel_innovation.z) && is_positive(pos_innovation.z);

    // check if EKF's NKF4.SH and NK4.SV > 1.0
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    if (!ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance, offset)) {
        checks_succeeded = false;
    }

    // if no failure
    if ((g2.fs_vibe_enabled == 0) || !checks_succeeded || !motors->armed() || !innov_velD_posD_positive || (vel_variance < 1.0f)) {
        if (vibration_check.high_vibes) {
            // start clear time
            if (vibration_check.clear_ms == 0) {
                vibration_check.clear_ms = now;
                return;
            }
            // turn off vibration compensation after 15 seconds
            if (now - vibration_check.clear_ms > 15000) {
                // restore ekf gains, reset timers and update user
                vibration_check.high_vibes = false;
                pos_control->set_vibe_comp(false);
                vibration_check.clear_ms = 0;
                AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_RESOLVED);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation OFF");
            }
        }
        vibration_check.start_ms = 0;
        return;
    }

    // start timer
    if (vibration_check.start_ms == 0) {
        vibration_check.start_ms = now;
        vibration_check.clear_ms = 0;
        return;
    }

    // check if failure has persisted for at least 1 second
    if (now - vibration_check.start_ms > 1000) {
        if (!vibration_check.high_vibes) {
            // switch ekf to use resistant gains
            vibration_check.high_vibes = true;
            pos_control->set_vibe_comp(true);
            AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_OCCURRED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation ON");
        }
    }
}



// ------------- OLD GPS FAILSAFE ---------------

void Copter::check_gps_position() {
    uint32_t now = millis();        // current system time
    float sane_dt;                  // time since last sane gps reading
    float accel_based_distance;     // movement based on max acceleration
    Location curr_pos;              // our current position estimate
    Location gps_pos;               // gps reported position
    float distance_cm;              // distance from gps to current position estimate in cm
    bool all_ok;                    // true if the new gps position passes sanity checks

    // exit immediately if we don't have gps lock
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        copter.gps_glitch = true;
        return;
    }

    // if not initialised or disabled update last good position and exit
    if (!copter.check_gps_initialised) {
        copter.gps_last_good_loc = gps.location();
        copter.gps_last_good_vel = gps.velocity();
        copter.gps_last_good_update_ms = now;
        copter.check_gps_initialised = true;
        copter.gps_glitch = false;
        return;
    }

    // calculate time since last sane gps reading in ms
    sane_dt = (now - copter.gps_last_good_update_ms) / 1000.0f;

    // project forward our position from last known velocity
    curr_pos = copter.gps_last_good_loc;
    curr_pos.offset(copter.gps_last_good_vel.x * sane_dt, copter.gps_last_good_vel.y * sane_dt);

    // calculate distance from recent gps position to current estimate
    gps_pos = gps.location();
    distance_cm = curr_pos.get_distance(gps_pos) * 100.0f;

    // all ok if within a given hardcoded radius
    if (distance_cm <= 200.0f) { //TODO add to params
        all_ok = true;
    }else{
        // or if within the maximum distance we could have moved based on our acceleration
        accel_based_distance = 0.5f * 1000.0f * sane_dt * sane_dt; //TODO add to params _accel_max_cmss = 1000.0f
        all_ok = (distance_cm <= accel_based_distance);
    }

    // store updates to gps position
    if (all_ok) {
        // position is acceptable
        copter.gps_last_good_update_ms = now;
        copter.gps_last_good_loc = gps_pos;
        copter.gps_last_good_vel = gps.velocity();
    }

    // update glitching flag
    copter.gps_glitch = !all_ok;
}

// failsafe_gps_check - check for gps failsafe
void Copter::check_gps_failsafe()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled or we have never had GPS lock
    if (!AP::ahrs().home_is_set()) { //TODO add param to disable gps failsafe
        // if we have just disabled the gps failsafe, ensure the gps failsafe event is cleared
        if (failsafe.ekf) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Failsafe resolved");
            failsafe_ekf_off_event();
        }
        return;
    }

    // calc time since last gps update
    last_gps_update_ms = millis() - copter.gps_last_good_update_ms;

    // check if all is well
    if (last_gps_update_ms < 5000) { //TODO add define FAILSAFE_GPS_TIMEOUT_MS 5000
        // check for recovery from gps failsafe
        if (failsafe.ekf) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Failsafe resolved");
            failsafe_ekf_off_event();
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if (failsafe.ekf || !motors->armed()) {
        return;
    }

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS Failsafe");
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_GPS, LogErrorCode::FAILSAFE_OCCURRED);

    failsafe_ekf_event();

    /*// take action based on flight mode and FS_GPS_ENABLED parameter
    if (mode_requires_GPS(control_mode) || g.failsafe_gps_enabled == FS_GPS_LAND_EVEN_STABILIZE) {
        if (g.failsafe_gps_enabled == FS_GPS_ALTHOLD && !failsafe.radio) {
            set_mode(ALT_HOLD);
        }else{
            set_mode_land_with_pause();
        }
    }
    // if flight mode is LAND ensure it's not the GPS controlled LAND
    if (control_mode == LAND) {
        land_do_not_use_GPS();
    }*/
}


// -------------------------------------------


// ------------------WIND---------------------

void Copter::accum_wind() {
    // initialise wind variables
    if (!copter.accum_wind_initialised) {
        copter.wind_data_last_item = 0;
        copter.wind_data_total_items = 0;
        copter.accum_wind_initialised = true;

        return;
    } //TODO remove

    if (copter.wind_data_last_item != 0 && millis() - copter.wind_data[copter.wind_data_last_item].ms < (uint32_t)g.wind_data_save_ms) {
        return;
    }

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Can not get velocity for wind calculation");
        return;
    }

    // check if flight is stable without accelerations
    Vector3f accel = ahrs.get_accel_ef();
    if (abs(accel.x) > 0.15f || abs(accel.y) > 0.15f) { // TODO is it required?
        //gcs().send_text(MAV_SEVERITY_INFO, "accel %3.3f %3.3f %3.3f", accel.x, accel.y, accel.z);
        return;
    }

    // copter velocity (east north)
    float vel_e = vel.y;
    float vel_n = vel.x;
    // combined horizontal velocity (both east and north)
    float vel_ne = sqrtf(vel_e * vel_e + vel_n * vel_n);
    // velocity angle (0 - 360)
    float vel_ang = degrees(atanf(abs(vel_e) / abs(vel_n)));
    if (vel_e >= 0) {
        if (vel_n <= 0) { vel_ang = 180.0f - vel_ang; }
    }
    else {
        if (vel_n <= 0) { vel_ang = 180.0f + vel_ang; }
        else { vel_ang = 360.0f - vel_ang; }
    }

    float roll = degrees(ahrs.get_roll()); // -180 - 180
    float pitch = degrees(ahrs.get_pitch()); // -180 - 180
    float yaw = degrees(wrap_2PI(ahrs.get_yaw())); // 0 - 360
    // reverse pitch so it will be positive on forward flight 
    pitch = - pitch;

    //velocity angle regarding roll and pitch
    float vel_ang_rp = wrap_360(vel_ang - yaw);
    // calculate velocity by roll and pitch
    float vel_r, vel_p, alpha;
    if (0.0f <= vel_ang_rp && vel_ang_rp < 90.0f) {
        alpha = radians(vel_ang_rp);
        vel_r = vel_ne * sinf(alpha);
        vel_p = vel_ne * cosf(alpha);
    }
    else if (90.0f <= vel_ang_rp && vel_ang_rp < 180.0f) {
        alpha = radians(vel_ang_rp - 90.0f);
        vel_r = vel_ne * cosf(alpha);
        vel_p = - vel_ne * sinf(alpha);
    }
    else if (180.0f <= vel_ang_rp && vel_ang_rp < 270.0f) {
        alpha = radians(vel_ang_rp - 180.0f);
        vel_r = - vel_ne * sinf(alpha);
        vel_p = - vel_ne * cosf(alpha);
    }
    else {
        alpha = radians(vel_ang_rp - 270.0f);
        vel_r = - vel_ne * cosf(alpha);
        vel_p = vel_ne * sinf(alpha);
    }

    if (g.wind_setup) {
        static int counter1 = 0;
        counter1 = (counter1 + 1) % 3;
        if (counter1 == 2) {
            gcs().send_text(MAV_SEVERITY_INFO, "r %2.2f %3.3f; p %2.2f %3.3f; y %2.2f", roll, vel_r, pitch, vel_p, yaw);
        }
    }

    // save values for future wind calculation
    wind_data_t wd = {millis(), roll, pitch, yaw, vel_r, vel_p};
    copter.wind_data_last_item = (copter.wind_data_last_item + 1) % WIND_DATA_COUNT;
    if (copter.wind_data_total_items < WIND_DATA_COUNT) { copter.wind_data_total_items += 1; }
    copter.wind_data[copter.wind_data_last_item] = wd;
}

void Copter::calc_wind() {
    if (!copter.accum_wind_initialised || copter.wind_data_total_items == 0) {
        return;
    }


    // calculate wind vector (east north) from saved values
    float wind_e = 0;
    float wind_n = 0;
    for (int i = 0; i < copter.wind_data_total_items; i++) {
        wind_data_t wd = copter.wind_data[i];
        float roll_wind = wd.roll - wd.vel_r * g.angle_velocity_coef; // TODO check if non linear coefs
        float pitch_wind = wd.pitch - wd.vel_p * g.angle_velocity_coef; 
        float alpha;
        if (0.0f <= wd.yaw && wd.yaw < 90.0f) {
            alpha = radians(wd.yaw);
            wind_e += roll_wind * cosf(alpha) + pitch_wind * sinf(alpha);
            wind_n += - roll_wind * sinf(alpha) + pitch_wind * cosf(alpha);
        }
        else if (90.0f <= wd.yaw && wd.yaw < 180.0f) {
            alpha = radians(wd.yaw - 90.0f);
            wind_e += - roll_wind * sinf(alpha) + pitch_wind * cosf(alpha);
            wind_n += - roll_wind * cosf(alpha) - pitch_wind * sinf(alpha);
        }
        else if (180.0f <= wd.yaw && wd.yaw < 270.0f) {
            alpha = radians(wd.yaw - 180.0f);
            wind_e += - roll_wind * cosf(alpha) - pitch_wind * sinf(alpha);
            wind_n += roll_wind * sinf(alpha) - pitch_wind * cosf(alpha);
        }
        else {
            alpha = radians(wd.yaw - 270.0f);
            wind_e += roll_wind * sinf(alpha) - pitch_wind * cosf(alpha);
            wind_n += roll_wind * cosf(alpha) + pitch_wind * sinf(alpha);
        }
    }

    // calculate wind angle from wind vector
    if (is_zero(wind_n)) {
        if (is_zero(wind_e)) { copter.wind_ang = 0.0f; }
        else if (wind_e > 0.0f) { copter.wind_ang = 90.0f; }
        else { copter.wind_ang = 270.0f; }
    }
    else {
        copter.wind_ang = degrees(atanf(abs(wind_e) / abs(wind_n)));
        if (wind_e >= 0.0f) {
            if (wind_n <= 0.0f) { copter.wind_ang = 180.0f - copter.wind_ang; }
        }
        else {
            if (wind_n <= 0.0f) { copter.wind_ang = 180.0f + copter.wind_ang; }
            else { copter.wind_ang = 360.0f - copter.wind_ang; }
        }
    }

    // calculate wind velocity
    copter.wind_vel = sqrtf(wind_e * wind_e + wind_n * wind_n) / copter.wind_data_total_items / g.wind_velocity_coef; // TODO check if non linear

    if (!g.wind_setup) {
        static int counter2 = 0;
        counter2 = (counter2 + 1) % 5;
        if (counter2 == 4) {
            //gcs().send_text(MAV_SEVERITY_INFO, "wind %3.3f* %3.3fm/s; n %3.3f e %3.3f", wind_ang, wind_vel, wind_n, wind_e);
            gcs().send_text(MAV_SEVERITY_INFO, "wind %0.0f* %2.2fm/s", copter.wind_ang, copter.wind_vel);
        }
    }
}

// -------------------------------------------

