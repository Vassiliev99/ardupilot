#include "mode.h"
#include "Plane.h"

bool ModeLandParachute::_enter()
{

    curr_stage = 0;
    stage_started_ms = millis();

    return true;
}

void ModeLandParachute::update()
{
    static int counter = 0;
    counter++;
    if (counter == 50) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_INFO, "Current stage: %d", curr_stage);
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, 1000); // disable motor

    switch (curr_stage) {
    case 0: // wait for motor to stop
        if (millis() - stage_started_ms > 1000) {
            curr_stage = 1;
            stage_started_ms = millis();
        }
        break;
    case 1: // cobra braking
        SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_left, 1700);
        SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_right, 1350);
        if (millis() - stage_started_ms > 500) {
            curr_stage = 2;
            stage_started_ms = millis();
        }
        break;
    case 2: // release parachute
        {
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, 1880);

            Vector3f vel;
            if (AP::ahrs().get_velocity_NED(vel)) {
                if (counter == 49) {
                    gcs().send_text(MAV_SEVERITY_INFO, "AHRS velocity: %5.5f", vel.z);
                    gcs().send_text(MAV_SEVERITY_INFO, "Baro climb rate: %5.5f", AP::baro().get_climb_rate());
                }
            }
            if (vel.z < 0 && millis() - stage_started_ms > 5000) { // TODO add 0 to params
                curr_stage = 3;
                stage_started_ms = millis();
            }
            break;
        }
    //TODO test if extra time on the ground needed
    case 3: // unfasten parachute
        SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, 1900);
        if (millis() - stage_started_ms > 1000) {
            curr_stage = 4;
            stage_started_ms = millis();
        }
        break;
    case 4: // end
        break;
    }
}