#include "Copter.h"

bool has_mode_set;
Location Current_position;
Location center_position;
float radius;
bool can_run_the_mission;

#define FIRST_INTERVAL_WAIT 3.0f
#define SECOND_INTERVAL_WAIT 3.0f

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    gcs().send_text(MAV_SEVERITY_INFO, "User code has started");

    radius = 5.0f;
    can_run_the_mission = false;
    has_mode_set = false;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    
    if(can_run_the_mission){
        //gcs().send_text(MAV_SEVERITY_INFO, "We are in auto %f", radius);
        //pos_control->set_alt_target(20000);
        wp_nav->update_wpnav();
    }
    
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    //if(copter.control_mode == Mode::Number::AUTO){
    //    gcs().send_text(MAV_SEVERITY_INFO, "We are in auto mode");
    //    has_mode_set = true;
    //}

    

    if(copter.current_loc.relative_alt == 1 && copter.current_loc.alt > 500 && !can_run_the_mission){
        can_run_the_mission = true;
        gcs().send_text(MAV_SEVERITY_INFO, "We can run the mission");
        gcs().send_text(MAV_SEVERITY_INFO, "Relative alt %d",copter.current_loc.alt);
        wp_nav->wp_and_spline_init();
        wp_nav->set_wp_destination_NED(Vector3f(5.0, 5.0, -5.0));
    }

    if(can_run_the_mission && !has_mode_set){
        //copter.set_mode(Mode::Number::AUTO, ModeReason::SCRIPTING);
        //copter.mode_auto.circle_movetoedge_start(center_position, radius);
        has_mode_set = true;
    }

}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
