#include "Copter.h"

#define MISSION_POS_SENSITIVITY 30
#define MISSION_ANGLE_SENSITIVITY 0.18f
#define YAW_RATE 6000.0f
#define MISSION_SPEED 70.0f
#define ANGLE_DT -0.10f
#define CENTER_Z 500.0f
#define CENTER_X 500.0f
#define RADIUS_MISSION 300.0f

// #define CLIMB_SPEED 30.0f

bool ModeMission::init(bool ignore_checks){
    if(ignore_checks){
        return true;
    }


    
    wp_nav->wp_and_spline_init();
    wp_nav->set_speed_xy(MISSION_SPEED);
    wp_nav->set_speed_up(MISSION_SPEED);

    // pos_control->init_vel_controller_xyz();
    //pos_control->init_xy_controller();
    
    attitude_control->reset_rate_controller_I_terms();
    //attitude_control->relax_attitude_controllers();

    //auto_yaw.set_rate(0.2);
    auto_yaw.set_mode(autopilot_yaw_mode::AUTO_YAW_RESETTOARMEDYAW);

    mission_leg = 1;
    has_started_leg = false;

    start_yaw_location = copter.current_loc;
    current_location = copter.current_loc;
    leg_has_finished = false;
    yaw_variable = YAW_RATE;

    travel_setpoint = Vector3f(500.0f, 0.0f, 200.0f);
    next_leg = 2;

    current_location.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN);
    has_reached_circle_start = false;
    current_radians = 0.0;

    return true;
}



void ModeMission::run(){
    switch(mission_leg){
        case 1:
            if(!first_segment_run()){
                first_segment_finish();
            }
        break;

        case 2:
            if(!second_segment_run()){
                second_segment_finish();
            }
        break;

        case 3:
            if(!last_segment_run()){
                last_segment_finish();
            }
        break;

        case 4:
            if(!return_run()){
                return_finish();
            }
        break;

        case 5:
            //copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        break;

        default:
        break;
    }
    
    //float dt = pos_control->time_since_last_xy_update();
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    pos_control->update_z_controller();

    if(output_rate){
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), yaw_variable);
    }else{
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }

    //attitude_control->rate_controller_run();
}


bool ModeMission::first_segment_run(){
    if(!has_started_leg){
        first_segment_init();
        has_started_leg = true;
    }

    //gcs().send_text(MAV_SEVERITY_INFO, "Distance to setpoint: %f", wp_nav->get_wp_distance_to_destination());
    //wp_nav->update_wpnav();

    
    if(wp_nav->reached_wp_destination()){
        //gcs().send_text(MAV_SEVERITY_INFO, "We are close to dest:%f",wp_nav->get_wp_distance_to_destination());
        return false;
    }

    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 9000.0f);
    return true;
}


bool ModeMission::second_segment_run(){
    if(!has_started_leg){
        second_segment_init();
        has_started_leg = true;
    }

    
    if(wp_nav->reached_wp_destination()){
        //gcs().send_text(MAV_SEVERITY_INFO, "We are close to dest:%f",wp_nav->get_wp_distance_to_destination());
        return false;
    }

    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 9000.0f);
    return true;
}

bool ModeMission::last_segment_run(){
    if(!has_started_leg){
        last_segment_init();
        has_started_leg = true;
    }

    if(!has_reached_circle_start && wp_nav->reached_wp_destination()){
        has_reached_circle_start = true;
        gcs().send_text(MAV_SEVERITY_INFO, "We are starting to circulate");
        return true;
    }

    if(has_reached_circle_start){
        output_rate = true;
        yaw_variable = YAW_RATE / 2.0f;

        next_circle_point.x = CENTER_X + (RADIUS_MISSION * cosf(wrap_2PI(current_radians + M_PI)));
        next_circle_point.z = CENTER_Z + (RADIUS_MISSION * sinf(wrap_2PI(current_radians + M_PI)));
        next_circle_point.y = 0.0f;

       

        if(wp_nav->reached_wp_destination()){
            gcs().send_text(MAV_SEVERITY_INFO, "Current Pos: %f %f %f", current_radians, next_circle_point.x, next_circle_point.z);
            current_radians += ANGLE_DT;
            wp_nav->set_wp_destination(next_circle_point, true);
        }      
    }

    if(-current_radians > 2 * M_PI){
        return false;
    }

    return true;
}

bool ModeMission::return_run(){
    if(!has_started_leg){
        return_init();
        has_started_leg = true;
    }

    if(wp_nav->reached_wp_destination()){
        //gcs().send_text(MAV_SEVERITY_INFO, "We are close to dest:%f",wp_nav->get_wp_distance_to_destination());
        //wp_nav->get_wp_stopping_point(stopping_point);
        
        //wp_nav->set_wp_destination(stopping_point, true);

        return false;
    }

    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 9000.0f);
    return true;
}

void ModeMission::first_segment_init(){
    start_yaw_location = copter.current_loc;
    ahrs.get_quat_body_to_ned(start_yaw_quat);

    //wp_nav->set_wp_origin_and_destination();
    gcs().send_text(MAV_SEVERITY_INFO, "First Segment has started");
    //attitude_control->set_accel_yaw_max(10.0f);
    output_rate = true;
    yaw_variable = YAW_RATE;

    
    wp_nav->set_wp_destination(travel_setpoint, true);
    pos_control->init_xy_controller();
}


void ModeMission::second_segment_init(){
    gcs().send_text(MAV_SEVERITY_INFO, "Second Segment has started");

    wp_nav->set_wp_destination(Vector3f(0.0f, 0.0f, 700.0f), true);
    pos_control->init_xy_controller();
}


void ModeMission::last_segment_init(){
    output_rate = false;
    yaw_variable = YAW_RATE / 2.0f;
    
    wp_nav->set_wp_destination(Vector3f(0.0f, 0.0f, 200.0f), true);
    gcs().send_text(MAV_SEVERITY_INFO, "Third Segment has started");

    current_circle_point = Vector3f(200.0f, 0.0f, 500.0f);
    pos_control->init_xy_controller();
}

void ModeMission::return_init(){
    gcs().send_text(MAV_SEVERITY_INFO, "We are returning Home");

    has_started_leg = true;
    output_rate = false;

    wp_nav->set_wp_destination(Vector3f(0.0f ,0.0f ,500.0f), true);
}


void ModeMission::first_segment_finish(){
    //current_location = copter.current_loc;
    
    wp_nav->get_wp_stopping_point(stopping_point);

    wp_nav->set_wp_destination(stopping_point, true);
    leg_has_finished = true;

    gcs().send_text(MAV_SEVERITY_INFO, "First Leg is finished");

    mission_leg = 2;
    has_started_leg = false;
}


void ModeMission::second_segment_finish(){
    
    output_rate = false;
    Quaternion q;
    ahrs.get_quat_body_to_ned(q);
    
    if(!(abs(q.get_euler_yaw() - start_yaw_quat.get_euler_yaw()) < MISSION_ANGLE_SENSITIVITY)){
        //attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
        return;
    }
    
    //auto_yaw.set_mode(autopilot_yaw_mode::AUTO_YAW_FIXED);
    //auto_yaw.set_fixed_yaw(0.0, 1000.0f, 1, true);
    
    wp_nav->get_wp_stopping_point(stopping_point);
    wp_nav->set_wp_destination(stopping_point, true);
    //attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    gcs().send_text(MAV_SEVERITY_INFO, "We are close to Initial yaw. Finished Second Leg.");
    
    has_started_leg = false;
    leg_has_finished = true;

    mission_leg = 3;
}

void ModeMission::last_segment_finish(){
    gcs().send_text(MAV_SEVERITY_INFO, "Third leg is finished. Coming Home.");

    // wp_nav->get_wp_stopping_point(stopping_point);
    // wp_nav->set_wp_destination(stopping_point, true);

    has_started_leg = false;
    mission_leg = 4;
}

void ModeMission::return_finish(){
    wp_nav->get_wp_stopping_point(stopping_point);
    wp_nav->set_wp_destination(stopping_point, true);

    copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
}
