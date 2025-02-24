/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  StratoBlimp simulator class
*/

#include "SIM_StratoBlimp.h"

#if AP_SIM_STRATOBLIMP_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace EXTERNALFC;

extern const AP_HAL::HAL& hal;

// SITL StratoBlimp parameters
const AP_Param::GroupInfo StratoBlimp::var_info[] = {
    // @Param: MASS
    // @DisplayName: mass
    // @Description: mass of blimp not including lifting gas
    // @Units: kg
    AP_GROUPINFO("MASS",     1, StratoBlimp,  mass, 80),

    // @Param: HMASS
    // @DisplayName: helium mass
    // @Description: mass of lifting gas
    // @Units: kg
    AP_GROUPINFO("HMASS",    2, StratoBlimp,  helium_mass, 13.54),

    // @Param: ARM_LEN
    // @DisplayName: arm length
    // @Description: distance from center of mass to one motor
    // @Units: m
    AP_GROUPINFO("ARM_LEN",  3, StratoBlimp,  arm_length, 3.6),

    // @Param: MOT_THST
    // @DisplayName: motor thrust
    // @Description: thrust at max throttle for one motor
    // @Units: N
    AP_GROUPINFO("MOT_THST", 4, StratoBlimp,  motor_thrust, 145),

    // @Param: DRAG_FWD
    // @DisplayName: drag in forward direction
    // @Description: drag on X axis
    AP_GROUPINFO("DRAG_FWD", 5, StratoBlimp,  drag_fwd, 0.27),

    // @Param: DRAG_SIDE
    // @DisplayName: drag in sidewards direction
    // @Description: drag on Y axis
    AP_GROUPINFO("DRAG_SIDE",  16, StratoBlimp,  drag_side, 0.5),

    // @Param: DRAG_UP
    // @DisplayName: drag in upward direction
    // @Description: drag on Z axis
    AP_GROUPINFO("DRAG_UP",  6, StratoBlimp,  drag_up, 0.4),

    // @Param: MOI_YAW
    // @DisplayName: moment of inertia in yaw
    // @Description: moment of inertia in yaw
    AP_GROUPINFO("MOI_YAW",  7, StratoBlimp,  moi_yaw, 2800),

    // @Param: MOI_ROLL
    // @DisplayName: moment of inertia in roll
    // @Description: moment of inertia in roll
    AP_GROUPINFO("MOI_ROLL", 8, StratoBlimp,  moi_roll, 1400),

    // @Param: MOI_PITCH
    // @DisplayName: moment of inertia in pitch
    // @Description: moment of inertia in pitch
    AP_GROUPINFO("MOI_PITCH", 9, StratoBlimp,  moi_pitch, 3050),
    
    // @Param: ALT_TARG
    // @DisplayName: altitude target
    // @Description: altitude target
    // @Units: m
    AP_GROUPINFO("ALT_TARG", 10, StratoBlimp,  altitude_target, 20000),

    // @Param: CLMB_RT
    // @DisplayName: target climb rate
    // @Description: target climb rate
    // @Units: m/s
    AP_GROUPINFO("CLMB_RT",  11, StratoBlimp, target_climb_rate, 5),

    // @Param: YAW_RT
    // @DisplayName: yaw rate
    // @Description: maximum yaw rate with full left throttle at target altitude
    // @Units: deg/s
    AP_GROUPINFO("YAW_RT",   12, StratoBlimp, yaw_rate_max, 60),

    // @Param: MOT_ANG
    // @DisplayName: motor angle
    // @Description: maximum motor tilt angle
    // @Units: deg
    AP_GROUPINFO("MOT_ANG",  13, StratoBlimp,  motor_angle, 20),

    // @Param: COL
    // @DisplayName: center of lift
    // @Description: center of lift position above CoG
    // @Units: m
    AP_GROUPINFO("COL",      14, StratoBlimp,  center_of_lift, 2.54),

    // @Param: WVANE
    // @DisplayName: weathervaning offset
    // @Description: center of drag for weathervaning
    // @Units: m
    AP_GROUPINFO("WVANE",    15, StratoBlimp,  center_of_drag, 0.3),

    // @Param: FLR
    // @DisplayName: free lift rate
    // @Description: amount of additional lift generated by the helper balloon (for the purpose of ascent), as a proportion of the 'neutral buoyancy' lift
    AP_GROUPINFO("FLR",  17, StratoBlimp,  free_lift_rate, 0.12),
    
    AP_GROUPEND
};

StratoBlimp::StratoBlimp(const char *frame_str) :
    Aircraft(frame_str)
{
    AP::sitl()->models.stratoblimp_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  calculate coefficients to match parameters
 */
void StratoBlimp::calculate_coefficients(void)
{
    // calculate yaw drag based on turn rate at the given altitude
    drag_yaw = 1.0;

    // get full throttle rotational accel for one motor
    Vector3f body_acc, rot_accel;
    handle_motor(1, 0, body_acc, rot_accel, -arm_length);

    // get rotational drag at target alt
    Vector3f vel_bf, g, drag_linear, drag_rotaccel;
    g.z = radians(yaw_rate_max);

    get_drag(vel_bf, g,
             altitude_target,
             drag_linear, drag_rotaccel);

    drag_yaw = rot_accel.z / -drag_rotaccel.z;
}

void StratoBlimp::handle_motor(float throttle, float tilt, Vector3f &body_acc, Vector3f &rot_accel, float lateral_position)
{
    const float angle_rad = radians(motor_angle) * tilt;
    const float thrust_x = motor_thrust * throttle;
    const float total_mass = mass + helium_mass;

    const Vector3f thrust{cosf(angle_rad)*thrust_x, 0, -sinf(angle_rad)*thrust_x}; // assume constant with pressure alt and linear
    Vector3f accel = thrust / total_mass;
    Vector3f pos{0, lateral_position, 0};

    Vector3f torque = (pos % thrust);

    rot_accel.z += torque.z / moi_yaw;
    body_acc += accel;
}


/*
  get body frame linear and rotational drag for a given velocity and altitude
 */
void StratoBlimp::get_drag(const Vector3f &velocity_linear,
                           const Vector3f &velocity_rot,
                           float altitude,
                           Vector3f &drag_linear, Vector3f &drag_rotaccel)
{
    Vector3f vel_air_bf = velocity_linear;
    const float drag_x_sign = vel_air_bf.x>0? -1 : 1;
    const float drag_y_sign = vel_air_bf.y>0? -1 : 1;
    const float drag_z_sign = vel_air_bf.z>0? -1 : 1;
    drag_linear.x = 0.5 * drag_x_sign * air_density * sq(vel_air_bf.x) * drag_fwd;
    drag_linear.y = 0.5 * drag_y_sign * air_density * sq(vel_air_bf.y) * drag_fwd;
    drag_linear.z = 0.5 * drag_z_sign * air_density * sq(vel_air_bf.z) * drag_up;

    drag_rotaccel = -velocity_rot * drag_yaw;

    /*
      apply torque from drag
    */
    Vector3f drag_force = drag_linear * mass;
    Vector3f drag_pos{-center_of_drag, 0, -center_of_lift};
    Vector3f drag_torque = (drag_pos % drag_force);
    drag_rotaccel += drag_torque / moi_pitch;
}

/*
  get vertical thrust from lift in Newtons
 */
float StratoBlimp::get_lift(float altitude)
{
    // start with neutral buoyancy
    float lift_accel = GRAVITY_MSS;

    // add lift from helper balloon if still attached
    if (helper_balloon_attached) {
        // helper balloon additional lift amount based on Free Lift Ratio
        lift_accel += GRAVITY_MSS*free_lift_rate;
        // detach helper balloon if the target altitude has been reached
        if (altitude >= altitude_target) {
            helper_balloon_attached = false;
        }
    }
    return mass * lift_accel;
}

// calculate rotational and linear accelerations in body frame
void StratoBlimp::calculate_forces(const struct sitl_input &input, Vector3f &body_acc, Vector3f &rot_accel)
{
    //float delta_time = frame_time_us * 1.0e-6f;

    if (!hal.scheduler->is_system_initialized()) {
        return;
    }

    const float left_tilt = filtered_servo_angle(input, 0);
    const float right_tilt = filtered_servo_angle(input, 1);

    const float left_throttle = filtered_servo_range(input, 2);
    const float right_throttle = filtered_servo_range(input, 3);
    const float ground_release = filtered_servo_range(input, 4);

    body_acc.zero();
    rot_accel.zero();

    handle_motor(left_throttle, left_tilt, body_acc, rot_accel, -arm_length);
    handle_motor(right_throttle, right_tilt, body_acc, rot_accel, arm_length);

    Vector3f drag_linear, drag_rotaccel;
    get_drag(velocity_air_bf, gyro,
             location.alt*0.01,
             drag_linear, drag_rotaccel);

    body_acc += drag_linear;
    rot_accel += drag_rotaccel;

    if (ground_release > 0.9) {
        released = true;
    }
    if (released) {
        Vector3f lift_thrust_ef{0, 0, -get_lift(location.alt*0.01)};
        Vector3f lift_thrust_bf = dcm.transposed() * lift_thrust_ef;

        body_acc += lift_thrust_bf / mass;

        /*
          apply righting moment
         */
        Vector3f lift_pos{0, 0, -center_of_lift};
        Vector3f lift_torque = (lift_pos % lift_thrust_bf);
        rot_accel += lift_torque / moi_roll;
    }
}

/*
  update the airship simulation by one time step
 */
void StratoBlimp::update(const struct sitl_input &input)
{
    air_density = get_air_density(location.alt*0.01);
    EAS2TAS = sqrtf(SSL_AIR_DENSITY / air_density);

    calculate_coefficients();

    float delta_time = frame_time_us * 1.0e-6f;

    Vector3f rot_accel = Vector3f(0,0,0);
    calculate_forces(input, accel_body, rot_accel);

    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    gyro.x = constrain_float(gyro.x, -radians(2000.0f), radians(2000.0f));
    gyro.y = constrain_float(gyro.y, -radians(2000.0f), radians(2000.0f));
    gyro.z = constrain_float(gyro.z, -radians(2000.0f), radians(2000.0f));

    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    update_wind(input);
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

#endif // AP_SIM_STRATOBLIMP_ENABLED
