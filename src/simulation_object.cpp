#include "mmpch.h"
#include "simulation_object.h"
#include "gamepad.h"

float desired_gait = 0.0f;
float desired_gait_velocity = 0.0f;

bool desired_strafe_update()
{
	return IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_LEFT_TRIGGER_2) > 0.5f;
}

vec3 desired_velocity_update(
	const vec3 gamepadstick_left,
	const float camera_azimuth,
	const quat simulation_rotation,
	const float fwrd_speed,
	const float side_speed,
	const float back_speed)
{
	// Find stick position in world space by rotating using camera azimuth
	vec3 global_stick_direction = quat_mul_vec3(
		quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), gamepadstick_left);

	// Find stick position local to current facing direction
	vec3 local_stick_direction = quat_inv_mul_vec3(
		simulation_rotation, global_stick_direction);

	// Scale stick by forward, sideways and backwards speeds
	vec3 local_desired_velocity = local_stick_direction.z > 0.0 ?
		vec3(side_speed, 0.0f, fwrd_speed) * local_stick_direction :
		vec3(side_speed, 0.0f, back_speed) * local_stick_direction;

	// Re-orientate into the world space
	return quat_mul_vec3(simulation_rotation, local_desired_velocity);
}

void desired_gait_update(
	float& desired_gait,
	float& desired_gait_velocity,
	const float dt,
	const float gait_change_halflife)
{
	simple_spring_damper_implicit(
		desired_gait,
		desired_gait_velocity,
		IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_RIGHT_FACE_DOWN) ? 1.0f : 0.0f,
		gait_change_halflife,
		dt);
}


quat desired_rotation_update(
	const quat desired_rotation,
	const vec3 gamepadstick_left,
	const vec3 gamepadstick_right,
	const float camera_azimuth,
	const bool desired_strafe,
	const vec3 desired_velocity)
{
	quat desired_rotation_curr = desired_rotation;

	// If strafe is active then desired direction is coming from right
	// stick as long as that stick is being used, otherwise we assume
	// forward facing
	if (desired_strafe)
	{
		vec3 desired_direction = quat_mul_vec3(quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), vec3(0, 0, -1));

		if (length(gamepadstick_right) > 0.01f)
		{
			desired_direction = quat_mul_vec3(quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), normalize(gamepadstick_right));
		}

		return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), vec3(0, 1, 0));
	}

	// If strafe is not active the desired direction comes from the left 
	// stick as long as that stick is being used
	else if (length(gamepadstick_left) > 0.01f)
	{

		vec3 desired_direction = normalize(desired_velocity);
		return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), vec3(0, 1, 0));
	}

	// Otherwise desired direction remains the same
	else
	{
		return desired_rotation_curr;
	}
}
