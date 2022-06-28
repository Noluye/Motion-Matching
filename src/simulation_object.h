#pragma once

extern float desired_gait;
extern float desired_gait_velocity;

bool desired_strafe_update();

vec3 desired_velocity_update(
	const vec3 gamepadstick_left,
	const float camera_azimuth,
	const quat simulation_rotation,
	const float fwrd_speed,
	const float side_speed,
	const float back_speed);

void desired_gait_update(
	float& desired_gait,
	float& desired_gait_velocity,
	const float dt,
	const float gait_change_halflife = 0.1f);

quat desired_rotation_update(
	const quat desired_rotation,
	const vec3 gamepadstick_left,
	const vec3 gamepadstick_right,
	const float camera_azimuth,
	const bool desired_strafe,
	const vec3 desired_velocity);