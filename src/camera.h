#pragma once

float orbit_camera_update_azimuth(
	const float azimuth,
	const vec3 gamepadstick_right,
	const bool desired_strafe,
	const float dt);

float orbit_camera_update_altitude(
	const float altitude,
	const vec3 gamepadstick_right,
	const bool desired_strafe,
	const float dt);

float orbit_camera_update_distance(
	const float distance,
	const float dt);

// Updates the camera using the orbit cam controls
void orbit_camera_update(
	Camera3D& cam,
	float& camera_azimuth,
	float& camera_altitude,
	float& camera_distance,
	const vec3 target,
	const vec3 gamepadstick_right,
	const bool desired_strafe,
	const float dt);