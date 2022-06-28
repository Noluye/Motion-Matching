#include "mmpch.h"
#include "camera.h"
#include "gamepad.h"


float orbit_camera_update_azimuth(
	const float azimuth,
	const vec3 gamepadstick_right,
	const bool desired_strafe,
	const float dt)
{
	vec3 gamepadaxis = desired_strafe ? vec3() : gamepadstick_right;
	return azimuth + 2.0f * dt * -gamepadaxis.x;
}

float orbit_camera_update_altitude(
	const float altitude,
	const vec3 gamepadstick_right,
	const bool desired_strafe,
	const float dt)
{
	vec3 gamepadaxis = desired_strafe ? vec3() : gamepadstick_right;
	return clampf(altitude + 2.0f * dt * gamepadaxis.z, 0.0, 0.4f * PIf);
}

float orbit_camera_update_distance(
	const float distance,
	const float dt)
{
	float gamepadzoom =
		IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_LEFT_TRIGGER_1) ? +1.0f :
		IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) ? -1.0f : 0.0f;

	return clampf(distance + 10.0f * dt * gamepadzoom, 0.1f, 100.0f);
}


void orbit_camera_update(
	Camera3D& cam,
	float& camera_azimuth,
	float& camera_altitude,
	float& camera_distance,
	const vec3 target,
	const vec3 gamepadstick_right,
	const bool desired_strafe,
	const float dt)
{
	camera_azimuth = orbit_camera_update_azimuth(camera_azimuth, gamepadstick_right, desired_strafe, dt);
	camera_altitude = orbit_camera_update_altitude(camera_altitude, gamepadstick_right, desired_strafe, dt);
	camera_distance = orbit_camera_update_distance(camera_distance, dt);

	quat rotation_azimuth = quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0));
	vec3 position = quat_mul_vec3(rotation_azimuth, vec3(0, 0, camera_distance));
	vec3 axis = normalize(cross(position, vec3(0, 1, 0)));

	quat rotation_altitude = quat_from_angle_axis(camera_altitude, axis);

	vec3 eye = target + quat_mul_vec3(rotation_altitude, position);

	cam.target = Vector3{ target.x, target.y, target.z };
	cam.position = Vector3{ eye.x, eye.y, eye.z };

	UpdateCamera(&cam);
}
