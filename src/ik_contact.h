#pragma once

// IK
extern bool ik_enabled;
extern float ik_max_length_buffer;
extern float ik_foot_height;
extern float ik_toe_length;
extern float ik_unlock_radius;
extern float ik_blending_halflife;


/// <summary>
/// prev_position: only store to calculate velocity
/// </summary>
struct Contact
{
	int index = -1;

	bool prev_state = false;
	bool lock = false;

	vec3 position;
	vec3 prev_position;
	vec3 fixed_point;

	vec3 velocity;

	vec3 offset_position;
	vec3 offset_velocity;

	Contact(int index_) { index = index_; }

	void reset(const vec3 pos, const vec3 vel, const bool state = false);

	void update(
		const vec3 cur_position,
		const bool cur_state,
		const float dt,
		const float eps = 1e-8);

	void draw();
};


// Rotate a joint to look toward some 
// given target position
void ik_look_at(
	quat& bone_rotation,
	const quat global_parent_rotation,
	const quat global_rotation,
	const vec3 global_position,
	const vec3 child_position,
	const vec3 target_position,
	const float eps = 1e-5f);

// Basic two-joint IK in the style of https://theorangeduck.com/page/simple-two-joint
// Here I add a basic "forward vector" which acts like a kind of pole-vector
// to control the bending direction
void ik_two_bone(
	quat& bone_root_lr,
	quat& bone_mid_lr,
	const vec3 bone_root,
	const vec3 bone_mid,
	const vec3 bone_end,
	const vec3 target,
	const vec3 fwd,
	const quat bone_root_gr,
	const quat bone_mid_gr,
	const quat bone_par_gr,
	const float max_length_buffer);

/// <summary>
/// set contacts' positions and velocity to current bone's positions and velocity
/// </summary>
/// <param name="bone_positions"></param>
/// <param name="bone_velocities"></param>
/// <param name="bone_rotations"></param>
/// <param name="bone_angular_velocities"></param>
/// <param name="bone_parents"></param>
void contacts_reset(const slice1d<vec3> bone_positions,
	const slice1d<vec3> bone_velocities,
	const slice1d<quat> bone_rotations,
	const slice1d<vec3> bone_angular_velocities,
	const slice1d<int> bone_parents);

void contacts_update(array1d<vec3>& global_bone_positions,
	array1d<quat>& global_bone_rotations,
	array1d<bool>& global_bone_computed,
	array1d<vec3>& bone_positions,
	array1d<quat>& bone_rotations,
	array1d<vec3>& adjusted_bone_positions,
	array1d<quat>& adjusted_bone_rotations,
	const slice1d<bool> curr_bone_contacts,
	const slice1d<int> bone_parents,
	float dt);

void contacts_draw();