#pragma once

// IK
extern bool ik_enabled;
extern float ik_max_length_buffer;
extern float ik_foot_height;
extern float ik_toe_length;
extern float ik_unlock_radius;
extern float ik_blending_halflife;

// Contact and Foot Locking data
extern array1d<int> contact_bones;

extern array1d<bool> contact_states;
extern array1d<bool> contact_locks;
extern array1d<vec3> contact_positions;
extern array1d<vec3> contact_velocities;
extern array1d<vec3> contact_points;
extern array1d<vec3> contact_targets;
extern array1d<vec3> contact_offset_positions;
extern array1d<vec3> contact_offset_velocities;

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

void foot_lock_reset(const slice1d<vec3> bone_positions,
	const slice1d<vec3> bone_velocities,
	const slice1d<quat> bone_rotations,
	const slice1d<vec3> bone_angular_velocities,
	const slice1d<int> bone_parents);

void contact_init(const slice1d<vec3> bone_positions,
	const slice1d<vec3> bone_velocities,
	const slice1d<quat> bone_rotations,
	const slice1d<vec3> bone_angular_velocities,
	const slice1d<int> bone_parents);

void contact_reset(int i,
	const vec3 input_contact_position,
	const vec3 input_contact_velocity,
	const bool input_contact_state = false);

void ik_contact_update(array1d<vec3>& global_bone_positions,
	array1d<quat>& global_bone_rotations,
	array1d<bool>& global_bone_computed,
	array1d<vec3>& bone_positions,
	array1d<quat>& bone_rotations,
	array1d<vec3>& adjusted_bone_positions,
	array1d<quat>& adjusted_bone_rotations,
	const slice1d<bool> curr_bone_contacts,
	const slice1d<int> bone_parents,
	float dt);

void draw_foot_lock_positions();