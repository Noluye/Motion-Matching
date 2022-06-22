#include "mmpch.h"
#include "ik_contact.h"
#include "character.h"
#include "database.h"

bool ik_enabled = true;
float ik_max_length_buffer = 0.015f;
float ik_foot_height = 0.02f;
float ik_toe_length = 0.15f;
float ik_unlock_radius = 0.2f;
float ik_blending_halflife = 0.1f;

array1d<int> contact_bones;

array1d<bool> contact_states;
array1d<bool> contact_locks;
array1d<vec3> contact_positions;
array1d<vec3> contact_velocities;
array1d<vec3> contact_points;
array1d<vec3> contact_targets;
array1d<vec3> contact_offset_positions;
array1d<vec3> contact_offset_velocities;

void ik_look_at(
	quat& bone_rotation,
	const quat global_parent_rotation,
	const quat global_rotation,
	const vec3 global_position,
	const vec3 child_position,
	const vec3 target_position,
	const float eps)
{
	vec3 curr_dir = normalize(child_position - global_position);
	vec3 targ_dir = normalize(target_position - global_position);

	if (fabs(1.0f - dot(curr_dir, targ_dir) > eps))
	{
		bone_rotation = quat_inv_mul(global_parent_rotation,
			quat_mul(quat_between(curr_dir, targ_dir), global_rotation));
	}
}

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
	const float max_length_buffer) {

	float max_extension =
		length(bone_root - bone_mid) +
		length(bone_mid - bone_end) -
		max_length_buffer;

	vec3 target_clamp = target;
	if (length(target - bone_root) > max_extension)
	{
		target_clamp = bone_root + max_extension * normalize(target - bone_root);
	}

	vec3 axis_dwn = normalize(bone_end - bone_root);
	vec3 axis_rot = normalize(cross(axis_dwn, fwd));

	vec3 a = bone_root;
	vec3 b = bone_mid;
	vec3 c = bone_end;
	vec3 t = target_clamp;

	float lab = length(b - a);
	float lcb = length(b - c);
	float lat = length(t - a);

	float ac_ab_0 = acosf(clampf(dot(normalize(c - a), normalize(b - a)), -1.0f, 1.0f));
	float ba_bc_0 = acosf(clampf(dot(normalize(a - b), normalize(c - b)), -1.0f, 1.0f));

	float ac_ab_1 = acosf(clampf((lab * lab + lat * lat - lcb * lcb) / (2.0f * lab * lat), -1.0f, 1.0f));
	float ba_bc_1 = acosf(clampf((lab * lab + lcb * lcb - lat * lat) / (2.0f * lab * lcb), -1.0f, 1.0f));

	quat r0 = quat_from_angle_axis(ac_ab_1 - ac_ab_0, axis_rot);
	quat r1 = quat_from_angle_axis(ba_bc_1 - ba_bc_0, axis_rot);

	vec3 c_a = normalize(bone_end - bone_root);
	vec3 t_a = normalize(target_clamp - bone_root);

	quat r2 = quat_from_angle_axis(
		acosf(clampf(dot(c_a, t_a), -1.0f, 1.0f)),
		normalize(cross(c_a, t_a)));

	bone_root_lr = quat_inv_mul(bone_par_gr, quat_mul(r2, quat_mul(r0, bone_root_gr)));
	bone_mid_lr = quat_inv_mul(bone_root_gr, quat_mul(r1, bone_mid_gr));
}

void foot_lock_reset(const slice1d<vec3> bone_positions,
	const slice1d<vec3> bone_velocities,
	const slice1d<quat> bone_rotations,
	const slice1d<vec3> bone_angular_velocities,
	const slice1d<int> bone_parents)
{
	for (int i = 0; i < contact_bones.size; i++)
	{
		vec3 bone_position;
		vec3 bone_velocity;
		quat bone_rotation;
		vec3 bone_angular_velocity;

		forward_kinematics_velocity(
			bone_position,
			bone_velocity,
			bone_rotation,
			bone_angular_velocity,
			bone_positions,
			bone_velocities,
			bone_rotations,
			bone_angular_velocities,
			bone_parents,
			contact_bones(i));

		contact_reset(i, bone_position, bone_velocity);
	}
}

void contact_init(const slice1d<vec3> bone_positions,
	const slice1d<vec3> bone_velocities,
	const slice1d<quat> bone_rotations,
	const slice1d<vec3> bone_angular_velocities,
	const slice1d<int> bone_parents)
{
	contact_bones.resize(2);
	contact_bones(0) = Bone_LeftToe;
	contact_bones(1) = Bone_RightToe;

	contact_states.resize(contact_bones.size);
	contact_locks.resize(contact_bones.size);
	contact_positions.resize(contact_bones.size);
	contact_velocities.resize(contact_bones.size);
	contact_points.resize(contact_bones.size);
	contact_targets.resize(contact_bones.size);
	contact_offset_positions.resize(contact_bones.size);
	contact_offset_velocities.resize(contact_bones.size);

	foot_lock_reset(bone_positions,
		bone_velocities,
		bone_rotations,
		bone_angular_velocities,
		bone_parents);
}

void contact_reset(
	int i,
	const vec3 input_contact_position,
	const vec3 input_contact_velocity,
	const bool input_contact_state)
{
	contact_states(i) = input_contact_state;
	contact_locks(i) = input_contact_state;
	contact_positions(i) = input_contact_position;
	contact_velocities(i) = input_contact_velocity;
	contact_points(i) = input_contact_position;
	contact_targets(i) = input_contact_position;
	contact_offset_positions(i) = vec3();
	contact_offset_velocities(i) = vec3();
}

static void contact_update(
	int i,
	const vec3 input_contact_position,
	const bool input_contact_state,
	const float unlock_radius,
	const float foot_height,
	const float halflife,
	const float dt,
	const float eps = 1e-8)
{
	// First compute the input contact position velocity via finite difference
	vec3 input_contact_velocity =
		(input_contact_position - contact_targets(i)) / (dt + eps);
	contact_targets(i) = input_contact_position;

	// Update the inertializer to tick forward in time
	inertialize_update(
		contact_positions(i),
		contact_velocities(i),
		contact_offset_positions(i),
		contact_offset_velocities(i),
		// If locked we feed the contact point and zero velocity, 
		// otherwise we feed the input from the animation
		contact_locks(i) ? contact_points(i) : input_contact_position,
		contact_locks(i) ? vec3() : input_contact_velocity,
		halflife,
		dt);

	// If the contact point is too far from the current input position 
	// then we need to unlock the contact
	bool unlock_contact = contact_locks(i) && (
		length(contact_points(i) - input_contact_position) > unlock_radius);

	// If the contact was previously inactive but is now active we 
	// need to transition to the locked contact state
	if (!contact_states(i) && input_contact_state)
	{
		// Contact point is given by the current position of 
		// the foot projected onto the ground plus foot height
		contact_locks(i) = true;
		contact_points(i) = contact_positions(i);
		contact_points(i).y = foot_height;

		inertialize_transition(
			contact_offset_positions(i),
			contact_offset_velocities(i),
			input_contact_position,
			input_contact_velocity,
			contact_points(i),
			vec3());
	}

	// Otherwise if we need to unlock or we were previously in 
	// contact but are no longer we transition to just taking 
	// the input position as-is
	else if ((contact_locks(i) && contact_states(i) && !input_contact_state)
		|| unlock_contact)
	{
		contact_locks(i) = false;

		inertialize_transition(
			contact_offset_positions(i),
			contact_offset_velocities(i),
			contact_points(i),
			vec3(),
			input_contact_position,
			input_contact_velocity);
	}

	// Update contact state
	contact_states(i) = input_contact_state;
}

void draw_foot_lock_positions()
{
	for (int i = 0; i < contact_positions.size; i++)
	{
		if (contact_locks(i))
		{
			DrawSphereWires(to_Vector3(contact_positions(i)), 0.05f, 4, 10, PINK);
		}
	}
}

void ik_contact_update(array1d<vec3>& global_bone_positions,
	array1d<quat>& global_bone_rotations,
	array1d<bool>& global_bone_computed,
	array1d<vec3>& bone_positions,
	array1d<quat>& bone_rotations,
	array1d<vec3>& adjusted_bone_positions,
	array1d<quat>& adjusted_bone_rotations,
	const slice1d<bool> curr_bone_contacts,
	const slice1d<int> bone_parents,
	float dt)
{
	if (ik_enabled)
	{
		for (int i = 0; i < contact_bones.size; i++)
		{
			// Find all the relevant bone indices
			int toe_bone  = contact_bones(i);
			int heel_bone = bone_parents(toe_bone);
			int knee_bone = bone_parents(heel_bone);
			int hip_bone  = bone_parents(knee_bone);
			int root_bone = bone_parents(hip_bone);

			// Compute the world space position for the toe
			global_bone_computed.zero();

			forward_kinematics_partial(
				global_bone_positions,
				global_bone_rotations,
				global_bone_computed,
				bone_positions,
				bone_rotations,
				bone_parents,
				toe_bone);

			// Update the contact state
			contact_update(
				i,
				global_bone_positions(toe_bone),
				curr_bone_contacts(i),
				ik_unlock_radius,
				ik_foot_height,
				ik_blending_halflife,
				dt);

			// Ensure contact position never goes through floor
			vec3 contact_position_clamp = contact_positions(i);
			contact_position_clamp.y = maxf(contact_position_clamp.y, ik_foot_height);

			// Re-compute toe, heel, knee, hip, and root bone positions
			for (int bone : {heel_bone, knee_bone, hip_bone, root_bone})
			{
				forward_kinematics_partial(
					global_bone_positions,
					global_bone_rotations,
					global_bone_computed,
					bone_positions,
					bone_rotations,
					bone_parents,
					bone);
			}

			// Perform simple two-joint IK to place heel
			ik_two_bone(
				adjusted_bone_rotations(hip_bone),
				adjusted_bone_rotations(knee_bone),
				global_bone_positions(hip_bone),
				global_bone_positions(knee_bone),
				global_bone_positions(heel_bone),
				contact_position_clamp + (global_bone_positions(heel_bone) - global_bone_positions(toe_bone)),
				quat_mul_vec3(global_bone_rotations(knee_bone), vec3(0.0f, 1.0f, 0.0f)),
				global_bone_rotations(hip_bone),
				global_bone_rotations(knee_bone),
				global_bone_rotations(root_bone),
				ik_max_length_buffer);

			// Re-compute toe, heel, and knee positions 
			global_bone_computed.zero();

			for (int bone : {toe_bone, heel_bone, knee_bone})
			{
				forward_kinematics_partial(
					global_bone_positions,
					global_bone_rotations,
					global_bone_computed,
					adjusted_bone_positions,
					adjusted_bone_rotations,
					bone_parents,
					bone);
			}

			// Rotate heel so toe is facing toward contact point
			ik_look_at(
				adjusted_bone_rotations(heel_bone),
				global_bone_rotations(knee_bone),
				global_bone_rotations(heel_bone),
				global_bone_positions(heel_bone),
				global_bone_positions(toe_bone),
				contact_position_clamp);

			// Re-compute toe and heel positions
			global_bone_computed.zero();

			for (int bone : {toe_bone, heel_bone})
			{
				forward_kinematics_partial(
					global_bone_positions,
					global_bone_rotations,
					global_bone_computed,
					adjusted_bone_positions,
					adjusted_bone_rotations,
					bone_parents,
					bone);
			}

			// Rotate toe bone so that the end of the toe 
			// does not intersect with the ground
			vec3 toe_end_curr = quat_mul_vec3(
				global_bone_rotations(toe_bone), vec3(ik_toe_length, 0.0f, 0.0f)) +
				global_bone_positions(toe_bone);

			vec3 toe_end_targ = toe_end_curr;
			toe_end_targ.y = maxf(toe_end_targ.y, ik_foot_height);

			ik_look_at(
				adjusted_bone_rotations(toe_bone),
				global_bone_rotations(heel_bone),
				global_bone_rotations(toe_bone),
				global_bone_positions(toe_bone),
				toe_end_curr,
				toe_end_targ);
		}
	}
}