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

std::vector<Contact> contacts = {
	{Bone_LeftToe}, {Bone_RightToe}
};

void Contact::reset(const vec3 pos, const vec3 vel, const bool state)
{
	prev_position = fixed_point = position = pos;
	velocity = vel;
	lock = prev_state = state;
	offset_velocity = offset_position = vec3();
}

void Contact::update(
	const vec3 cur_position,
	const bool cur_state,
	const float dt,
	const float eps)
{

	// First compute the input contact position velocity via finite difference
	vec3 cur_velocity = (cur_position - prev_position) / (dt + eps);
	prev_position = cur_position;

	// Update the inertializer to tick forward in time
	// Input: offset_position & offset_position(will be updated)
	// Input: fixed_point/cur_position
	// Input: vec3()/cur_velocity
	inertialize_update(
		position,
		velocity,
		offset_position,
		offset_velocity,
		// If locked we feed the contact point and zero velocity, 
		// otherwise we feed the input from the animation
		lock ? fixed_point : cur_position,
		lock ? vec3() : cur_velocity,
		ik_blending_halflife,
		dt);

	// If the contact was previously inactive but is now active we 
	// need to transition to the locked contact state
	if (!prev_state && cur_state)
	{
		// Contact point is given by the current position of 
		// the foot projected onto the ground plus foot height
		lock = true;
		fixed_point = position;
		fixed_point.y = ik_foot_height;

		inertialize_transition(
			offset_position,
			offset_velocity,
			cur_position,
			cur_velocity,
			fixed_point,
			vec3());
	}

	// Otherwise if we need to unlock or we were previously in 
	// contact but are no longer we transition to just taking 
	// the input position as-is
	else if (lock && (
		(prev_state && !cur_state) ||
		// If the contact point is too far from the current input position
		// then we need to unlock the contact
		length(fixed_point - cur_position) > ik_unlock_radius 
		))
	{
		lock = false;

		inertialize_transition(
			offset_position,
			offset_velocity,
			fixed_point,
			vec3(),
			cur_position,
			cur_velocity);
	}

	// Update contact state
	prev_state = cur_state;
}

void Contact::draw()
{
	if (lock) DrawSphereWires(to_Vector3(position), 0.05f, 4, 10, PINK);
}




// ------------------------------------------------------------
// contacts

void contacts_reset(const slice1d<vec3> bone_positions,
	const slice1d<vec3> bone_velocities,
	const slice1d<quat> bone_rotations,
	const slice1d<vec3> bone_angular_velocities,
	const slice1d<int> bone_parents)
{
	for (auto& contact : contacts)
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
			contact.index);
		contact.reset(bone_position, bone_velocity);
	}
}

void contacts_update(array1d<vec3>& global_bone_positions,
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
	if (!ik_enabled) return;

	for (int i = 0; i < contacts.size(); ++i)
	{
		auto& contact = contacts[i];

		// Find all the relevant bone indices
		int toe_bone = contact.index;
		int heel_bone = bone_parents(toe_bone);
		int knee_bone = bone_parents(heel_bone);
		int hip_bone = bone_parents(knee_bone);
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
		contact.update(global_bone_positions(toe_bone), curr_bone_contacts(i), dt);

		// Ensure contact position never goes through floor
		vec3 contact_position_clamp = contact.position;
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


void contacts_draw()
{
	for (auto& contact : contacts) contact.draw();
}
// ------------------------------------------------------------
// IK
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
