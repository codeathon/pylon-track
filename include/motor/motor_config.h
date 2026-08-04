#pragma once

#include "experiment/arena_config.h"
#include "motor/prey_motor.h"

// Map arena JSON motor section to PreyMotor driver config.
inline PreyMotorConfig prey_motor_from_config(const MotorConfig& cfg) {
	PreyMotorConfig out;
	out.can_interface = cfg.can_interface;
	out.node_id = cfg.node_id;
	out.pulley_radius_m = cfg.pulley_radius_m;
	out.chain_direction_sign = cfg.chain_direction_sign;
	out.chain_mm_per_motor_turn = cfg.chain_mm_per_motor_turn;
	out.chain_inertia_kg_m2 = cfg.chain_inertia_kg_m2;
	out.chain_viscous_friction_nm_s_per_rad = cfg.chain_viscous_friction_nm_s_per_rad;
	out.chain_static_friction_nm = cfg.chain_static_friction_nm;
	out.torque_constant_nm_per_a = cfg.torque_constant_nm_per_a;
	return out;
}
