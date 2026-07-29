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
	return out;
}
