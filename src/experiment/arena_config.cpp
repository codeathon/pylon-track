#include "experiment/arena_config.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include "log/logger.h"

namespace {

template<typename T>
T json_or(const nlohmann::json& j, const char* key, T fallback) {
	if (j.contains(key)) {
		return j.at(key).get<T>();
	}
	return fallback;
}

} // namespace

bool load_arena_experiment_config(const std::string& path, ArenaExperimentConfig& out) {
	std::ifstream file(path);
	if (!file.is_open()) {
		log_error("experiment", "Cannot open arena config: " + path);
		return false;
	}
	try {
		const nlohmann::json j = nlohmann::json::parse(file);
		if (j.contains("chase_policy")) {
			const auto& c = j.at("chase_policy");
			out.chase.min_chain_speed_mps = json_or(c, "min_chain_speed_mps",
				out.chase.min_chain_speed_mps);
			out.chase.max_chain_speed_mps = json_or(c, "max_chain_speed_mps",
				out.chase.max_chain_speed_mps);
			out.chase.cone_half_angle_deg = json_or(c, "cone_half_angle_deg",
				out.chase.cone_half_angle_deg);
			out.chase.threat_distance_mm = json_or(c, "threat_distance_mm",
				out.chase.threat_distance_mm);
			out.chase.creep_distance_mm = json_or(c, "creep_distance_mm",
				out.chase.creep_distance_mm);
		}
		if (j.contains("trial")) {
			out.trial_timeout_s = json_or(j.at("trial"), "timeout_s", out.trial_timeout_s);
		}
		log_info("experiment", "Loaded arena config: " + path);
		return true;
	} catch (const std::exception& e) {
		log_error("experiment", std::string("Arena config parse error: ") + e.what());
		return false;
	}
}
