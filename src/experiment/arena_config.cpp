#include "experiment/arena_config.h"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include "log/logger.h"

namespace {

namespace fs = std::filesystem;

template<typename T>
T json_or(const nlohmann::json& j, const char* key, T fallback) {
	if (j.contains(key)) {
		return j.at(key).get<T>();
	}
	return fallback;
}

// Avoid nlohmann get<uint8_t> quirks — load node_id as int then clamp.
uint8_t json_node_id(const nlohmann::json& j, uint8_t fallback) {
	if (!j.contains("node_id")) {
		return fallback;
	}
	const int id = j.at("node_id").get<int>();
	if (id < 0 || id > 63) {
		return fallback;
	}
	return static_cast<uint8_t>(id);
}

std::string absolute_config_path(const std::string& path) {
	std::error_code ec;
	const fs::path abs = fs::weakly_canonical(fs::absolute(path), ec);
	return ec ? path : abs.string();
}

std::vector<cv::Point> parse_polygon_points(const nlohmann::json& points_json) {
	std::vector<cv::Point> points;
	if (!points_json.is_array()) {
		return points;
	}
	for (const auto& pt : points_json) {
		if (!pt.is_array() || pt.size() < 2) {
			continue;
		}
		points.emplace_back(pt.at(0).get<int>(), pt.at(1).get<int>());
	}
	return points;
}

void load_ignore_regions(const nlohmann::json& vision_json, ArenaMaskConfig& mask) {
	if (!vision_json.contains("ignore_regions")) {
		return;
	}
	for (const auto& region_json : vision_json.at("ignore_regions")) {
		PolygonRegion region;
		region.points = parse_polygon_points(region_json.at("points"));
		if (region.points.size() >= 3) {
			mask.ignore_regions.push_back(region);
		}
	}
}

void load_track_roi(const nlohmann::json& vision_json, ArenaMaskConfig& mask) {
	if (!vision_json.contains("track_roi") || vision_json.at("track_roi").is_null()) {
		return;
	}
	mask.track_roi = parse_polygon_points(vision_json.at("track_roi").at("points"));
	mask.has_track_roi = mask.track_roi.size() >= 3;
}

} // namespace

bool load_arena_experiment_config(const std::string& path, ArenaExperimentConfig& out) {
	const std::string abs_path = absolute_config_path(path);
	std::ifstream file(abs_path);
	if (!file.is_open()) {
		log_error("experiment", "Cannot open arena config: " + abs_path);
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
			out.chase.flee_threat_threshold = json_or(c, "flee_threat_threshold",
				out.chase.flee_threat_threshold);
			out.chase.min_flee_mm = json_or(c, "min_flee_mm", out.chase.min_flee_mm);
			out.chase.max_flee_mm = json_or(c, "max_flee_mm", out.chase.max_flee_mm);
			out.chase.flee_gap_gain = json_or(c, "flee_gap_gain", out.chase.flee_gap_gain);
			out.chase.flee_speed_gain = json_or(c, "flee_speed_gain",
				out.chase.flee_speed_gain);
			out.chase.flee_accel_mps2 = json_or(c, "flee_accel_mps2",
				out.chase.flee_accel_mps2);
			out.chase.hunt_event_min_interval_ms = json_or(c,
				"hunt_event_min_interval_ms", out.chase.hunt_event_min_interval_ms);
		}
		if (j.contains("motor")) {
			const auto& m = j.at("motor");
			// Dump raw JSON so stale/wrong-file edits are obvious on the lab machine.
			log_info("experiment", "motor section from " + abs_path + ": " + m.dump());
			out.motor.can_interface = json_or(m, "can_interface", out.motor.can_interface);
			out.motor.node_id = json_node_id(m, out.motor.node_id);
			out.motor.pulley_radius_m = json_or(m, "pulley_radius_m", out.motor.pulley_radius_m);
			out.motor.chain_direction_sign = json_or(m, "chain_direction_sign",
				out.motor.chain_direction_sign);
			out.motor.chain_mm_per_motor_turn = json_or(m, "chain_mm_per_motor_turn",
				out.motor.chain_mm_per_motor_turn);
		} else {
			log_error("experiment", "No \"motor\" section in " + abs_path);
		}
		if (j.contains("shuttle")) {
			const auto& s = j.at("shuttle");
			out.shuttle.backend = json_or(s, "backend", out.shuttle.backend);
			out.shuttle.wobble_leg_ms = json_or(s, "wobble_leg_ms", out.shuttle.wobble_leg_ms);
			out.shuttle.end_pulse_ms = json_or(s, "end_pulse_ms", out.shuttle.end_pulse_ms);
			out.shuttle.hallway_high_turns = json_or(s, "hallway_high_turns",
				out.shuttle.hallway_high_turns);
			out.shuttle.hallway_low_turns = json_or(s, "hallway_low_turns",
				out.shuttle.hallway_low_turns);
			if (s.contains("labjack")) {
				const auto& lj = s.at("labjack");
				out.shuttle.labjack.device_type = json_or(lj, "device_type",
					out.shuttle.labjack.device_type);
				out.shuttle.labjack.connection = json_or(lj, "connection",
					out.shuttle.labjack.connection);
				out.shuttle.labjack.identifier = json_or(lj, "identifier",
					out.shuttle.labjack.identifier);
				out.shuttle.labjack.pin_a = json_or(lj, "pin_a", out.shuttle.labjack.pin_a);
				out.shuttle.labjack.pin_b = json_or(lj, "pin_b", out.shuttle.labjack.pin_b);
				out.shuttle.labjack.high_voltage = json_or(lj, "high_voltage",
					out.shuttle.labjack.high_voltage);
			}
		}
		if (j.contains("vision")) {
			const auto& v = j.at("vision");
			load_ignore_regions(v, out.vision.mask);
			load_track_roi(v, out.vision.mask);
			out.vision.ferret_area_px_min = json_or(v, "ferret_area_px_min",
				out.vision.ferret_area_px_min);
			out.vision.ferret_area_px_max = json_or(v, "ferret_area_px_max",
				out.vision.ferret_area_px_max);
			out.vision.prey_area_px_min = json_or(v, "prey_area_px_min",
				out.vision.prey_area_px_min);
			out.vision.prey_area_px_max = json_or(v, "prey_area_px_max",
				out.vision.prey_area_px_max);
			out.vision.max_compactness = json_or(v, "max_compactness",
				out.vision.max_compactness);
		}
		if (j.contains("trial")) {
			out.trial_timeout_s = json_or(j.at("trial"), "timeout_s", out.trial_timeout_s);
		}
		log_info("experiment", "Loaded arena config: " + abs_path);
		return true;
	} catch (const std::exception& e) {
		log_error("experiment", std::string("Arena config parse error: ") + e.what()
			+ " (" + abs_path + ")");
		return false;
	}
}

namespace {

nlohmann::json points_to_json(const std::vector<cv::Point>& points) {
	nlohmann::json arr = nlohmann::json::array();
	for (const auto& pt : points) {
		arr.push_back({pt.x, pt.y});
	}
	return arr;
}

nlohmann::json mask_to_vision_json(const ArenaMaskConfig& mask) {
	nlohmann::json vision;
	nlohmann::json regions = nlohmann::json::array();
	for (const auto& region : mask.ignore_regions) {
		if (region.points.size() >= 3) {
			regions.push_back({{"points", points_to_json(region.points)}});
		}
	}
	vision["ignore_regions"] = regions;
	if (mask.has_track_roi && mask.track_roi.size() >= 3) {
		vision["track_roi"] = {{"points", points_to_json(mask.track_roi)}};
	} else {
		vision["track_roi"] = nullptr;
	}
	return vision;
}

bool merge_write_json_section(const std::string& path,
	const char* section, nlohmann::json section_json)
{
	std::ifstream in(path);
	if (!in.is_open()) {
		log_error("experiment", "Cannot open config for write: " + path);
		return false;
	}
	nlohmann::json root;
	try {
		in >> root;
	} catch (const std::exception& e) {
		log_error("experiment", std::string("Config parse error: ") + e.what());
		return false;
	}
	in.close();
	root[section] = section_json;
	std::ofstream out(path);
	if (!out.is_open()) {
		log_error("experiment", "Cannot write config: " + path);
		return false;
	}
	out << root.dump(1, '\t') << '\n';
	log_info("experiment", "Wrote " + std::string(section) + " → " + path);
	return true;
}

} // namespace

bool save_arena_vision_masks(const std::string& path, const ArenaMaskConfig& mask) {
	nlohmann::json vision = mask_to_vision_json(mask);
	std::ifstream in(path);
	if (!in.is_open()) {
		return false;
	}
	nlohmann::json root;
	try {
		in >> root;
	} catch (const std::exception& e) {
		log_error("experiment", std::string("Config parse error: ") + e.what());
		return false;
	}
	in.close();
	if (root.contains("vision") && root["vision"].is_object()) {
		for (auto it = vision.begin(); it != vision.end(); ++it) {
			root["vision"][it.key()] = it.value();
		}
	} else {
		root["vision"] = vision;
	}
	std::ofstream out(path);
	if (!out.is_open()) {
		return false;
	}
	out << root.dump(1, '\t') << '\n';
	log_info("experiment", "Wrote vision masks → " + path);
	return true;
}

bool save_motor_calibration(const std::string& path, const MotorConfig& motor) {
	nlohmann::json m;
	m["can_interface"] = motor.can_interface;
	m["node_id"] = motor.node_id;
	m["pulley_radius_m"] = motor.pulley_radius_m;
	m["chain_direction_sign"] = motor.chain_direction_sign;
	m["chain_mm_per_motor_turn"] = motor.chain_mm_per_motor_turn;
	return merge_write_json_section(path, "motor", m);
}
