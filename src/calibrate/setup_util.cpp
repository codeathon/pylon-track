#include "calibrate/setup_util.h"
#include "calibrate/setup_options.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace fs = std::filesystem;

SetupStep parse_setup_step(const std::string& name) {
	if (name == "arena") return SetupStep::Arena;
	if (name == "camera") return SetupStep::Camera;
	if (name == "odrive") return SetupStep::ODrive;
	if (name == "labjack") return SetupStep::LabJack;
	return SetupStep::All;
}

std::string resolve_arena_config_path(const char* argv0, const std::string& user_path) {
	if (!user_path.empty()) {
		return user_path;
	}
	const fs::path exe_dir = fs::path(argv0 ? argv0 : "").parent_path();
	const fs::path candidates[] = {
		exe_dir / "arena_experiment.json",
		exe_dir / "config" / "arena_experiment.json",
		fs::path("config") / "arena_experiment.json",
	};
	for (const auto& path : candidates) {
		if (fs::exists(path)) {
			return path.string();
		}
	}
	return {};
}

std::string resolve_calib_output_path(const char* argv0, const std::string& user_path) {
	if (!user_path.empty()) {
		return user_path;
	}
	const fs::path exe_dir = fs::path(argv0 ? argv0 : "").parent_path();
	const fs::path beside_exe = exe_dir / "calib.npz";
	if (fs::exists(beside_exe.parent_path()) || exe_dir.empty()) {
		return beside_exe.string();
	}
	return "calib.npz";
}

bool can_interface_up(const std::string& iface) {
	const fs::path operstate = fs::path("/sys/class/net") / iface / "operstate";
	std::ifstream file(operstate);
	if (!file.is_open()) {
		return false;
	}
	std::string state;
	file >> state;
	return state == "up";
}

void print_setup_banner(const char* step_name, int step_num, int step_total) {
	std::cout << "\n=== Arena setup (step " << step_num << "/" << step_total
		<< "): " << step_name << " ===\n";
}

bool prompt_enter(const char* message) {
	std::cout << message;
	std::string line;
	return static_cast<bool>(std::getline(std::cin, line));
}

std::optional<bool> prompt_yes_no(const char* message) {
	while (true) {
		std::cout << message;
		std::string raw;
		if (!std::getline(std::cin, raw)) {
			return std::nullopt;
		}
		if (raw == "y" || raw == "Y" || raw == "yes") {
			return true;
		}
		if (raw == "n" || raw == "N" || raw == "no") {
			return false;
		}
		std::cout << "Answer y or n.\n";
	}
}

std::optional<float> prompt_float(const char* message) {
	while (true) {
		std::cout << message;
		std::string raw;
		if (!std::getline(std::cin, raw)) {
			return std::nullopt;
		}
		try {
			return std::stof(raw);
		} catch (...) {
			std::cout << "Enter a number.\n";
		}
	}
}

bool interruptible_sleep_ms(const SetupOptions& opts, int total_ms) {
	constexpr int kStepMs = 50;
	int elapsed = 0;
	while (elapsed < total_ms) {
		if (opts.running && !opts.running->load()) {
			return false;
		}
		const int chunk = std::min(kStepMs, total_ms - elapsed);
		std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
		elapsed += chunk;
	}
	return !opts.running || opts.running->load();
}
