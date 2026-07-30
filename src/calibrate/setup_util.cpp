#include "calibrate/setup_util.h"
#include "calibrate/setup_options.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fs = std::filesystem;

SetupStep parse_setup_step(const std::string& name) {
	if (name == "arena") return SetupStep::Arena;
	if (name == "camera") return SetupStep::Camera;
	if (name == "odrive") return SetupStep::ODrive;
	if (name == "labjack") return SetupStep::LabJack;
	return SetupStep::All;
}

std::string resolve_arena_config_path(const char* argv0, const std::string& user_path) {
	auto canonicalize = [](const fs::path& p) -> std::string {
		std::error_code ec;
		const fs::path abs = fs::weakly_canonical(fs::absolute(p), ec);
		return ec ? p.string() : abs.string();
	};
	if (!user_path.empty()) {
		// Always resolve --config to an absolute path so edits match the file loaded.
		return canonicalize(user_path);
	}
	const fs::path exe_dir = fs::path(argv0 ? argv0 : "").parent_path();
	const fs::path candidates[] = {
		exe_dir / "arena_experiment.json",
		exe_dir / "config" / "arena_experiment.json",
		fs::path("config") / "arena_experiment.json",
	};
	for (const auto& path : candidates) {
		if (fs::exists(path)) {
			return canonicalize(path);
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
	// Match `ip addr show` administrative UP (IFF_UP).
	// Do not use operstate — SocketCAN often reports "unknown" while UP.
	if (iface.empty() || iface.size() >= IFNAMSIZ) {
		return false;
	}
	const int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		return false;
	}
	struct ifreq req;
	std::memset(&req, 0, sizeof(req));
	std::snprintf(req.ifr_name, IFNAMSIZ, "%s", iface.c_str());
	const int rc = ioctl(fd, SIOCGIFFLAGS, &req);
	close(fd);
	if (rc < 0) {
		return false;
	}
	return (req.ifr_flags & IFF_UP) != 0;
}

std::string can_interface_down_hint(const std::string& iface) {
	return "CAN interface " + iface
		+ " is not UP (check: ip addr show " + iface + "). "
		+ "Plug in USB-CAN adapter and re-run make (loads gs_usb + brings iface up).";
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

bool prompt_yes_no(const char* message) {
	while (true) {
		std::cout << message;
		std::string raw;
		if (!std::getline(std::cin, raw)) {
			return false;
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

float prompt_float(const char* message) {
	while (true) {
		std::cout << message;
		std::string raw;
		if (!std::getline(std::cin, raw)) {
			return 0.0f;
		}
		try {
			return std::stof(raw);
		} catch (...) {
			std::cout << "Enter a number.\n";
		}
	}
}
