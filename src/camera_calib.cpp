#include "camera_calib.h"
#include "logger.h"

#include <cnpy.h>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <cstdlib>

namespace fs = std::filesystem;

namespace {

bool file_exists(const std::string& path) {
	return !path.empty() && fs::is_regular_file(path);
}

std::string exe_directory(const char* argv0) {
	if (argv0 == nullptr || argv0[0] == '\0') {
		return {};
	}
	std::error_code ec;
	const fs::path exe = fs::weakly_canonical(fs::path(argv0), ec);
	if (ec) {
		return {};
	}
	return exe.parent_path().string();
}

cv::Mat npy_to_mat64(const cnpy::NpyArray& arr) {
	if (arr.word_size != sizeof(double)) {
		throw std::runtime_error("expected float64 numpy array");
	}
	int rows = 1;
	int cols = 1;
	if (arr.shape.size() == 1) {
		cols = static_cast<int>(arr.shape[0]);
	} else if (arr.shape.size() == 2) {
		rows = static_cast<int>(arr.shape[0]);
		cols = static_cast<int>(arr.shape[1]);
	} else {
		throw std::runtime_error("unsupported numpy array rank");
	}
	cv::Mat m(rows, cols, CV_64F);
	std::memcpy(m.data, arr.data<double>(), arr.num_vals * sizeof(double));
	return m;
}

cv::Size read_img_size(const cnpy::NpyArray& arr) {
	if (arr.shape.size() != 1 || arr.shape[0] != 2) {
		throw std::runtime_error("img_size must be a length-2 vector");
	}
	auto read_index = [&](size_t i) -> int {
		if (arr.word_size == sizeof(int64_t)) {
			return static_cast<int>(arr.data<int64_t>()[i]);
		}
		if (arr.word_size == sizeof(int32_t)) {
			return static_cast<int>(arr.data<int32_t>()[i]);
		}
		if (arr.word_size == sizeof(double)) {
			return static_cast<int>(arr.data<double>()[i]);
		}
		throw std::runtime_error("unsupported img_size dtype");
	};
	return cv::Size(read_index(0), read_index(1));
}

double read_scalar_double(const cnpy::NpyArray& arr) {
	if (arr.num_vals != 1) {
		throw std::runtime_error("expected scalar array");
	}
	if (arr.word_size == sizeof(double)) {
		return arr.data<double>()[0];
	}
	if (arr.word_size == sizeof(float)) {
		return static_cast<double>(arr.data<float>()[0]);
	}
	if (arr.word_size == sizeof(int64_t)) {
		return static_cast<double>(arr.data<int64_t>()[0]);
	}
	if (arr.word_size == sizeof(int32_t)) {
		return static_cast<double>(arr.data<int32_t>()[0]);
	}
	throw std::runtime_error("unsupported scalar dtype");
}

void build_standard_maps(CameraCalib& calib, cv::Size frame_size) {
	const cv::Mat new_k = cv::getOptimalNewCameraMatrix(
		calib.camera_matrix, calib.dist_coeffs, frame_size,
		calib.undistort_alpha, frame_size);
	cv::initUndistortRectifyMap(
		calib.camera_matrix, calib.dist_coeffs, cv::Mat(), new_k,
		frame_size, CV_16SC2, calib.map1, calib.map2);
}

void build_fisheye_maps(CameraCalib& calib, cv::Size frame_size) {
	cv::Mat new_k;
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
		calib.camera_matrix, calib.dist_coeffs, frame_size,
		cv::Matx33d::eye(), new_k, calib.undistort_alpha, frame_size, 1.0);
	cv::fisheye::initUndistortRectifyMap(
		calib.camera_matrix, calib.dist_coeffs, cv::Matx33d::eye(), new_k,
		frame_size, CV_16SC2, calib.map1, calib.map2);
}

void build_undistort_maps(CameraCalib& calib, cv::Size frame_size) {
	if (calib.model == CalibModel::Fisheye) {
		build_fisheye_maps(calib, frame_size);
	} else {
		build_standard_maps(calib, frame_size);
	}
	calib.image_size = frame_size;
}

} // namespace

std::string resolve_calib_path(const char* argv0, const std::string& cli_path) {
	if (!cli_path.empty()) {
		return cli_path;
	}
	const char* env_path = std::getenv("PYLON_CAMERA_CALIB");
	if (env_path != nullptr && env_path[0] != '\0') {
		return env_path;
	}
	const std::string beside_exe = exe_directory(argv0) + "/calib.npz";
	if (file_exists(beside_exe)) {
		return beside_exe;
	}
	if (file_exists("calib.npz")) {
		return "calib.npz";
	}
	return {};
}

std::optional<CameraCalib> load_camera_calib(const std::string& path,
	cv::Size frame_size)
{
	try {
		const cnpy::npz_t npz = cnpy::npz_load(path);
		if (npz.find("K") == npz.end() || npz.find("dist") == npz.end()) {
			log_error("calib", path + " missing required arrays K and/or dist");
			return std::nullopt;
		}

		CameraCalib calib;
		if (npz.find("model_id") != npz.end()) {
			const int model = static_cast<int>(read_scalar_double(npz.at("model_id")));
			calib.model = model == 1 ? CalibModel::Fisheye : CalibModel::Standard;
		}
		if (npz.find("undistort_alpha") != npz.end()) {
			calib.undistort_alpha = read_scalar_double(npz.at("undistort_alpha"));
		}

		calib.camera_matrix = npy_to_mat64(npz.at("K"));
		calib.dist_coeffs = npy_to_mat64(npz.at("dist"));
		if (calib.camera_matrix.rows != 3 || calib.camera_matrix.cols != 3) {
			log_error("calib", "K must be 3×3");
			return std::nullopt;
		}
		if (calib.dist_coeffs.rows != 1 && calib.dist_coeffs.cols != 1) {
			log_error("calib", "dist must be a vector");
			return std::nullopt;
		}

		if (npz.find("img_size") != npz.end()) {
			const cv::Size npz_size = read_img_size(npz.at("img_size"));
			if (npz_size.width != frame_size.width
				|| npz_size.height != frame_size.height) {
				std::ostringstream oss;
				oss << "calib.npz img_size " << npz_size.width << "x"
					<< npz_size.height << " != camera " << frame_size.width
					<< "x" << frame_size.height
					<< " — recapture at this AOI or update camera_config.json";
				log_error("calib", oss.str());
				return std::nullopt;
			}
		}

		if (npz.find("rms") != npz.end()) {
			calib.rms = read_scalar_double(npz.at("rms"));
		}

		build_undistort_maps(calib, frame_size);

		const char* model_name = calib.model == CalibModel::Fisheye ? "fisheye" : "standard";
		std::ostringstream oss;
		oss << "Loaded " << path << " (" << model_name << ", rms=" << calib.rms
			<< " px, alpha=" << calib.undistort_alpha << ", "
			<< frame_size.width << "x" << frame_size.height << ")";
		log_info("calib", oss.str());
		return calib;
	} catch (const std::exception& e) {
		log_error("calib", std::string("Failed to load ") + path + ": " + e.what());
		return std::nullopt;
	}
}
