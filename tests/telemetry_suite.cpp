#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "log/logger.h"
#include "telemetry_run.h"

using namespace Pylon;

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running.store(false); }

struct Options {
    bool    verbose    = false;
    std::string log_file;
    std::string camera_config;
    double  warmup_s   = 30.0;
    float   gsd_mm_px  = GSD_MM_PX;
};

static bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--verbose") == 0) {
            o.verbose = true;
        } else if (std::strcmp(argv[i], "--log-file") == 0) {
            if (i + 1 >= argc) { std::cerr << "ERROR: --log-file needs a path\n"; return false; }
            o.log_file = argv[++i];
        } else if (std::strcmp(argv[i], "--camera-config") == 0) {
            if (i + 1 >= argc) { std::cerr << "ERROR: --camera-config needs a path\n"; return false; }
            o.camera_config = argv[++i];
        } else if (std::strcmp(argv[i], "--warmup") == 0) {
            if (i + 1 >= argc) { std::cerr << "ERROR: --warmup needs seconds\n"; return false; }
            o.warmup_s = std::stod(argv[++i]);
        } else if (std::strcmp(argv[i], "--gsd") == 0) {
            if (i + 1 >= argc) { std::cerr << "ERROR: --gsd needs mm/px value\n"; return false; }
            o.gsd_mm_px = std::stof(argv[++i]);
        } else {
            std::cerr << "ERROR: Unknown argument: " << argv[i] << '\n';
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    Options opts;
    if (!parse_args(argc, argv, opts)) return 1;

    Logger& logger = Logger::instance();
    logger.set_level(opts.verbose ? LogLevel::Debug : LogLevel::Info);
    if (!opts.log_file.empty()) {
        logger.set_log_file(opts.log_file);
        log_info("telemetry", "Logging to file: " + opts.log_file);
    }

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    PylonInitialize();

    try {
        CBaslerUniversalInstantCamera camera(
            CTlFactory::GetInstance().CreateFirstDevice());

        log_info("telemetry", "Camera: "
            + std::string(camera.GetDeviceInfo().GetModelName()));

        const std::string config_path =
            resolve_camera_config_path(argv[0], opts.camera_config);
        log_info("camera", "Loading camera config: " + config_path);

        CameraSettings camera_settings;
        if (!load_camera_config(config_path, camera_settings)) {
            PylonTerminate();
            return 1;
        }
        try {
            configure_camera(camera, camera_settings);
        } catch (const std::exception& e) {
            log_error("camera", std::string("Camera configuration failed: ") + e.what());
            PylonTerminate();
            return 1;
        }

        TelemetryOptions tel_opts;
        tel_opts.warmup_s  = opts.warmup_s;
        tel_opts.gsd_mm_px = opts.gsd_mm_px;

        const int rc = run_telemetry(camera, tel_opts, g_running);
        log_info("telemetry", "Done");
        PylonTerminate();
        return rc;

    } catch (const GenericException& e) {
        log_error("telemetry", std::string("Pylon error: ") + e.GetDescription());
        PylonTerminate();
        return 1;
    }
}
