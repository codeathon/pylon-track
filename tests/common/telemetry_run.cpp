#include "telemetry_run.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "log/logger.h"

using SteadyClock = std::chrono::steady_clock;

int run_telemetry(Pylon::CBaslerUniversalInstantCamera& camera,
    const TelemetryOptions& opts,
    const std::atomic<bool>& keep_running)
{
    FerretTracker tracker(/*enable_display=*/false,
        static_cast<int>(opts.warmup_s * FPS),
        opts.gsd_mm_px);

    camera.RegisterImageEventHandler(&tracker,
        Pylon::RegistrationMode_Append, Pylon::Cleanup_None);

    log_info("telemetry", "Warming up background model for "
        + std::to_string(static_cast<int>(opts.warmup_s))
        + "s — keep arena empty");

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly,
        Pylon::GrabLoop_ProvidedByInstantCamera);

    const auto session_start = SteadyClock::now();
    auto last_print_time     = SteadyClock::now();
    bool had_valid_last      = false;

    while (keep_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // get_tracking_frame() is the thread-safe snapshot; the raw
        // ferret/prey public members are written unlocked from the Pylon
        // grab thread, so reading them directly here would be a data race.
        TrackingFrame frame;
        if (!tracker.get_tracking_frame(frame)
            || !frame.ferret.state.valid || !frame.prey.state.valid) {
            had_valid_last = false;
            continue;
        }
        const TrackState& ferret_state = frame.ferret.state;
        const TrackState& prey_state = frame.prey.state;

        const auto now = SteadyClock::now();

        // t= : milliseconds since session started
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            now - session_start).count();

        // dt= : time since last printed line (gap between valid readings)
        const double dt_ms = had_valid_last
            ? std::chrono::duration<double, std::milli>(now - last_print_time).count()
            : 0.0;
        const double hz = (dt_ms > 0.0) ? 1000.0 / dt_ms : 0.0;

        last_print_time = now;
        had_valid_last  = true;

        const float dx = ferret_state.pos_mm.x - prey_state.pos_mm.x;
        const float dy = ferret_state.pos_mm.y - prey_state.pos_mm.y;
        const float distance_mm = std::sqrt(dx * dx + dy * dy);

        std::printf(
            "t=%.1fms  dt=%.1fms(%.0fHz)  "
            "Ferret: (%.0f, %.0f)mm  %.0fmm/s  %.0fdeg  |  "
            "Prey: (%.0f, %.0f)mm  %.0fmm/s  %.0fdeg  |  "
            "Dist: %.0fmm\n",
            elapsed_ms, dt_ms, hz,
            ferret_state.pos_mm.x, ferret_state.pos_mm.y,
            ferret_state.speed_mm_s, ferret_state.direction_deg,
            prey_state.pos_mm.x,   prey_state.pos_mm.y,
            prey_state.speed_mm_s,  prey_state.direction_deg,
            distance_mm);
    }

    camera.StopGrabbing();
    camera.DeregisterImageEventHandler(&tracker);
    return 0;
}
