#include "vision/tracking_frame.h"

#include <cmath>

bool TrackingFrame::both_valid() const {
	return ferret.state.valid && prey.state.valid;
}

void fill_tracking_derived(TrackingFrame& frame, float fps) {
	frame.distance_mm = -1.0f;
	frame.bearing_deg = 0.0f;
	frame.closing_speed_mm_s = 0.0f;
	if (!frame.both_valid()) {
		return;
	}

	const float dx = frame.prey.state.pos_mm.x - frame.ferret.state.pos_mm.x;
	const float dy = frame.prey.state.pos_mm.y - frame.ferret.state.pos_mm.y;
	frame.distance_mm = std::sqrt(dx * dx + dy * dy);
	frame.bearing_deg = std::atan2(-dy, dx) * 180.0f / static_cast<float>(M_PI);

	const float ferret_heading_rad = frame.ferret.state.direction_deg
		* static_cast<float>(M_PI) / 180.0f;
	const float vx = std::cos(ferret_heading_rad) * frame.ferret.state.speed_mm_s;
	const float vy = -std::sin(ferret_heading_rad) * frame.ferret.state.speed_mm_s;
	const float dist = frame.distance_mm;
	if (dist > 1e-3f) {
		frame.closing_speed_mm_s = (vx * dx + vy * dy) / dist;
	}
	(void)fps;
}
