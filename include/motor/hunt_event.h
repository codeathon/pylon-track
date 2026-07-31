#pragma once

#include <cstdint>

// Edge / re-arm state for discrete hunt events (owned by ChaseController).
struct HuntArmState {
	bool threat_was_high = false;
	// Set when a flee completes (or a failed arm) — gates sustained re-arm.
	int64_t last_event_ns = 0;
};

// Rising edge of threat_high, or sustained high after min_interval from last_event_ns.
// Pure function for unit tests — updates state.threat_was_high every call.
bool evaluate_hunt_event(bool threat_high, int64_t now_ns, int min_interval_ms,
	HuntArmState& state);

// Call when a flee finishes (Done) so the re-arm interval starts from completion.
void note_hunt_flee_complete(HuntArmState& state, int64_t now_ns);
