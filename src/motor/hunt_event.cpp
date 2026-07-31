#include "motor/hunt_event.h"

#include <algorithm>

bool evaluate_hunt_event(bool threat_high, int64_t now_ns, int min_interval_ms,
	HuntArmState& state)
{
	const int64_t interval_ns = static_cast<int64_t>(std::max(0, min_interval_ms))
		* 1000000LL;
	const bool rising = threat_high && !state.threat_was_high;
	// Sustained pressure re-arms only after a prior event timestamp + interval.
	const bool interval_elapsed = (state.last_event_ns > 0)
		&& (now_ns - state.last_event_ns >= interval_ns);
	const bool fire = rising || (threat_high && interval_elapsed);
	state.threat_was_high = threat_high;
	return fire;
}

void note_hunt_flee_complete(HuntArmState& state, int64_t now_ns) {
	state.last_event_ns = now_ns;
}
