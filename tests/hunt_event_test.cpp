// Offline unit tests for hunt-event rising edge + re-arm interval.
// Build: make test_hunt_event
// Run:   ./bin/test_hunt_event

#include <cstdlib>
#include <iostream>

#include "motor/hunt_event.h"

namespace {

int g_failures = 0;

void expect(bool ok, const char* name) {
	if (ok) {
		std::cout << "  PASS  " << name << '\n';
		return;
	}
	std::cout << "  FAIL  " << name << '\n';
	++g_failures;
}

void test_rising_edge_fires_once() {
	std::cout << "rising edge fires once while threat held\n";
	HuntArmState state;
	expect(evaluate_hunt_event(true, 1'000'000'000LL, 200, state), "first_cross");
	expect(state.threat_was_high, "threat_latched");
	expect(!evaluate_hunt_event(true, 1'050'000'000LL, 200, state), "held_no_rearm");
	expect(!evaluate_hunt_event(true, 1'100'000'000LL, 200, state),
		"held_still_no_last_event");
}

void test_dip_then_rise_again() {
	std::cout << "threat dip then rise fires again\n";
	HuntArmState state;
	expect(evaluate_hunt_event(true, 1'000'000'000LL, 200, state), "first");
	expect(!evaluate_hunt_event(false, 1'010'000'000LL, 200, state), "dip");
	expect(!state.threat_was_high, "cleared");
	expect(evaluate_hunt_event(true, 1'020'000'000LL, 200, state), "second_cross");
}

void test_sustained_rearm_after_interval() {
	std::cout << "sustained high re-arms after flee complete + interval\n";
	HuntArmState state;
	expect(evaluate_hunt_event(true, 1'000'000'000LL, 200, state), "first");
	// Flee finishes 50ms later — interval clock starts here.
	note_hunt_flee_complete(state, 1'050'000'000LL);
	expect(!evaluate_hunt_event(true, 1'100'000'000LL, 200, state),
		"before_interval");
	// 200ms after complete → 1'050ms + 200ms = 1'250ms
	expect(evaluate_hunt_event(true, 1'250'000'000LL, 200, state),
		"after_interval");
	note_hunt_flee_complete(state, 1'300'000'000LL);
	expect(!evaluate_hunt_event(true, 1'400'000'000LL, 200, state),
		"second_before_interval");
	expect(evaluate_hunt_event(true, 1'500'000'000LL, 200, state),
		"second_after_interval");
}

void test_low_threat_never_fires() {
	std::cout << "low threat never fires\n";
	HuntArmState state;
	expect(!evaluate_hunt_event(false, 1'000'000'000LL, 200, state), "low");
	note_hunt_flee_complete(state, 1'000'000'000LL);
	expect(!evaluate_hunt_event(false, 2'000'000'000LL, 200, state), "still_low");
}

} // namespace

int main() {
	std::cout << "=== Hunt event unit tests ===\n";
	test_rising_edge_fires_once();
	test_dip_then_rise_again();
	test_sustained_rearm_after_interval();
	test_low_threat_never_fires();
	std::cout << "=== " << (g_failures == 0 ? "ALL PASSED" : "FAILURES")
		<< " (" << g_failures << ") ===\n";
	return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
