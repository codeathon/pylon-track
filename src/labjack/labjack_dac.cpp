#include "labjack/labjack_dac.h"
#include "log/logger.h"

#include <cmath>
#include <string>

// LabJack LJM C library
#include <LabJackM.h>

// ── helpers ──────────────────────────────────────────────────────────────────

static std::string ljm_error_str(int err) {
    char buf[LJM_MAX_NAME_SIZE] = {};
    LJM_ErrorToString(err, buf);
    return buf;
}

// ── LabJackDAC ───────────────────────────────────────────────────────────────

LabJackDAC::LabJackDAC(Config cfg) : cfg_(cfg) {}

bool LabJackDAC::open() {
    int handle = 0;
    const int err = LJM_Open(LJM_dtT4, LJM_ctUSB, "LJM_idANY", &handle);
    if (err != LJME_NOERROR) {
        LOG_INFO("LabJack: open failed — " + ljm_error_str(err));
        return false;
    }
    handle_ = handle;

    // Start with both DACs on
    write_dac("DAC0", 5.0f);
    write_dac("DAC1", 5.0f);
    dac0_active_ = true;
    dac1_active_ = true;

    LOG_INFO("LabJack: T4 connected (USB)");
    return true;
}

void LabJackDAC::update(float motor_position_turns) {
    if (handle_ < 0) return;

    // Wrap position into [0, cycle_turns)
    const float cycle = cfg_.cycle_turns;
    float pos = std::fmod(motor_position_turns, cycle);
    if (pos < 0.0f) pos += cycle;

    // DAC0 window
    const bool want_dac0 = (pos >= cfg_.dac0_on_start && pos <= cfg_.dac0_on_end);
    if (want_dac0 != dac0_active_) {
        write_dac("DAC0", want_dac0 ? cfg_.dac0_voltage : 5.0f);
        dac0_active_ = want_dac0;
        LOG_INFO(std::string("LabJack: DAC0 ") + (want_dac0 ? "OFF" : "ON") +
                 " at pos=" + std::to_string(pos) + " turns");
    }

    // DAC1 window
    const bool want_dac1 = (pos >= cfg_.dac1_on_start && pos <= cfg_.dac1_on_end);
    if (want_dac1 != dac1_active_) {
        write_dac("DAC1", want_dac1 ? cfg_.dac1_voltage : 5.0f);
        dac1_active_ = want_dac1;
        LOG_INFO(std::string("LabJack: DAC1 ") + (want_dac1 ? "OFF" : "ON") +
                 " at pos=" + std::to_string(pos) + " turns");
    }
}

void LabJackDAC::close() {
    if (handle_ < 0) return;

    write_dac("DAC0", 5.0f);
    write_dac("DAC1", 5.0f);
    dac0_active_ = true;
    dac1_active_ = true;

    LJM_Close(handle_);
    handle_ = -1;
    LOG_INFO("LabJack: closed");
}

// ── internal ──────────────────────────────────────────────────────────────────

void LabJackDAC::write_dac(const char* name, float voltage) {
    const int err = LJM_eWriteName(handle_, name, static_cast<double>(voltage));
    if (err != LJME_NOERROR) {
        LOG_INFO(std::string("LabJack: write ") + name +
                 " failed — " + ljm_error_str(err));
    }
}
