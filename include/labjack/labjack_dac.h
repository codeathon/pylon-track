#pragma once

// LabJack T4 DAC controller — USB, LJM library.
//
// Tracks motor rotation position over a 20-rotation cycle and activates
// DAC outputs based on configurable windows:
//   DAC0: active during rotations 15–17
//   DAC1: active during rotations 18–20
//
// Typical usage:
//   LabJackDAC lj;
//   lj.open();
//   lj.update(motor_position_turns);   // call from tracking loop
//   lj.close();

class LabJackDAC {
public:
    struct Config {
        float cycle_turns  = 20.0f;  // rotations per full cycle

        float dac0_on_start  = 15.0f;
        float dac0_on_end    = 17.0f;
        float dac0_voltage   =  5.0f;

        float dac1_on_start  = 18.0f;
        float dac1_on_end    = 20.0f;
        float dac1_voltage   =  5.0f;
    };

    explicit LabJackDAC(Config cfg = {});
    ~LabJackDAC() { close(); }

    LabJackDAC(const LabJackDAC&)            = delete;
    LabJackDAC& operator=(const LabJackDAC&) = delete;

    // Connect to the first T4 found over USB. Returns false on error.
    bool open();

    // Call every tracking loop iteration with the cumulative motor position
    // in turns. Activates / deactivates DAC0 and DAC1 based on the position
    // within the current 20-turn cycle.
    void update(float motor_position_turns);

    // Set both DACs to 0 V and close the device.
    void close();

    bool is_open() const { return handle_ >= 0; }

    // Last known state of each channel (true = active voltage).
    bool dac0_active() const { return dac0_active_; }
    bool dac1_active() const { return dac1_active_; }

private:
    void write_dac(const char* name, float voltage);

    Config cfg_;
    int    handle_      = -1;
    bool   dac0_active_ = true;
    bool   dac1_active_ = true;
};
