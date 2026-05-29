#include "camera_config.h"

void configure_camera(CBaslerUniversalInstantCamera& cam) {
    cam.Open();

    // Mono8 — no debayer, fastest path
    cam.PixelFormat.SetValue(PixelFormat_Mono8);

    // AOI: center-crop height to 960px → covers 1.99m × 0.99m at 1.2m mount height
    // Boosts frame rate from 160fps → ~200fps
    cam.Width.SetValue(1920);
    cam.Height.SetValue(960);
    cam.OffsetX.SetValue(0);
    cam.OffsetY.SetValue(120);  // (1200 - 960) / 2

    // Exposure: 2ms → 2.9px motion blur at ferret sprint speed (1.5m/s)
    cam.ExposureAuto.SetValue(ExposureAuto_Off);
    cam.ExposureTime.SetValue(2000.0);  // µs

    // Gain: start at 6dB, tune until background is ~80-100 DN under IR lighting
    cam.GainAuto.SetValue(GainAuto_Off);
    cam.Gain.SetValue(6.0);  // dB

    // Frame rate: unlock and push to hardware max (~200fps at 1920x960)
    cam.AcquisitionFrameRateEnable.SetValue(true);
    cam.AcquisitionFrameRate.SetValue(200.0);

    // Free-run continuous acquisition, no trigger
    cam.TriggerMode.SetValue(TriggerMode_Off);

    // Saturate USB3 bandwidth — minimizes transfer latency
    cam.DeviceLinkThroughputLimitMode.SetValue(
        DeviceLinkThroughputLimitMode_Off);
}
