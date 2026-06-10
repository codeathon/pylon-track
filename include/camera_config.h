#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include "camera_settings.h"

using namespace Basler_UniversalCameraParams;
using namespace Pylon;

void configure_camera(CBaslerUniversalInstantCamera& cam, const CameraSettings& settings);
