#pragma once

#include <opencv2/opencv.hpp>

struct TrackState {
    cv::Point2f pos_px;      // position in pixels
    cv::Point2f pos_mm;      // position in mm (arena coords, origin = top-left)
    float speed_mm_s = 0.0f; // scalar speed
    float direction_deg = 0.0f; // 0=right, 90=up (CCW), image Y-axis flipped
    bool valid = false;
};

cv::KalmanFilter make_kalman(float fps);
