#include "tracker.h"

cv::KalmanFilter make_kalman(float fps) {
    cv::KalmanFilter kf(4, 2);
    float dt = 1.0f / fps;

    // State: [x, y, vx, vy]
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1,  0,
        0, 0, 0,  1);

    cv::setIdentity(kf.measurementMatrix);                          // H = I(2x4) top rows
    cv::setIdentity(kf.processNoiseCov,     cv::Scalar::all(1e-4)); // process smoothness
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(0.1));  // detection noise
    cv::setIdentity(kf.errorCovPost,        cv::Scalar::all(1.0));
    return kf;
}
