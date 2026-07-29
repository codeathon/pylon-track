#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include "experiment/arena_config.h"
#include "tracker/tracker.h"

struct AssociationResult {
	int ferret_idx = -1;
	int prey_idx = -1;
	float ferret_confidence = 0.0f;
	float prey_confidence = 0.0f;
	const char* reject_reason = nullptr;
};

// Scores MOG2 contours against animal priors and Kalman predictions so chains
// and pulleys are rejected instead of stealing ferret/prey track slots.
class ObjectAssociator {
public:
	explicit ObjectAssociator(const VisionConfig& vision);

	AssociationResult associate(
		const std::vector<std::vector<cv::Point>>& contours,
		const TrackState& ferret_prior,
		const TrackState& prey_prior) const;

private:
	VisionConfig vision_;

	struct Candidate {
		int index = -1;
		float area = 0.0f;
		float compactness = 0.0f;
		cv::Point2f centroid;
		bool ferret_area_ok = false;
		bool prey_area_ok = false;
	};

	static float contour_compactness(const std::vector<cv::Point>& contour);
	static cv::Point2f contour_centroid(const std::vector<cv::Point>& contour);
	static float proximity_score(cv::Point2f centroid, const TrackState& prior);

	std::vector<Candidate> build_candidates(
		const std::vector<std::vector<cv::Point>>& contours) const;
};
