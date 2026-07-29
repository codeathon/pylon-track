#include "vision/object_associator.h"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace {

constexpr float kPi = 3.14159265f;

bool area_in_range(float area, float min_a, float max_a) {
	return area >= min_a && area <= max_a;
}

float area_fit_score(float area, float min_a, float max_a) {
	if (!area_in_range(area, min_a, max_a)) {
		return 0.0f;
	}
	const float mid = 0.5f * (min_a + max_a);
	const float half_span = 0.5f * (max_a - min_a);
	if (half_span < 1e-3f) {
		return 1.0f;
	}
	const float norm = std::fabs(area - mid) / half_span;
	return std::max(0.0f, 1.0f - norm);
}

} // namespace

ObjectAssociator::ObjectAssociator(const VisionConfig& vision) : vision_(vision) {}

float ObjectAssociator::contour_compactness(const std::vector<cv::Point>& contour) {
	const float area = static_cast<float>(cv::contourArea(contour));
	const float perim = static_cast<float>(cv::arcLength(contour, true));
	if (perim < 1e-3f || area <= 0.0f) {
		return 0.0f;
	}
	return (4.0f * kPi * area) / (perim * perim);
}

cv::Point2f ObjectAssociator::contour_centroid(const std::vector<cv::Point>& contour) {
	const cv::Moments m = cv::moments(contour);
	return {static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00)};
}

float ObjectAssociator::proximity_score(cv::Point2f centroid, const TrackState& prior) {
	if (!prior.valid) {
		return 0.5f;
	}
	const float dx = centroid.x - prior.pos_px.x;
	const float dy = centroid.y - prior.pos_px.y;
	const float dist = std::sqrt(dx * dx + dy * dy);
	return std::exp(-dist / 120.0f);
}

std::vector<ObjectAssociator::Candidate> ObjectAssociator::build_candidates(
	const std::vector<std::vector<cv::Point>>& contours) const
{
	std::vector<Candidate> out;
	out.reserve(contours.size());
	for (size_t i = 0; i < contours.size(); ++i) {
		const float area = static_cast<float>(cv::contourArea(contours[i]));
		const float compactness = contour_compactness(contours[i]);
		if (compactness < vision_.max_compactness) {
			continue;
		}
		Candidate c;
		c.index = static_cast<int>(i);
		c.area = area;
		c.compactness = compactness;
		c.centroid = contour_centroid(contours[i]);
		c.ferret_area_ok = area_in_range(area, vision_.ferret_area_px_min,
			vision_.ferret_area_px_max);
		c.prey_area_ok = area_in_range(area, vision_.prey_area_px_min,
			vision_.prey_area_px_max);
		out.push_back(c);
	}
	return out;
}

AssociationResult ObjectAssociator::associate(
	const std::vector<std::vector<cv::Point>>& contours,
	const TrackState& ferret_prior,
	const TrackState& prey_prior) const
{
	AssociationResult result;
	const std::vector<Candidate> candidates = build_candidates(contours);
	if (candidates.empty()) {
		result.reject_reason = "no_candidates";
		return result;
	}

	float best_ferret_score = -1.0f;
	for (const auto& c : candidates) {
		if (!c.ferret_area_ok) {
			continue;
		}
		const float score = 0.6f * area_fit_score(c.area, vision_.ferret_area_px_min,
				vision_.ferret_area_px_max)
			+ 0.4f * proximity_score(c.centroid, ferret_prior);
		if (score > best_ferret_score) {
			best_ferret_score = score;
			result.ferret_idx = c.index;
			result.ferret_confidence = score;
		}
	}

	float best_prey_score = -1.0f;
	for (const auto& c : candidates) {
		if (c.index == result.ferret_idx || !c.prey_area_ok) {
			continue;
		}
		const float score = 0.6f * area_fit_score(c.area, vision_.prey_area_px_min,
				vision_.prey_area_px_max)
			+ 0.4f * proximity_score(c.centroid, prey_prior);
		if (score > best_prey_score) {
			best_prey_score = score;
			result.prey_idx = c.index;
			result.prey_confidence = score;
		}
	}

	if (result.ferret_idx < 0 && result.prey_idx < 0) {
		result.reject_reason = "no_animal_match";
	}
	return result;
}
