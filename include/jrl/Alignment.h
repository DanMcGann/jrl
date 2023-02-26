#pragma once
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Geometry>
#include <vector>

#include "jrl/Dataset.h"
#include "jrl/Results.h"

namespace jrl {

namespace alignment {

namespace internal {
/** TRANSLATION EXTRACTION
 * A method for extracting the translation from gtsam objects (POSES/POINTS/VECTORS)
 */

// For any linear object, the translation is itself
template <class T>
gtsam::Vector translation(T obj) {
  return obj;
}

// Pose Specializations
template <>
gtsam::Vector translation<gtsam::Pose3>(gtsam::Pose3 obj) {
  return obj.translation();
}
template <>
gtsam::Vector translation<gtsam::Pose2>(gtsam::Pose2 obj) {
  return obj.translation();
}

/** TRANSLATION DIMENSION
 * A method for extracting the dimension of the translation component from gtsam objects (POSES/POINTS/VECTORS)
 */
template <class T>
size_t translationDimension(T obj) {
  return obj.size();
}

// Pose Specializations
template <>
size_t translationDimension<gtsam::Pose3>(gtsam::Pose3 obj) {
  return 3;
}
template <>
size_t translationDimension<gtsam::Pose2>(gtsam::Pose2 obj) {
  return 2;
}

/** Transform
 * A method for transforming a pose by its corresponding Umeyama transform
 */
template <class T>  // for vectors and such just multiply
T transform(gtsam::Matrix transform, T obj) {
  return transform * obj;
}

// Pose Specializations
template <>
gtsam::Pose2 transform<gtsam::Pose2>(gtsam::Matrix transform, gtsam::Pose2 obj) {
  return gtsam::Pose2(transform).compose(obj);
}
template <>
gtsam::Pose3 transform<gtsam::Pose3>(gtsam::Matrix transform, gtsam::Pose3 obj) {
  return gtsam::Pose3(transform).compose(obj);
}

}  // namespace internal

/**
 * Aligns two HOMOGENOUS trajectories (all objects must be of same type)
 */
template <class POSE_TYPE>
gtsam::Values align(const gtsam::Values& estimate_trajectory, const gtsam::Values& reference_trajectory,
                    bool with_scale = false) {
  // High level information
  gtsam::KeyVector keys = reference_trajectory.keys();
  POSE_TYPE sample_obj = reference_trajectory.at<POSE_TYPE>(keys.front());
  size_t d = internal::translationDimension<POSE_TYPE>(sample_obj);
  size_t nposes = reference_trajectory.size();

  // Construct d dimensional matrices of the translational components of all trajectory poses
  gtsam::Matrix ref_mat(nposes, d);
  gtsam::Matrix est_mat(nposes, d);
  for (size_t k = 0; k < nposes; k++) {
    ref_mat.row(k) = internal::translation<POSE_TYPE>(reference_trajectory.at<POSE_TYPE>(keys[k]));
    est_mat.row(k) = internal::translation<POSE_TYPE>(estimate_trajectory.at<POSE_TYPE>(keys[k]));
  }

  // Use Eigen to compute the Uymeuama transform
  gtsam::Matrix transform = Eigen::umeyama(est_mat.transpose(), ref_mat.transpose(), with_scale);  // (Src, Dst, With Scaling)

  // Transform all trajectory poses by the computed alignment transform
  gtsam::Values aligned_trajectory;
  for (size_t k = 0; k < nposes; k++) {
    aligned_trajectory.insert(keys[k], internal::transform(transform , estimate_trajectory.at<POSE_TYPE>(keys[k])));
  }

  return aligned_trajectory;
}

}  // namespace alignment
}  // namespace jrl
