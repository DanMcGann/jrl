#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Geometry>
#include <vector>

namespace jrl {

namespace alignment {

namespace internal {
/** TRANSLATION EXTRACTION
 * A method for extracting the translation from gtsam objects (POSES/POINTS/VECTORS)
 */
template <class T>
inline gtsam::Vector translation(T obj);
template <>
inline gtsam::Vector translation<gtsam::Pose3>(gtsam::Pose3 obj);
template <>
inline gtsam::Vector translation<gtsam::Pose2>(gtsam::Pose2 obj);

/** TRANSLATION DIMENSION
 * A method for extracting the dimension of the translation component from gtsam objects (POSES/POINTS/VECTORS)
 */
template <class T>
inline size_t translationDimension(T obj);
template <>
inline size_t translationDimension<gtsam::Pose3>(gtsam::Pose3 obj);
template <>
inline size_t translationDimension<gtsam::Pose2>(gtsam::Pose2 obj);

/** Transform
 * A method for transforming a pose by its corresponding Umeyama transform
 */
template <class T>
inline T transform(gtsam::Matrix transform, T obj);
template <>
inline gtsam::Pose2 transform<gtsam::Pose2>(gtsam::Matrix transform, gtsam::Pose2 obj);
template <>
inline gtsam::Pose3 transform<gtsam::Pose3>(gtsam::Matrix transform, gtsam::Pose3 obj);

}  // namespace internal

/** @brief Aligns two HOMOGENOUS trajectories with Umeyama alignment
 * Note: HOMOGENOUS means all objects in the trajectory must be of same type e.x. Pose2 or Point3
 * @param estimate_trajectory: The estimated trajectory from some method
 * @param reference_trajectory: The reference trajectory preferably groundtruth
 * @param align_with_scale: Flag indicating to preform Umeyama with scale the estimate
 */
template <class POSE_TYPE>
inline gtsam::Values align(const gtsam::Values& estimate_trajectory, const gtsam::Values& reference_trajectory,
                           bool align_with_scale = false);

}  // namespace alignment
}  // namespace jrl

#include "jrl/Alignment-inl.h"
