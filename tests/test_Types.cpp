#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>
#include <jrl/Dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/IOMeasurements.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

#define EXPECT_MATRICES_EQ(M_actual, M_expected) \
  EXPECT_TRUE(M_actual.isApprox(M_expected, 1e-6)) << "  Actual:\n" << M_actual << "\nExpected:\n" << M_expected

TEST(Types, EntryRemove) {
  // Make dummy factors
  gtsam::noiseModel::Gaussian::shared_ptr cov = gtsam::noiseModel::Gaussian::Covariance(Eigen::Matrix3d::Identity());
  gtsam::PriorFactor<gtsam::Pose2> factor_prior(X(0), gtsam::Pose2::Identity(), cov);
  gtsam::BetweenFactor<gtsam::Pose2> factor_between(X(0), X(1), gtsam::Pose2::Identity(), cov);

  // Make dataset
  gtsam::NonlinearFactorGraph graph;
  graph.push_back(factor_prior);
  graph.push_back(factor_between);

  std::vector<std::string> tags{jrl::PriorFactorPose2Tag, jrl::BetweenFactorPose2Tag};
  std::map<gtsam::FactorIndex, bool> potential_outlier_statuses{{0, false}, {1, true}};
  jrl::Entry entry = jrl::Entry(0, tags, graph, potential_outlier_statuses);

  auto pred = jrl::Entry::RemoveTypes({jrl::BetweenFactorPose2Tag});
  jrl::Entry split = entry.filtered(pred);

  EXPECT_EQ(1, split.measurements.size());
  EXPECT_EQ(1, split.measurement_types.size());
  EXPECT_EQ(jrl::PriorFactorPose2Tag, split.measurement_types[0]);
  EXPECT_TRUE(factor_prior.equals(*split.measurements[0]));
  EXPECT_FALSE(split.potential_outlier_statuses[0]);
}

TEST(Types, EntryKeep) {
  // Make dummy factors
  gtsam::noiseModel::Gaussian::shared_ptr cov = gtsam::noiseModel::Gaussian::Covariance(Eigen::Matrix3d::Identity());
  gtsam::PriorFactor<gtsam::Pose2> factor_prior(X(0), gtsam::Pose2::Identity(), cov);
  gtsam::BetweenFactor<gtsam::Pose2> factor_between(X(0), X(1), gtsam::Pose2::Identity(), cov);

  // Make dataset
  gtsam::NonlinearFactorGraph graph;
  graph.push_back(factor_prior);
  graph.push_back(factor_between);

  std::vector<std::string> tags{jrl::PriorFactorPose2Tag, jrl::BetweenFactorPose2Tag};
  std::map<gtsam::FactorIndex, bool> potential_outlier_statuses{{0, false}, {1, true}};
  jrl::Entry entry = jrl::Entry(0, tags, graph, potential_outlier_statuses);

  auto pred = jrl::Entry::KeepTypes({jrl::BetweenFactorPose2Tag});
  jrl::Entry split = entry.filtered(pred);

  EXPECT_EQ(1, split.measurements.size());
  EXPECT_EQ(1, split.measurement_types.size());
  EXPECT_EQ(jrl::BetweenFactorPose2Tag, split.measurement_types[0]);
  EXPECT_TRUE(factor_between.equals(*split.measurements[0]));
  EXPECT_TRUE(split.potential_outlier_statuses[0]);
}