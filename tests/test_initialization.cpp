#include <gtsam/geometry/Pose2.h>
#include <jrl/Dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/IOMeasurements.h>
#include <jrl/Initialization.h>

#include "gtest/gtest.h"

/**********************************************************************************************************************/
TEST(Initialization, Prior) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(0, gtsam::Pose2(1, 1, 2), gtsam::noiseModel::Unit::Create(3));
  jrl::Entry entry(0, {jrl::PriorFactorPose2Tag}, graph);
  jrl::Initializer init;
  gtsam::Values result = init.initialization(entry, gtsam::Values());

  ASSERT_EQ(result.size(), 1);
  ASSERT_TRUE(result.at<gtsam::Pose2>(0).equals(gtsam::Pose2(1, 1, 2)));
}

/**********************************************************************************************************************/
TEST(Initialization, Between) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(0, 1, gtsam::Pose2(1, 1, 2),
                                                           gtsam::noiseModel::Unit::Create(3));
  jrl::Entry entry(0, {jrl::BetweenFactorPose2Tag}, graph);

  gtsam::Values current_solution;
  current_solution.insert(0, gtsam::Pose2(0, 0, 0));

  jrl::Initializer init;
  gtsam::Values result = init.initialization(entry, current_solution);

  ASSERT_EQ(result.size(), 1);
  ASSERT_TRUE(result.at<gtsam::Pose2>(1).equals(gtsam::Pose2(1, 1, 2)));
}

/**********************************************************************************************************************/
TEST(Initialization, Dependency) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(0, gtsam::Pose2(0, 0, 0), gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(0, 1, gtsam::Pose2(1, 1, 2),
                                                           gtsam::noiseModel::Unit::Create(3));
  jrl::Entry entry(0, {jrl::PriorFactorPose2Tag, jrl::BetweenFactorPose2Tag}, graph);
  jrl::Initializer init;
  gtsam::Values result = init.initialization(entry, gtsam::Values());

  ASSERT_EQ(result.size(), 2);
  ASSERT_TRUE(result.at<gtsam::Pose2>(0).equals(gtsam::Pose2(0, 0, 0)));
  ASSERT_TRUE(result.at<gtsam::Pose2>(1).equals(gtsam::Pose2(1, 1, 2)));
}

/**********************************************************************************************************************/
TEST(Initialization, DependencyComplex) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(0, gtsam::Pose2(0, 0, 0), gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(0, 1, gtsam::Pose2(1, 0, 0),
                                                           gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(1, 2, gtsam::Pose2(1, 0, 0),
                                                           gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(2, 3, gtsam::Pose2(1, 0, 0),
                                                           gtsam::noiseModel::Unit::Create(3));

  jrl::Entry entry(
      0, {jrl::PriorFactorPose2Tag, jrl::BetweenFactorPose2Tag, jrl::BetweenFactorPose2Tag, jrl::BetweenFactorPose2Tag},
      graph);
  jrl::Initializer init;
  gtsam::Values result = init.initialization(entry, gtsam::Values());

  ASSERT_EQ(result.size(), 4);
  ASSERT_TRUE(result.at<gtsam::Pose2>(0).equals(gtsam::Pose2(0, 0, 0)));
  ASSERT_TRUE(result.at<gtsam::Pose2>(1).equals(gtsam::Pose2(1, 0, 0)));
  ASSERT_TRUE(result.at<gtsam::Pose2>(2).equals(gtsam::Pose2(2, 0, 0)));
  ASSERT_TRUE(result.at<gtsam::Pose2>(3).equals(gtsam::Pose2(3, 0, 0)));
}

/**********************************************************************************************************************/
TEST(Initialization, Priority) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(0, gtsam::Pose2(0, 0, 0), gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(0, 1, gtsam::Pose2(1, 1, 2),
                                                           gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(1, gtsam::Pose2(2, 2, 2), gtsam::noiseModel::Unit::Create(3));

  jrl::Entry entry(0, {jrl::PriorFactorPose2Tag, jrl::BetweenFactorPose2Tag, jrl::PriorFactorPose2Tag}, graph);
  jrl::Initializer init;
  gtsam::Values result = init.initialization(entry, gtsam::Values());

  ASSERT_EQ(result.size(), 2);
  ASSERT_TRUE(result.at<gtsam::Pose2>(0).equals(gtsam::Pose2(0, 0, 0)));
  ASSERT_TRUE(result.at<gtsam::Pose2>(1).equals(gtsam::Pose2(2, 2, 2)));
}

/**********************************************************************************************************************/
TEST(Initialization, UnsatisfiedDependency) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(0, 1, gtsam::Pose2(1, 1, 2),
                                                           gtsam::noiseModel::Unit::Create(3));
  jrl::Entry entry(0, {jrl::BetweenFactorPose2Tag}, graph);
  jrl::Initializer init;
  EXPECT_THROW(init.initialization(entry, gtsam::Values()), std::runtime_error);
}

/**********************************************************************************************************************/
TEST(Initialization, NoTopologicalOrder) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(0, 1, gtsam::Pose2(1, 1, 2),
                                                           gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(1, 0, gtsam::Pose2(-1, -1, -2),
                                                           gtsam::noiseModel::Unit::Create(3));
  jrl::Entry entry(0, {jrl::BetweenFactorPose2Tag, jrl::BetweenFactorPose2Tag}, graph);
  jrl::Initializer init;
  EXPECT_THROW(init.initialization(entry, gtsam::Values()), std::runtime_error);
}

/**********************************************************************************************************************/
TEST(ForwardModels, BearingRangePose2) {
  jrl::BearingRangeForwardModel<gtsam::Pose2> fwd_mdl;
  gtsam::Pose2 p0(1, 3, 1.2345);
  gtsam::Pose2 p1(-2, -75, -2.643);
  auto b = p0.bearing(p1);
  auto r = p0.range(p1);
  gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>::shared_ptr factor =
      boost::make_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>>(
          0, 1, gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>(b, r), gtsam::noiseModel::Unit::Create(2));
  gtsam::Values inputs;
  inputs.insert(0, p0);

  gtsam::Values result = fwd_mdl.predict(factor, inputs);

  ASSERT_TRUE(result.at<gtsam::Pose2>(1).equals(gtsam::Pose2(-2, -75, 0)));
}

/**********************************************************************************************************************/
TEST(ForwardModels, BearingRangePose3) {
  jrl::BearingRangeForwardModel<gtsam::Pose3> fwd_mdl;
  gtsam::Pose3 p0(gtsam::Rot3::RzRyRx(-1, -2, -3), gtsam::Point3(5, -6, 7));
  gtsam::Pose3 p1(gtsam::Rot3::RzRyRx(9, 4, 7), gtsam::Point3(-8, 1, 2));
  auto b = p0.bearing(p1);
  auto r = p0.range(p1);
  gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>::shared_ptr factor =
      boost::make_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>>(
          0, 1, gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3>(b, r), gtsam::noiseModel::Unit::Create(3));
  gtsam::Values inputs;
  inputs.insert(0, p0);

  gtsam::Values result = fwd_mdl.predict(factor, inputs);

  ASSERT_TRUE(result.at<gtsam::Pose3>(1).equals(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-8, 1, 2))));
}
