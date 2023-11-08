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