#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/StereoFactor.h>
#include <jrl/Dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>
#include <jrl/IOMeasurements.h>
#include <jrl/IOValues.h>

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

#define EXPECT_MATRICES_EQ(M_actual, M_expected) \
  EXPECT_TRUE(M_actual.isApprox(M_expected, 1e-6)) << "  Actual:\n" << M_actual << "\nExpected:\n" << M_expected


TEST(Custom, Factor){
    // Make dummy factors
    gtsam::noiseModel::Gaussian::shared_ptr cov = gtsam::noiseModel::Gaussian::Covariance(Eigen::Matrix3d::Identity());
    gtsam::PriorFactor<gtsam::Pose2> prior(X(0), gtsam::Pose2::Identity(), cov);
    std::string customType = "CustomFactor";

    // Make dataset
    gtsam::NonlinearFactorGraph graph;
    graph.push_back(prior);
    jrl::Entry entry(0, {customType}, graph);

    // Make custom writer/parser (Copied from prior information)
    jrl::Writer writer;
    writer.registerMeasurementSerializer(customType, [customType](gtsam::NonlinearFactor::shared_ptr& factor) { return jrl::io_measurements::serializePrior<gtsam::Pose2>(&jrl::io_values::serialize<gtsam::Pose2>, customType, factor); });

    jrl::Parser parser;
    parser.registerMeasurementParser(customType, [](const json& input){ return jrl::io_measurements::parsePrior<gtsam::Pose2>(&jrl::io_values::parse<gtsam::Pose2>, input); });

    // Send it round trip!
    auto serialized = writer.serializeMeasurements({entry});
    jrl::Entry read = parser.parseMeasurements(serialized)[0];
    gtsam::PriorFactor<gtsam::Pose2>::shared_ptr read_factor = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose2>>(entry.measurements.at(0));

    EXPECT_TRUE(prior.equals(*read_factor));
}


TEST(Custom, Value){
    // Make dummy factors
    gtsam::Pose2 x(1,2,3);
    std::string customType = "CustomValue";

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values theta;
    jrl::ValueTypes types;

    // Make dataset
    theta.insert(X(0), x);
    types[X(0)] = customType;
    jrl::TypedValues typed_values(theta, types);

    // Make custom writer/parser (Copied from prior information)
    jrl::Writer writer;
    writer.registerValueSerializer(customType, [customType](gtsam::Key key, gtsam::Values& vals) { return jrl::io_values::serialize<gtsam::Pose2>(vals.at<gtsam::Pose2>(key)); });

    jrl::Parser parser;
    parser.registerValueParser(customType, [](const json& input, gtsam::Key key, gtsam::Values& accum) { return jrl::io_values::valueAccumulator<gtsam::Pose2>(&jrl::io_values::parse<gtsam::Pose2>, input, key, accum); });

    // Send it round trip!
    auto serialized = writer.serializeValues(typed_values);
    gtsam::Values read = parser.parseValues(serialized).values;
    gtsam::Pose2 x_read = read.at<gtsam::Pose2>(X(0));

    EXPECT_TRUE(x.equals(x_read));
}