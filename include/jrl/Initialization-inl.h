#pragma once
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "jrl/Initialization.h"

namespace jrl {
/**
 * ########  ########  ####  #######  ########
 * ##     ## ##     ##  ##  ##     ## ##     ##
 * ##     ## ##     ##  ##  ##     ## ##     ##
 * ########  ########   ##  ##     ## ########
 * ##        ##   ##    ##  ##     ## ##   ##
 * ##        ##    ##   ##  ##     ## ##    ##
 * ##        ##     ## ####  #######  ##     ##
 */
/**********************************************************************************************************************/
template <typename T>
ForwardMeasurementModel::Signature PriorForwardModel<T>::signature(
    const gtsam::NonlinearFactor::shared_ptr& measurement) const {
  typename gtsam::PriorFactor<T>::shared_ptr mptr =
      std::dynamic_pointer_cast<typename gtsam::PriorFactor<T>>(measurement);
  return Signature({}, {mptr->key()});
}

/**********************************************************************************************************************/
template <typename T>
gtsam::Values PriorForwardModel<T>::predict(const gtsam::NonlinearFactor::shared_ptr& measurement,
                                            const gtsam::Values& inputs) const {
  typename gtsam::PriorFactor<T>::shared_ptr mptr =
      std::dynamic_pointer_cast<typename gtsam::PriorFactor<T>>(measurement);
  gtsam::Values val_result;
  val_result.insert(mptr->key(), mptr->prior());
  return val_result;
}

/**
 * ########  ######## ######## ##      ## ######## ######## ##    ##
 * ##     ## ##          ##    ##  ##  ## ##       ##       ###   ##
 * ##     ## ##          ##    ##  ##  ## ##       ##       ####  ##
 * ########  ######      ##    ##  ##  ## ######   ######   ## ## ##
 * ##     ## ##          ##    ##  ##  ## ##       ##       ##  ####
 * ##     ## ##          ##    ##  ##  ## ##       ##       ##   ###
 * ########  ########    ##     ###  ###  ######## ######## ##    ##
 */
/**********************************************************************************************************************/
template <typename T>
ForwardMeasurementModel::Signature BetweenForwardModel<T>::signature(
    const gtsam::NonlinearFactor::shared_ptr& measurement) const {
  typename gtsam::BetweenFactor<T>::shared_ptr mptr =
      std::dynamic_pointer_cast<typename gtsam::BetweenFactor<T>>(measurement);
  return Signature({mptr->key1()}, {mptr->key2()});
}
/**********************************************************************************************************************/
template <typename T>
gtsam::Values BetweenForwardModel<T>::predict(const gtsam::NonlinearFactor::shared_ptr& measurement,
                                              const gtsam::Values& inputs) const {
  typename gtsam::BetweenFactor<T>::shared_ptr mptr =
      std::dynamic_pointer_cast<typename gtsam::BetweenFactor<T>>(measurement);
  T composed = gtsam::traits<T>::Compose(inputs.at<T>(mptr->key1()), mptr->measured());

  gtsam::Values val_result;
  val_result.insert(mptr->key2(), composed);
  return val_result;
}

/**
 * ########  ########    ###    ########  #### ##    ##  ######      ########     ###    ##    ##  ######   ########
 * ##     ## ##         ## ##   ##     ##  ##  ###   ## ##    ##     ##     ##   ## ##   ###   ## ##    ##  ##
 * ##     ## ##        ##   ##  ##     ##  ##  ####  ## ##           ##     ##  ##   ##  ####  ## ##        ##
 * ########  ######   ##     ## ########   ##  ## ## ## ##   ####    ########  ##     ## ## ## ## ##   #### ######
 * ##     ## ##       ######### ##   ##    ##  ##  #### ##    ##     ##   ##   ######### ##  #### ##    ##  ##
 * ##     ## ##       ##     ## ##    ##   ##  ##   ### ##    ##     ##    ##  ##     ## ##   ### ##    ##  ##
 * ########  ######## ##     ## ##     ## #### ##    ##  ######      ##     ## ##     ## ##    ##  ######   ########
 */
/**********************************************************************************************************************/
template <typename A1, typename A2>
ForwardMeasurementModel::Signature BearingRangeForwardModel<A1, A2>::signature(
    const gtsam::NonlinearFactor::shared_ptr& measurement) const {
  typename gtsam::BearingRangeFactor<A1, A2>::shared_ptr mptr =
      std::dynamic_pointer_cast<typename gtsam::BearingRangeFactor<A1, A2>>(measurement);
  return Signature({mptr->keys().front()}, {mptr->keys().back()});
}

/**********************************************************************************************************************/
template <typename A1, typename A2>
gtsam::Values BearingRangeForwardModel<A1, A2>::predict(const gtsam::NonlinearFactor::shared_ptr& measurement,
                                                        const gtsam::Values& inputs) const {
  typename gtsam::BearingRangeFactor<A1, A2>::shared_ptr mptr =
      std::dynamic_pointer_cast<typename gtsam::BearingRangeFactor<A1, A2>>(measurement);
  A2 projected = BearingRangeForwardModel<A1, A2>::project(inputs.at<A1>(mptr->keys().front()), mptr->measured());

  gtsam::Values val_result;
  val_result.insert(mptr->keys().back(), projected);
  return val_result;
}

}  // namespace jrl
