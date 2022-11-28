/*! @file OrientationEstimator.h
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */
#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "StateEstimatorContainer.h"

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */
class VectorNavOrientationEstimator : public GenericEstimator {
 public:
  void run() override;
  void setup() override {}
  
 protected:
  bool _b_first_visit = true;
  Quat<double> _ori_ini_inv;
};


#endif  // PROJECT_ORIENTATIONESTIMATOR_H
