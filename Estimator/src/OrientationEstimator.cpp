/*! @file OrientationEstimator.cpp
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
#include "Orientation.h"
#include "OrientationEstimator.h"
/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
void VectorNavOrientationEstimator::run() {
    this->_stateEstimatorData.result->orientation = this->_stateEstimatorData.vectorNavData->quat;
    this->_stateEstimatorData.result->rpy = Ori::quatToRPY(this->_stateEstimatorData.result->orientation);
//    printf("yaw %f\n", _stateEstimatorData.result->rpy[2]);
    Vec3<double> eu = this->_stateEstimatorData.result->rpy;
    Eigen::Matrix3d T;
    T.setZero();
    T << cos(eu[1]) * cos(eu[2]), -sin(eu[2]), 0,
         cos(eu[1]) * sin(eu[2]), cos(eu[2]), 0,
         0, 0, 1;
    this->_stateEstimatorData.result->rBody = this->_stateEstimatorData.result->orientation.matrix();

    this->_stateEstimatorData.result->omegaBody = this->_stateEstimatorData.vectorNavData->gyro;

    this->_stateEstimatorData.result->omegaWorld = T * this->_stateEstimatorData.result->omegaBody;

    this->_stateEstimatorData.result->aBody = this->_stateEstimatorData.vectorNavData->accelerometer;

    this->_stateEstimatorData.result->aWorld = this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->aBody;
}
