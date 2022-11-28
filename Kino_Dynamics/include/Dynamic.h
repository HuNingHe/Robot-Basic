/*! @file Dynamic.h, modified from https://zhuanlan.zhihu.com/p/375937799
 *  @brief Build a robot's Dynamic model
 *  @date 2022-5-6
 *  @author HuNing-He
 */

#ifndef MPC_DYNAMIC_H
#define MPC_DYNAMIC_H
#include <rbdl/rbdl.h>

class DynamicModel{
public:

    explicit DynamicModel(RigidBodyDynamics::Model* model);

    void getMassInertia(Eigen::MatrixXd &h);
    void getInverseMassInertia(Eigen::MatrixXd &hinv);
    void getGravity(Eigen::VectorXd &grav);
    void getCoriolis(Eigen::VectorXd &coriolis);
    void UpdateDynamics(const RigidBodyDynamics::Math::VectorNd & q, const RigidBodyDynamics::Math::VectorNd & qdot);
//    void

private:
    /*!
     * H: Inertial Matrix
     * Hinv: inverse Inertial Matrix
     * G: Generalized Gravity Force Matrix
     * C: Coriolis Force Matrix
     */
    Eigen::MatrixXd H;
    Eigen::MatrixXd Hinv;
    Eigen::VectorXd G;
    Eigen::VectorXd C;
    RigidBodyDynamics::Model* _model;
};


#endif // MPC_DYNAMIC_H
