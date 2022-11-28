/*! @file MyCheetah.h
 *  @brief Build a quadruped robot model with close loop(planar 4-link leg)
 *  @date 2022-5-8
 *  @author HuNing-He
 */

#ifndef MPC_ROBOT_MODEL_H
#define MPC_ROBOT_MODEL_H
#include "MathTypes.h"
#include <rbdl/rbdl.h>
#include "Quadruped.h"

class DynamicModel;
class KinematicModel;
class FourLinkLeg;

class RobotModel {
private:
    FourLinkLeg *leg;
    DynamicModel* dyn_model;
    KinematicModel* kin_model;
    RigidBodyDynamics::Model *_model;
    Eigen::VectorXd _q;
    Eigen::VectorXd _qd;
    Eigen::VectorXd _qdd;
    Eigen::VectorXd _tau;
    /*！ For closed loop leg
     * \dot{q} = G * \dot{y}
     * \ddot{q} = G * \ddot{y} + g = G * \ddot{y} + \dot{G} * \dot{y}
     */
    Eigen::Matrix<double, 26, 18> G; // Constraint Matrix for each leg
    Eigen::VectorXd g_vec; // Constraint Matrix for each leg

public:
    unsigned int num_qdot_real;
    Quadruped *quadruped;

    explicit RobotModel(Quadruped *_quadruped, FourLinkLeg *_leg = nullptr);
    ~RobotModel();

    /*!
     * @param y: independent generalized position coordinate
     * @param yd: independent generalized velocity coordinate
     * @param ydd: independent generalized acceleration coordinate
     */
    void UpdateSystem(const Eigen::VectorXd &y, const Eigen::VectorXd &yd);
    void getMassInertia(Eigen::MatrixXd &H) const;
    void getInverseMassInertia(Eigen::MatrixXd &Hinv) const;
    void getGravity(Eigen::VectorXd &grav) const;
    void getCoriolis(Eigen::VectorXd &coriolis) const;
    //in world frame
    void getCentroidJacobian(Eigen::MatrixXd &Jcent) const;
    void getCentroidInertia(Eigen::MatrixXd &Icent) const;
    void getCentroidMatrix(Eigen::MatrixXd &Mcent) const;
    // according to "Improved computation of the humanoid centroidal dynamics and application for
    // whole body control" by Patrick Wensing
    //when rpy not equal 0, result is a little fault!!!
    void getAg_AgdotQdot(Eigen::MatrixXd &Ag, Eigen::MatrixXd &AgdotQdot);

    //test dwl's code, https://github1s.com/robot-locomotion/dwl/blob/HEAD/dwl/dwl/model/WholeBodyDynamics.cpp#L204
    void getComState_dwl(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                         double &total_mass, RigidBodyDynamics::Math::Vector3d &com_pos, RigidBodyDynamics::Math::Vector3d &com_vel,
                         RigidBodyDynamics::Math::Vector3d &ang_momentum);

    //when rpy not equal 0, result is a little fault!!!
    void getCentroidInertia_dwl(const Eigen::VectorXd &q, Eigen::MatrixXd &Icent);
    void getCoMPosition(Vec3<double> &com_pos) const;
    void getCoMVelocity(Vec3<double> &com_vel) const;
    void getPos(int link_id, Vec3<double> &pos) const;
    void getOri(int link_id, Eigen::Quaternion<double> & quat) const;
    void getLinearVel(int link_id, Vec3<double> &lin_vel) const;
    void getAngularVel(int link_id, Vec3<double> &ang_vel) const;
    void getCoMJacobi(Eigen::MatrixXd &J) const;
    void getFullJacobi(int link_id, Eigen::MatrixXd &J) const;
    void getFullJDotQdot(int link_id, Eigen::VectorXd &JDotQdot) const;
    void getWorld2BodyMatrix(Mat3<double> &World2Body);
    void getCentroidVelocity(Eigen::VectorXd & centroid_vel) const;
    void PrintLinkList();
    unsigned int FindLinkId(const char *link_name);

    void getJacobiLinearPart(int link_id, Eigen::MatrixXd &J) const;
    void getJDotQdotLinearPart(int link_id, Eigen::VectorXd &JDotQdot) const;
};
#endif  // MPC_ROBOT_MODEL_H
