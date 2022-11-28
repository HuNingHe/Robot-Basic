/*! @file Kinematic.h, modified from https://zhuanlan.zhihu.com/p/375937799
 *  @brief Build a robot kinematic model
 *  @date 2022-5-6
 *  @author HuNing-He
 */
#ifndef MPC_KINEMATIC_H
#define MPC_KINEMATIC_H
#include "MathTypes.h"
#include <rbdl/rbdl.h>
#include <utility>

class SingleLegKinematic {
private:
    bool _is_elbow = false;
    Vec3<double> _link_length;
public:
    /*!
     * @brief: Joint frames are same as MIT Cheetah
     * @param link_length: hip,thigh and leg length
     * @param is_elbow: elbow-configured leg flag
     */
    explicit SingleLegKinematic(Vec3<double> link_length, bool is_elbow = true);

    /*!
     * @param theta: 3 x 1 array, represent abad, hip and knee joint theta
     * @param pos: foot position in hip frame
     */
    void Forward(Vec3<double> theta, Vec3<double> &pos);

    /*!
     * @param p: 3 x 1 array, represent foot position in hip frame
     * @param theta: abad, hip and knee joint theta
     */
     void Inverse(Vec3<double> p, Vec3<double> &theta);

    /*!
     * @param theta: 3 x 1 array, represent abad, hip and knee joint theta
     * @param jacobi: jacobi matrix
     */
     void Jacobi(Vec3<double> theta, Mat3<double> &jacobi);
};


/*！
 * Remind that nothing need to do here when a robot has a loop constraint leg, you just get Jg and modify it
 * manually according to loop constraints
 */
class KinematicModel{
private:
    Eigen::MatrixXd Ig, Jg, Ag;
    RigidBodyDynamics::Model* _model;
    /*!
     * @brief: mainly refer to "Dynamic behaviors on the NAO robot with closed-loop whole body operational space control" by Donghyun Kim
     * @param q: generalized coordinates of model, the last one entry of q is quaternion of body orientation's w
     * @param qdot: velocity of q
     */
    void UpdateCentroidFrame(const RigidBodyDynamics::Math::VectorNd & q, const RigidBodyDynamics::Math::VectorNd & qdot);

public:
    Vec3<double> _com_pos;
    RigidBodyDynamics::Math::VectorNd _centroid_vel;
    explicit KinematicModel(RigidBodyDynamics::Model* model);
    /*!
     * @brief: get com position of link id in world frame
     * @param link_id: body id
     * @param pos: position
     */
    void getPos(int link_id, Vec3<double> & pos);
    /*!
     * @brief: get body orientation of link id in world frame
     * @param link_id: body id
     * @param quat: quaternion of body id in world frame
     */
    void getOri(int link_id, Eigen::Quaternion<double> & quat);
    /*!
     * @brief: get body linear velocity of link id in world frame
     * @param link_id: body id
     * @param vel: linear velocity of body id in world frame(x, y, z)
     */
    void getLinearVel(int link_id, Vec3<double> & vel);
    /*!
     * @brief: get body angular velocity of link id in world frame
     * @param link_id: body id
     * @param ang_vel: angular velocity of body id in world frame $(w_x, w_y, w_z)$
     */
    void getAngularVel(int link_id, Vec3<double> & ang_vel);
    /*!
     * @brief: jacobi matrix of link id in world frame
     * @param link_id: body id
     * @param J: jacobi matrix of body id about q
     */
    void getJacobi(int link_id, Eigen::MatrixXd & J);

    /*!
     * @brief: Calculate $/dot{J}/dot{q}$, this is easy by calculating body acceleration when $\ddot{q} = 0$
     * with: $\ddot{x} = J\ddot{q}+/dot{J}/dot{q}$
     * @param link_id: body id
     * @param JDotQdot: jacobi matrix of body id about q
     */
    void getJDotQdot(int link_id, RigidBodyDynamics::Math::VectorNd & JDotQdot);

    void getCoMJacobi  (Eigen::MatrixXd & J) const;
    void getCoMPos  (Vec3<double> & com_pos) const;
    void getCoMVel (Vec3<double> & com_vel) const;
    void getCentroidInertia(Eigen::MatrixXd & Icent){ Icent = Ig; }
    void getCentroidJacobi(Eigen::MatrixXd & Jcent){ Jcent = Jg; }
    void getCentroidMatrix(Eigen::MatrixXd & Mcent){ Mcent = Ag; }
    /*!
     * @param BodyMatrix: rotational matrix(SO3) from base frame to body frame
     */
    void getWorldToBodyMatrix(Mat3<double> & BodyMatrix);

    void UpdateKinematics(const RigidBodyDynamics::Math::VectorNd & q, const RigidBodyDynamics::Math::VectorNd & qdot);

    unsigned int find_body_id(int id) const;
    unsigned int find_body_id(const char* link_name) const;
    void DisplayLinks();
};

#endif // MPC_KINEMATIC_H