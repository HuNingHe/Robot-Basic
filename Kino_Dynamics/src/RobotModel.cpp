#include "RobotModel.h"
#include "Kinematic.h"
#include "Dynamic.h"
#include "FourLinkLeg.h"

using namespace RigidBodyDynamics;

RobotModel::RobotModel(Quadruped *_quadruped, FourLinkLeg *_leg) {
    leg = _leg;
    quadruped = _quadruped;
    _model = new Model();
    _model->gravity = quadruped->gravity;

    Body floating_body = Body(quadruped->body_mass, quadruped->body_com, quadruped->body_inertial);

    Body lf_hip = Body(quadruped->hip_mass, quadruped->lf_hip_com, quadruped->lf_hip_inertial);
    Body rf_hip = Body(quadruped->hip_mass, quadruped->rf_hip_com, quadruped->rf_hip_inertial);
    Body rh_hip = Body(quadruped->hip_mass, quadruped->rh_hip_com, quadruped->rh_hip_inertial);
    Body lh_hip = Body(quadruped->hip_mass, quadruped->lh_hip_com, quadruped->lh_hip_inertial);

    Body left_thigh = Body(quadruped->thigh_mass, quadruped->left_thigh_com, quadruped->left_thigh_inertial);
    Body right_thigh = Body(quadruped->thigh_mass, quadruped->right_thigh_com, quadruped->right_thigh_inertial);

    Body left_rotor = Body(quadruped->rotor_mass, quadruped->left_rotor_com, quadruped->left_rotor_inertial);
    Body right_rotor = Body(quadruped->rotor_mass, quadruped->right_rotor_com, quadruped->right_rotor_inertial);

    Body link = Body(quadruped->link_mass, quadruped->link_com, quadruped->link_inertial);
    Body left_leg = Body(quadruped->leg_mass, quadruped->leg_com, quadruped->left_leg_inertial);
    Body right_leg = Body(quadruped->leg_mass, quadruped->leg_com, quadruped->right_leg_inertial);
    Body toe = Body(quadruped->toe_mass, quadruped->toe_com, quadruped->toe_inertial);

    /*!
     * @brief: RBDL has a special joint type :JointTypeFloatingBase.
     * The first three DoF are translations along X,Y, and Z.
     * For the rotational part it uses a JointTypeSpherical joint. It describe the floating body's orientation using quaternion.
     * It is internally modeled by a JointTypeTranslationXYZ and a JointTypeSpherical joint.So, we usually use this joint other than "floating_joint" above
     * w in quaternion is append at the end of q-vector.
     * additionally, qdot is translational velocity first and then angular velocity in body frame
     */

    uint base_id = _model->AddBody(0, SpatialTransform(), Joint(JointTypeFloatingBase), floating_body, "Trunk");
    // LF Leg
    uint lf_hip_id = _model->AddBody(base_id, Xtrans(quadruped->hipLocation.row(0)), Joint(JointTypeRevoluteX), lf_hip, "LF_Hip");
    uint lf_thigh_id = _model->AddBody(lf_hip_id, Xtrans(quadruped->thighLocation.row(0)), Joint(JointTypeRevoluteY), left_thigh, "LF_Thigh");
    uint lf_rotor_id = _model->AddBody(lf_thigh_id, Xtrans(quadruped->rotorLocation.row(0)), Joint(JointTypeRevoluteY), left_rotor, "LF_Rotor");
    uint lf_link_id = _model->AddBody(lf_rotor_id, Xtrans(quadruped->linkLocation.row(0)), Joint(JointTypeRevoluteY), link, "LF_Link");
    uint lf_leg_id = _model->AddBody(lf_link_id, Xtrans(quadruped->legLocation), Joint(JointTypeRevoluteY), left_leg, "LF_Leg");
    _model->AddBody(lf_leg_id, Xtrans(quadruped->toeLocation), Joint(JointTypeFixed), toe, "LF_Foot");

    // RF Leg
    uint rf_hip_id = _model->AddBody(base_id, Xtrans(quadruped->hipLocation.row(1)), Joint(JointTypeRevoluteX), rf_hip, "RF_Hip");
    uint rf_thigh_id = _model->AddBody(rf_hip_id, Xtrans(quadruped->thighLocation.row(1)), Joint(JointTypeRevoluteY), right_thigh, "RF_Thigh");
    uint rf_rotor_id = _model->AddBody(rf_thigh_id, Xtrans(quadruped->rotorLocation.row(1)), Joint(JointTypeRevoluteY), right_rotor, "RF_Rotor");
    uint rf_link_id = _model->AddBody(rf_rotor_id, Xtrans(quadruped->linkLocation.row(1)), Joint(JointTypeRevoluteY), link, "RF_Link");
    uint rf_leg_id = _model->AddBody(rf_link_id, Xtrans(quadruped->legLocation), Joint(JointTypeRevoluteY), right_leg, "RF_Leg");
    _model->AddBody(rf_leg_id, Xtrans(quadruped->toeLocation), Joint(JointTypeFixed), toe, "RF_Foot");

    // RH Leg
    uint rh_hip_id = _model->AddBody(base_id, Xtrans(quadruped->hipLocation.row(2)), Joint(JointTypeRevoluteX), rh_hip, "RH_Hip");
    uint rh_thigh_id = _model->AddBody(rh_hip_id, Xtrans(quadruped->thighLocation.row(2)), Joint(JointTypeRevoluteY), right_thigh, "RH_Thigh");
    uint rh_rotor_id = _model->AddBody(rh_thigh_id, Xtrans(quadruped->rotorLocation.row(2)), Joint(JointTypeRevoluteY), right_rotor, "RH_Rotor");
    uint rh_link_id = _model->AddBody(rh_rotor_id, Xtrans(quadruped->linkLocation.row(2)), Joint(JointTypeRevoluteY), link, "RH_Link");
    uint rh_leg_id = _model->AddBody(rh_link_id, Xtrans(quadruped->legLocation), Joint(JointTypeRevoluteY), right_leg, "RH_Leg");
    _model->AddBody(rh_leg_id, Xtrans(quadruped->toeLocation), Joint(JointTypeFixed), toe, "RH_Foot");

    // LH Leg
    uint lh_hip_id = _model->AddBody(base_id, Xtrans(quadruped->hipLocation.row(3)), Joint(JointTypeRevoluteX), lh_hip, "LH_Hip");
    uint lh_thigh_id = _model->AddBody(lh_hip_id, Xtrans(quadruped->thighLocation.row(3)), Joint(JointTypeRevoluteY), left_thigh, "LH_Thigh");
    uint lh_rotor_id = _model->AddBody(lh_thigh_id, Xtrans(quadruped->rotorLocation.row(3)), Joint(JointTypeRevoluteY), left_rotor, "LH_Rotor");
    uint lh_link_id = _model->AddBody(lh_rotor_id, Xtrans(quadruped->linkLocation.row(3)), Joint(JointTypeRevoluteY), link, "LH_Link");
    uint lh_leg_id = _model->AddBody(lh_link_id, Xtrans(quadruped->legLocation), Joint(JointTypeRevoluteY), left_leg, "LH_Leg");
    _model->AddBody(lh_leg_id, Xtrans(quadruped->toeLocation), Joint(JointTypeFixed), toe, "LH_Foot");

    _q = Eigen::VectorXd::Zero(_model->q_size);
    _qd = Eigen::VectorXd::Zero(_model->qdot_size);
    _qdd = Eigen::VectorXd::Zero(_model->qdot_size);
    _tau = Eigen::VectorXd::Zero(18); // should be 26 if it is open chain
    G.setZero();
    G.block<8, 8>(0, 0) = Eigen::Matrix<double, 8, 8>::Identity();
    G(8, 8) = 1;
    G(11, 9) = 1;
    G(12, 10) = 1;
    G(13, 11) = 1;
    G(16, 12) = 1;
    G(17, 13) = 1;
    G(18, 14) = 1;
    G(21, 15) = 1;
    G(22, 16) = 1;
    G(23, 17) = 1;
    g_vec = Eigen::VectorXd::Zero(_model->qdot_size);
    num_qdot_real = 18;
    kin_model = new KinematicModel(_model);
    dyn_model = new DynamicModel(_model);
}

RobotModel::~RobotModel(){
    delete _model;
    _model = nullptr;
    delete dyn_model;
    dyn_model = nullptr;
    delete kin_model;
    kin_model = nullptr;
}

void RobotModel::UpdateSystem(const Eigen::VectorXd &y, const Eigen::VectorXd &yd) {
    if (leg != nullptr) {
        // 每条腿两个闭链关节, 一共八个
        assert(y.rows() == _q.rows() - 8 && yd.rows() == _qd.rows() - 8);
        Eigen::Vector3d leg_theta;
        for (int leg_idx = 0; leg_idx < 4; ++leg_idx) {
            G(leg_idx * 5 + 9, leg_idx * 3 + 8) = leg->Theta2Diff(y[leg_idx * 3 + 8]);
            G(leg_idx * 5 + 10, leg_idx * 3 + 8) = leg->ReductionRatio(y[leg_idx * 3 + 8]) - 1 - G(leg_idx * 5 + 9, leg_idx * 3 + 8);
            g_vec[leg_idx * 5 + 9] = (0.02951376 * pow(y[leg_idx * 3 + 8] + leg->_motor_correction , 2) - 0.09537444*(y[leg_idx * 3 + 8] + leg->_motor_correction) + 0.10199759) * pow(yd[leg_idx * 3 + 8], 2);
            g_vec[leg_idx * 5 + 10] = -g_vec[leg_idx * 5 + 9] + (-0.15599752 * pow(y[leg_idx * 3 + 8] + leg->_motor_correction, 3) + 0.79962495*pow(y[leg_idx * 3 + 8] + leg->_motor_correction, 2) - 1.73932138*(y[leg_idx * 3 + 8] + leg->_motor_correction) + 1.3528932) * pow(yd[leg_idx * 3 + 8], 2);
            leg_theta = leg->UpdateTheta(y[leg_idx * 3 + 8]);
            for (int joint_idx = 0; joint_idx < 3; ++joint_idx) {
                _q[leg_idx * 5 + 8 + joint_idx] = leg_theta(joint_idx);
                _qd[leg_idx * 5 + 8  + joint_idx] = yd[leg_idx * 3 + 8] * G(leg_idx * 5 + 8 + joint_idx, leg_idx * 3 + 8);
            }
        }

        for (int i = 0; i < 8; ++i) {
            _q[i] = y[i];
            _qd[i] = yd[i];
        }
        // G是稀疏矩阵, 避免矩阵乘法，此处也可以_qd = G * yd, 但节约时间, 此处不用
        _q[11] = y[9];
        _q[12] = y[10];
        _q[16] = y[12];
        _q[17] = y[13];
        _q[21] = y[15];
        _q[22] = y[16];
        _q[26] = y[18];

        _qd[11] = yd[9];
        _qd[12] = yd[10];
        _qd[16] = yd[12];
        _qd[17] = yd[13];
        _qd[21] = yd[15];
        _qd[22] = yd[16];
    }else{
        assert(y.rows() == _q.rows() && yd.rows() == _qd.rows());
        _q = y;
        _qd = yd;
    }
    // qdd always be zeros
    UpdateKinematicsCustom(*_model, &_q, &_qd, &_qdd);
    dyn_model->UpdateDynamics(_q, _qd);
    kin_model->UpdateKinematics(_q, _qd);
}

void RobotModel::getCentroidInertia(Eigen::MatrixXd & Icent) const {
    kin_model->getCentroidInertia(Icent);
}

void RobotModel::getCentroidJacobian(Eigen::MatrixXd & Jcent) const {
    Jcent.setZero();
    kin_model->getCentroidJacobi(Jcent);
}

void RobotModel::getCentroidMatrix(Eigen::MatrixXd & Mcent) const{
    kin_model->getCentroidMatrix(Mcent);
}

void RobotModel::getInverseMassInertia(Eigen::MatrixXd & Hinv) const {
    dyn_model->getInverseMassInertia(Hinv);
}

void RobotModel::getMassInertia(Eigen::MatrixXd & H) const {
    if (leg != nullptr){
        Eigen::MatrixXd H_tmp;
        dyn_model->getMassInertia(H_tmp);
        H = G.transpose() * H_tmp * G;
    } else{
        dyn_model->getMassInertia(H);
    }
}

void RobotModel::getGravity(Eigen::VectorXd & grav) const {
    if (leg != nullptr){
        Eigen::VectorXd grav_tmp;
        dyn_model->getGravity(grav_tmp);
        grav = G.transpose() * grav_tmp;
    } else{
        dyn_model->getGravity(grav);
    }
}

void RobotModel::getCoriolis(Eigen::VectorXd & coriolis) const {
    if (leg != nullptr){
        Eigen::MatrixXd H_tmp;
        dyn_model->getMassInertia(H_tmp);
        Eigen::VectorXd C_tmp;
        dyn_model->getCoriolis(C_tmp);
        coriolis = G.transpose() * (C_tmp + H_tmp * g_vec);
    } else{
        dyn_model->getCoriolis(coriolis);
    }
}

void RobotModel::getFullJacobi(int link_id, Eigen::MatrixXd & J) const{
    kin_model->getJacobi(link_id, J);
}

void RobotModel::getJacobiLinearPart(int link_id, Eigen::MatrixXd & J) const{
    kin_model->getJacobi(link_id, J);
    /*！
     * J * \dot{q} = \dot{x} = J * G * \dot{y}
     * s.t. J` = J * G
     * For speed consideration, we don't use J * G.
     * We use method as below, which is equivalent with J * G
     */
    if (leg != nullptr) {
        J.col(8) += J.col(9) * G(9, 8) + J.col(10) * G(10, 8);
        J.col(9) = J.col(11);
        J.col(10) = J.col(12);
        J.col(11) = J.col(13) + J.col(14) * G(14, 11) + J.col(15) * G(15, 11);
        J.col(12) = J.col(16);
        J.col(13) = J.col(17);
        J.col(14) = J.col(18) + J.col(19) * G(19, 14) + J.col(20) * G(20, 14);
        J.col(15) = J.col(21);
        J.col(16) = J.col(22);
        J.col(17) = J.col(23) + J.col(24) * G(24, 17) + J.col(25) * G(25, 17);
        J.block(0, 0, 3, _model->qdot_size) = J.block(3, 0, 3, _model->qdot_size);
        J.conservativeResize(3, num_qdot_real);
    } else{
        J.block(0, 0, 3, _model->qdot_size) = J.block(3, 0, 3, _model->qdot_size);
        J.conservativeResize(3, _model->qdot_size);
    }
}

void RobotModel::getJDotQdotLinearPart(int link_id, Eigen::VectorXd & JDotQdot) const{
    kin_model->getJDotQdot(link_id, JDotQdot);
    if (leg != nullptr) {
        /*! based on \ddot{y} = 0
         * \ddot{x} = J * \ddot{q} + \dot{J} * \dot{q}
         * = J * (G * \ddot{y} + g) + \dot{J} * \dot{q} = J * g + \dot{J} * \dot{q}
         * so, add J_tmp * g is enough
         */
        Eigen::MatrixXd J_tmp;
        kin_model->getJacobi(link_id, J_tmp);
        JDotQdot += J_tmp * g_vec;
    }
    JDotQdot.head(3) = JDotQdot.tail(3);
    JDotQdot.conservativeResize(3, 1);
}

void RobotModel::getFullJDotQdot(int link_id, Eigen::VectorXd & JDotQdot) const{
    kin_model->getJDotQdot(link_id, JDotQdot);
}

void RobotModel::getPos(int link_id, Vec3<double> & pos) const{
    kin_model->getPos(link_id, pos);
}

void RobotModel::getOri(int link_id, Eigen::Quaternion<double> & quat) const{
    kin_model->getOri(link_id, quat);
}

void RobotModel::getLinearVel(int link_id, Vec3<double> & vel) const{
    kin_model->getLinearVel(link_id, vel);
}

void RobotModel::getAngularVel(int link_id, Vec3<double> & ang_vel) const{
    kin_model->getAngularVel(link_id, ang_vel);
}

void RobotModel::getCoMJacobi(Eigen::MatrixXd & J) const{
    J = Eigen::MatrixXd::Zero(3, _model->qdot_size);
    kin_model->getCoMJacobi(J);
}

void RobotModel::getCoMPosition(Vec3<double> & com_pos) const{
    com_pos = kin_model->_com_pos;
}

void RobotModel::getCoMVelocity(Vec3<double> & com_vel) const{
    kin_model->getCoMVel(com_vel);
}

void RobotModel::getCentroidVelocity(Eigen::VectorXd & centroid_vel) const{
    centroid_vel = kin_model->_centroid_vel;
}

void RobotModel::getWorld2BodyMatrix(Mat3<double> & World2Body){
    kin_model -> getWorldToBodyMatrix(World2Body);
}

void RobotModel::PrintLinkList(){
    kin_model->DisplayLinks();
}

unsigned int RobotModel::FindLinkId(const char *_link_name){
    return kin_model->find_body_id(_link_name);
}

void RobotModel::getAg_AgdotQdot(Eigen::MatrixXd & Ag, Eigen::MatrixXd & AgdotQdot){
    Eigen::MatrixXd H, U1;
    Eigen::VectorXd Cq;
    Mat3<double> body_R_base;
    Vec3<double> compos;

    U1.setZero(6, _model->qdot_size);
    U1.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);

    dyn_model->getMassInertia(H);
    Eigen::MatrixXd base_lin_mat = H.block(0,0,3,6);
    Eigen::MatrixXd base_ang_mat = H.block(3,0,3,6);
    Eigen::MatrixXd joint_lin_mat = H.block(0,6,3,12);
    Eigen::MatrixXd joint_ang_mat = H.block(3,6,3,12);
    //moemntum: rpy xyz
    //vel: rpy xyz joint
    H.block(0,0,3,6) << base_ang_mat.rightCols(3), base_ang_mat.leftCols(3);
    H.block(3,0,3,6) << base_lin_mat.rightCols(3), base_lin_mat.leftCols(3);
    H.block(0,6,3,12) = joint_ang_mat;
    H.block(3,6,3,12) = joint_lin_mat;
    dyn_model->getCoriolis(Cq);
    getCoMPosition(compos);
    RigidBodyDynamics::Math::SpatialTransform base_X_com(Mat3<double>::Identity(), -compos);
    getWorld2BodyMatrix(body_R_base);
    RigidBodyDynamics::Math::SpatialTransform body_X_base(body_R_base, Vec3<double>::Zero());
    Ag = base_X_com.toMatrixTranspose() * U1 * H;
    Ag.block(0,0,3,18) = (base_X_com.toMatrixTranspose() * body_X_base.toMatrixTranspose() * U1 * H).block(0,0,3,18);
    Ag.block(0,3,3,3) = Mat3<double>::Zero();

    AgdotQdot = base_X_com.toMatrixTranspose() * U1 * Cq;
}

void RobotModel::getComState_dwl(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot, double& total_mass,
                                 Vector3d& com_pos, Vector3d& com_vel, Vector3d& ang_momentum){
    RigidBodyDynamics::Utils::CalcCenterOfMass(*_model, q, qdot, NULL, total_mass, com_pos, &com_vel, NULL, &ang_momentum, NULL, false);
}

//when rpy not equal 0, result is a little fault!!!
void RobotModel::getCentroidInertia_dwl(const Eigen::VectorXd & q, Eigen::MatrixXd & Icent){
    Eigen::MatrixXd joint_inertia_mat;
    //crba's mass matrix is based on Body frame
    dyn_model->getMassInertia(joint_inertia_mat);
    Vector3d com_pos, comvel, angmomen;
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(_model->qdot_size);
    double mass;

    getComState_dwl(q, qd, mass, com_pos, comvel, angmomen);
    RigidBodyDynamics::Math::SpatialTransform base_X_com(Mat3<double>::Identity(), -com_pos);
    Mat3<double> baseTobody;
    getWorld2BodyMatrix(baseTobody);
    RigidBodyDynamics::Math::SpatialTransform body_X_base(baseTobody, Vec3<double>::Zero());
    //cout << "dwl_compos:\n" << com_pos.transpose() << "\n\n";
    //cout << vecCross(com_pos) << "\n\n" << base_X_com.toMatrix()<< "\n\n";
    Eigen::MatrixXd base_inertia_mat = Eigen::MatrixXd::Zero(6,6);
    base_inertia_mat = body_X_base.toMatrixTranspose() * joint_inertia_mat.block(0,0,6,6) * body_X_base.toMatrix();
    Eigen::MatrixXd base_lin_mat = base_inertia_mat.block(0,0,3,6);
    Eigen::MatrixXd base_ang_mat = base_inertia_mat.block(3,0,3,6);
    Eigen::MatrixXd Ibase;
    Ibase.resize(6,6);
    Ibase.block(0,0,3,6) << base_ang_mat.rightCols(3), base_ang_mat.leftCols(3);
    Ibase.block(3,0,3,6) << base_lin_mat.rightCols(3), base_lin_mat.leftCols(3);

    Icent = base_X_com.toMatrixTranspose() * Ibase * base_X_com.toMatrix();
}