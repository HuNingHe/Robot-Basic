#include "Kinematic.h"
#include "Quadruped.h"
#include "Orientation.h"
#include <iostream>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

SingleLegKinematic::SingleLegKinematic(Vec3<double> link_length, bool is_elbow): _is_elbow(is_elbow){
    assert(link_length[0] >= 0 && link_length[1] > 0 && link_length[2] > 0);
    _link_length = std::move(link_length);
}

void SingleLegKinematic::Forward(Vec3<double> theta, Vec3<double>& pos){
    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double c2 = cos(theta[1]);
    double s2 = sin(theta[1]);
    double c23 = cos(theta[1] + theta[2]);
    double s23 = sin(theta[1] + theta[2]);

    pos[0] = -_link_length[2] * s23 - _link_length[1] * s2;
    pos[1] = _link_length[2] * c23 * s1 + _link_length[1] * c2 * s1 + _link_length[0] * s1;
    pos[2] = -_link_length[2] * c23 * c1 - _link_length[1] * c2 * c1 - _link_length[0] * c1;
}

void SingleLegKinematic::Inverse(Vec3<double> p, Vec3<double> &theta){
    theta[0] = atan2(p[1], -p[2]);
    double tmp1 = p[1] * sin(theta[0]) - p[2] * cos(theta[0]) - _link_length[0];
    double tmp2 = pow(p[0], 2) + pow(tmp1, 2);
    theta[2] = acos((tmp2 - pow(_link_length[1], 2) - pow(_link_length[2], 2)) / (2 * _link_length[1] * _link_length[2]));
    // The range of function acos is [0, pi]. Therefore, theta3 should be negative for elbow-configured leg
    if (_is_elbow){
        theta[2] *= -1;
    }
    theta[1] = asin(-sin(theta[2]) * _link_length[2] / sqrt(tmp2)) - atan2(p[0], tmp1);
}

void SingleLegKinematic::Jacobi(Vec3<double> theta, Mat3<double> &jacobi){
    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double c2 = cos(theta[1]);
    double s2 = sin(theta[1]);
    double c23 = cos(theta[1] + theta[2]);
    double s23 = sin(theta[1] + theta[2]);

    jacobi(0, 0) = 0;
    jacobi(0, 1) = -_link_length[2] * c23 - _link_length[1] * c2;
    jacobi(0, 2) = -_link_length[2] * c23;

    jacobi(1, 0) = _link_length[2] * c23 * c1 + _link_length[1] * c2 * c1 + _link_length[0] * c1;
    jacobi(1, 1) = -_link_length[2] * s23 * s1 - _link_length[1] * s2 * s1;
    jacobi(1, 2) = -_link_length[2] * s23 * s1;

    jacobi(2, 0) = _link_length[2] * c23 * s1 + _link_length[1] * c2 * s1 + _link_length[0] * s1;
    jacobi(2, 1) = _link_length[2] * s23 * c1 + _link_length[1] * s2 * c1;
    jacobi(2, 2) = _link_length[2] * s23 * c1;
}

const char* link_name[] ={
    "Trunk",
    "LF_Hip","LF_Thigh","LF_Rotor","LF_Link","LF_Leg",
    "RF_Hip","RF_Thigh","RF_Rotor","RF_Link","RF_Leg",
    "RH_Hip","RH_Thigh","RH_Rotor","RH_Link","RH_Leg",
    "LH_Hip","LH_Thigh","LH_Rotor","LH_Link","LH_Leg"
};

KinematicModel::KinematicModel(Model* model){
    _model = model;
    Ig = Eigen::MatrixXd::Zero(6, 6);
    Jg = Eigen::MatrixXd::Zero(6, _model->qdot_size);
    Ag = Eigen::MatrixXd::Zero(6, _model->qdot_size);
}

void KinematicModel::UpdateKinematics(const VectorNd & q, const VectorNd & qdot){
    UpdateCentroidFrame(q, qdot);
}

void KinematicModel::UpdateCentroidFrame(const VectorNd & q, const VectorNd & qdot){
    double mass;
    Vector3d link_pos;
    getCoMPos(_com_pos);
    // transformation from link(i) frame to com frame
    Eigen::MatrixXd link_X_com = Eigen::MatrixXd::Zero(6, 6);
    Ig.setZero();
    Ag.setZero();
    // body(i)'s spatial inertial represent in body frame
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(6, 6);
    // spatial jacobi matrix
    Eigen::MatrixXd Jsp = Eigen::MatrixXd::Zero(6, _model->qdot_size);

    unsigned int start_idx = find_body_id(robot_link::Trunk);
    /*!
     * cmm: cross matrix of each body's com vector in parent frame
     * R: rotation matrix of each body in base frame
     */
    Matrix3d cmm, R;

    for (unsigned int i(start_idx); i<_model->mBodies.size(); ++i){
        R = CalcBodyWorldOrientation(*_model, q, i, false);
        link_pos = CalcBodyToBaseCoordinates ( *_model, q, i, Vector3d::Zero(), false);

        Jsp.setZero();
        CalcBodySpatialJacobian( *_model, q, i, Jsp, false);

        mass = _model->mBodies[i].mMass;
        I.setZero();
        cmm = Ori::crossMatrix(_model->mBodies[i].mCenterOfMass);
        I.block(0, 0, 3, 3) = _model->mBodies[i].mInertia + mass * cmm * cmm.transpose();
        I.block(0,3, 3,3) = mass * cmm;
        I.block(3,0, 3,3) = mass * cmm.transpose();
        I.block(3, 3, 3, 3) = mass * Eigen::MatrixXd::Identity(3,3);

        link_X_com.block(0,0,3,3) = R;
        link_X_com.block(3,3,3,3) = R;
        link_X_com.block(3,0,3,3) = -R * Ori::crossMatrix(link_pos - _com_pos);
        // sum up all inertial in com frame
        Ig += link_X_com.transpose() * I * link_X_com;
        //xyz rpy joint
        Ag += link_X_com.transpose() * I * Jsp;
    }

    // exchange left and right columns, if rpy xyz joint
    // Eigen::MatrixXd lin_mat = Ag.block(0,0,3,6);
    // Eigen::MatrixXd ang_mat = Ag.block(3,0,3,6);
    // Ag.block(0,0,3,6) << lin_mat.rightCols(3), lin_mat.leftCols(3);
    // Ag.block(3,0,3,6) << ang_mat.rightCols(3), ang_mat.leftCols(3);
    Jg = Ig.inverse() * Ag;
    _centroid_vel = Jg * qdot;
}

void KinematicModel::getCoMJacobi(Eigen::MatrixXd & Jcom) const{
    VectorNd q(_model->q_size);

    Jcom = Eigen::MatrixXd::Zero(3, _model->qdot_size);
    Eigen::MatrixXd J(3, _model->qdot_size);

    double mass;
    double tot_mass(0.0);
    unsigned int start_idx = find_body_id(robot_link::Trunk);

    for (unsigned int i(start_idx); i < _model->mBodies.size() ; ++i){
        mass = _model->mBodies[i].mMass;
        // CoM Jacobi Update
        J.setZero();
        CalcPointJacobian(*_model, q, i, _model->mBodies[i].mCenterOfMass, J, false);
        Jcom +=  mass * J;
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}

void KinematicModel::getCoMPos(Vec3<double> & CoM_pos)const{
    VectorNd q(_model->q_size);
    CoM_pos.setZero();
    Vec3<double> link_pos;

    unsigned int start_idx = find_body_id(robot_link::Trunk);
    double mass;
    double tot_mass(0.0);
    for (unsigned int i(start_idx); i< _model->mBodies.size() ; ++i){
        mass = _model->mBodies[i].mMass;
        // CoM position Update
        link_pos = CalcBodyToBaseCoordinates ( *_model, q, i, _model->mBodies[i].mCenterOfMass, false);
        CoM_pos += mass * link_pos;
        tot_mass += mass;
    }
    CoM_pos /= tot_mass;
}

void KinematicModel::getCoMVel(Vec3<double> & CoM_vel) const{
    VectorNd q, qdot;
    unsigned int start_idx = find_body_id(robot_link::Trunk);
    CoM_vel.setZero();
    Vec3<double> link_vel;

    double mass;
    double tot_mass(0.0);
    for (unsigned int i(start_idx); i< _model->mBodies.size() ; ++i){
        mass = _model->mBodies[i].mMass;
        // CoM velocity Update
        link_vel = CalcPointVelocity (*_model, q, qdot, i, _model->mBodies[i].mCenterOfMass, false);
        CoM_vel += mass * link_vel;
        tot_mass += mass;
    }
    CoM_vel /= tot_mass;
}

void KinematicModel::getPos(int link_id, Vec3<double> & pos){
    Vec3<double> zero;
    Matrix3d R;
    VectorNd q(_model->q_size);

    unsigned int body_id = find_body_id(link_id);
    if(body_id >=_model->fixed_body_discriminator){
        zero = _model->mFixedBodies[body_id - _model->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  _model->mBodies[body_id].mCenterOfMass;
    }

    pos = CalcBodyToBaseCoordinates(*_model, q, find_body_id(link_id), zero, false);
}

void KinematicModel::getOri(int link_id, Eigen::Quaternion<double> & quat){
    Matrix3d R;
    VectorNd q(_model->q_size);
    R = CalcBodyWorldOrientation( *_model, q, find_body_id(link_id), false);
    quat = R.transpose();
    // 如果转角超过180度, 认为反转了多少度
    if(quat.w() < 0.){
        quat.w() *= (-1.);
        quat.x() *= (-1.);
        quat.y() *= (-1.);
        quat.z() *= (-1.);
    }
    quat.normalize();
}

void KinematicModel::getLinearVel(int link_id, Vec3<double> & vel){
    Vec3<double> zero;
    VectorNd q, qdot;

    unsigned int body_id = find_body_id(link_id);
    if(body_id >=_model->fixed_body_discriminator){
        zero = _model->mFixedBodies[body_id - _model->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  _model->mBodies[body_id].mCenterOfMass;
    }

    vel = CalcPointVelocity ( *_model, q, qdot, find_body_id(link_id), zero, false);
}

void KinematicModel::getAngularVel(int link_id, Vec3<double> & ang_vel){
    unsigned int body_id = find_body_id(link_id);
    VectorNd vel, q, qdot;

    if(body_id >=_model->fixed_body_discriminator){
        vel = CalcPointVelocity6D(*_model, q, qdot, body_id, _model->mFixedBodies[body_id - _model->fixed_body_discriminator].mCenterOfMass, false);
    }else{
        vel = CalcPointVelocity6D(*_model, q, qdot, body_id, _model->mBodies[body_id].mCenterOfMass, false);
    }
    ang_vel = vel.head(3);
}

void KinematicModel::getJacobi(int link_id, Eigen::MatrixXd &J){
    VectorNd q(_model->q_size);
    J = Eigen::MatrixXd::Zero(6, _model->qdot_size);
    unsigned int body_id = find_body_id(link_id);

    if(body_id >=_model->fixed_body_discriminator) {
        CalcPointJacobian6D(*_model, q, body_id, _model->mFixedBodies
                            [body_id - _model->fixed_body_discriminator].mCenterOfMass, J, false);
    } else {
        CalcPointJacobian6D(*_model, q, body_id, _model->mBodies[body_id].mCenterOfMass, J, false);
    }
    /*************this will transform the jacobian from world frame to body frame********/
    // easy to understand
    // Eigen::Matrix<double, 6, 6> World2Body6D;
    // Eigen::Matrix3d World2Body;
    // World2Body6D.setZero(); World2Body.setZero();
    // World2Body = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    // World2Body6D.block(0,0,3,3) = World2Body;
    // World2Body6D.block(3,3,3,3) = World2Body;
    // J = World2Body6D * J;
}

void KinematicModel::getJDotQdot(int link_id, VectorNd & JDotQdot){
    VectorNd q, qdot, qddot;
    unsigned int body_id = find_body_id(link_id);
    if(body_id >=_model->fixed_body_discriminator){
        JDotQdot = CalcPointAcceleration6D(*_model, q, qdot, qddot, body_id,
                                           _model->mFixedBodies
                                           [body_id - _model->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        JDotQdot = CalcPointAcceleration6D(*_model, q, qdot, qddot, body_id,
                                           _model->mBodies[body_id].mCenterOfMass, false);
    }
    // If this function is called after ForwardDynamics() without an update of the kinematic state,
    // gravity acceleration has to be added to the result.
    // 此处因为假设关节角加速度为零求取而来, 原代码在加速度计算中忽略了重力加速度
    JDotQdot[5] += _model->gravity[2];
}

void KinematicModel::getWorldToBodyMatrix(Mat3<double> & BodyMatrix){
    VectorNd q;
    unsigned int body_idx = find_body_id(robot_link::Trunk);
    BodyMatrix = CalcBodyWorldOrientation(*_model, q, body_idx, false);
}

unsigned int KinematicModel::find_body_id(int id) const{
    switch(id){
        case robot_link::Trunk:
            return find_body_id("Trunk");
        case robot_link::LF_Foot:
            return find_body_id("LF_Foot");
        case robot_link::RF_Foot:
            return find_body_id("RF_Foot");
        case robot_link::RH_Foot:
            return find_body_id("RH_Foot");
        case robot_link::LH_Foot:
            return find_body_id("LH_Foot");
        default:
            std::cout << "Unknown id" << std::endl;
            break;
    }
    return 0;
}

unsigned int KinematicModel::find_body_id(const char* _link_name) const{
    unsigned int body_id;
    body_id = _model->GetBodyId(_link_name);
    return body_id;
}

//according to this function, we know the number of each joint
void KinematicModel::DisplayLinks() {
    int link_list_len;
    link_list_len = sizeof(link_name)/sizeof(link_name[0]);
    std::cout << "link_num: " << link_list_len << std::endl;
    std::vector<const char*> link_name_list(link_name, link_name + link_list_len);
    std::vector<unsigned int> body_id_list;
    unsigned int body_id;
    for (auto & i : link_name_list) {
        body_id = _model->GetBodyId(i);
        body_id_list.push_back(body_id);
    }
    for (long unsigned i(0); i < body_id_list.size(); ++i) {
        std::cout << link_name_list[i] << ": " << body_id_list[i] << std::endl;
    }
}
