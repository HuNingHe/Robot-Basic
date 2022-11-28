#include "Dynamic.h"
#include "LinearAlgebra.h"

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

DynamicModel::DynamicModel(RigidBodyDynamics::Model* model){
    _model = model;
    H = MatrixNd::Zero(_model->qdot_size, _model->qdot_size);
    G = VectorNd::Zero(_model->qdot_size);
    C = VectorNd::Zero(_model->qdot_size);
}

void DynamicModel::getMassInertia(Eigen::MatrixXd & h){
    h = H;
}

void DynamicModel::getInverseMassInertia(Eigen::MatrixXd & hinv){
    hinv = Hinv;
}

void DynamicModel::getGravity(Eigen::VectorXd & grav){
    grav = G;
}

void DynamicModel::getCoriolis(Eigen::VectorXd & coriolis){
    coriolis = C;
}

void DynamicModel::UpdateDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot){
    // Mass Matrix
    CompositeRigidBodyAlgorithm(*_model, q, H, false);
    pseudoInverse(H, Hinv);
    // Coriolis
    NonlinearEffects(*_model, q, qdot, C);
    // Gravity
    InverseDynamics(*_model, q, Eigen::VectorXd::Zero(_model->qdot_size), Eigen::VectorXd::Zero(_model->qdot_size), G);
    // InverseDynamics(*_model, q, qdot, VectorNd::Zero(_model->qdot_size), C);
    C -= G;
}
