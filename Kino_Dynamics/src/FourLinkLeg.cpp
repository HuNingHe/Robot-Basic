//
// Created by hun on 22-5-9.
//
#include "FourLinkLeg.h"

FourLinkLeg::FourLinkLeg(Vec4<double> link_length, double motor_correction, double knee_bias) : _knee_bias(knee_bias), _motor_correction(motor_correction){
    for (int i = 0; i < 4; ++i) {
        assert(link_length[i] > 0);
    }
    _link_length = std::move(link_length);
}

Vec3<double>& FourLinkLeg::UpdateTheta(double theta){
    theta += _motor_correction;
    double a = 2 * _link_length[0] * _link_length[2] * sin(theta);
    double b = 2 * _link_length[2] * (_link_length[0] * cos(theta) - _link_length[3]);
    double c = pow(_link_length[1], 2) - pow(_link_length[0], 2) - pow(_link_length[2], 2) - pow(_link_length[3], 2) + 2 * _link_length[0] * _link_length[3] * cos(theta);
    _theta[0] = theta;
    double theta4 = asin(-c / sqrt(pow(a, 2) + pow(b, 2))) - atan2(b, a);
    _theta[1] = asin((_link_length[2] * sin(theta4) - _link_length[0] * sin(theta)) / _link_length[1]) - theta;
    _theta[2] = theta4 - M_PI - theta - _theta[1];
    return  _theta;
}

double FourLinkLeg::Motor2Knee(double theta) {
    theta += _motor_correction;
    double a = 2 * _link_length[0] * _link_length[2] * sin(theta);
    double b = 2 * _link_length[2] * (_link_length[0] * cos(theta) - _link_length[3]);
    double c = pow(_link_length[1], 2) - pow(_link_length[0], 2) - pow(_link_length[2], 2) - pow(_link_length[3], 2) + 2 * _link_length[0] * _link_length[3] * cos(theta);
    return asin(-c / sqrt(pow(a, 2) + pow(b, 2))) - atan2(b, a) - _knee_bias - M_PI;
}

double FourLinkLeg::Knee2Motor(double theta) {
    theta = M_PI + theta + _knee_bias;
    double a = 2 * _link_length[0] * _link_length[2] * sin(theta);
    double b = -2 * _link_length[0] * (_link_length[2] * cos(theta) + _link_length[3]);
    double c = pow(_link_length[1], 2) - pow(_link_length[0], 2) - pow(_link_length[2], 2) - pow(_link_length[3], 2) - 2 * _link_length[2] * _link_length[3] * cos(theta);
    return M_PI - (asin(-c / sqrt(pow(a, 2) + pow(b, 2))) - atan2(b, a)) - _motor_correction;
}

double FourLinkLeg::ReductionRatio(double theta, int order) const {
    theta += _motor_correction;
    switch (order) {
        case 2:
            return -0.24889879 * pow(theta, 2) + 0.79389053 * theta + 0.03116565;
        case 3:
            return 0.0260247 * pow(theta, 3) - 0.36927345 * pow(theta, 2) + 0.9533875 * theta - 0.02398126;
        default:
            return -0.03899938 * pow(theta, 4) + 0.26654165 * pow(theta, 3) - 0.86966069 * pow(theta, 2) + 1.3528932 * theta - 0.12091342;
    }
}

double FourLinkLeg::Theta2Diff(double theta, int order) const {
    theta += _motor_correction;
    if (order == 2) {
        return -0.00218291 * pow(theta, 2) + 0.04170418 * theta - 1.06273506;
    } else{
        return 0.00983792 * pow(theta, 3) - 0.04768722 * pow(theta, 2) + 0.10199759 * theta - 1.08358182;
    }
}

double FourLinkLeg::Theta3Diff(double theta) const {
    return ReductionRatio(theta) - Theta2Diff(theta) - 1;
}