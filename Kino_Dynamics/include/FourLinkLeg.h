/*!
 * @file FourLinkLeg.h
 * @brief This file contains motion analysis of four linked leg, all angular units being radians
 * @date 2022-5-1
 * @author HuNing-He
 */

#ifndef MPC_FOUR_LINK_LEG_H
#define MPC_FOUR_LINK_LEG_H
#include "MathTypes.h"

class FourLinkLeg{
private:
    double _knee_bias;
    Vec4<double> _link_length;
    Vec3<double> _theta;
public:
    double _motor_correction;
    /*!
     *@param link_length: 4 x 1 array, represent the four link length, refer to paper: 四足机器人腿部结构运动学分析与仿真, 李栓成等
     *@param knee_bias: a fixed bias between link and actual knee angle(default to zero)
     *@param motor_correction: init angle of motor, defined by exported 3D model, Webots regard this position as zero position,
            but it's not the actual zero position, so that we need to make some corrections(default to zero)
     */
    FourLinkLeg(Vec4<double> link_length, double motor_correction = 0.0, double knee_bias = 0.0);

    /*!
     * @brief get the link thetas for closed loop dynamics calculation
     * @param theta: actual motor position
     * @return: all thetas of open-loop
     */
    Vec3<double>& UpdateTheta(double theta);

    /*!
     * @param theta: actual motor position
     * @return: desire knee position, should be negative in elbow-configured leg,
        which depends on the joint coordinate when building kinematics
     */
    double Motor2Knee(double theta);

    /*!
     * @brief: According to the paper, this can be easily done by exchanging l1 and l3 link
     * @param theta: desire knee angle(negative in knee-configured leg), should first convert to the third link's angle
     * @return: desire motor angle
     */
    double Knee2Motor(double theta);

    /*!
     * @brief: we do this by polynomial curve fitting(already done in python)
     * @param theta: actual motor position
     * @param order: Polynomial fitting order
     * @return: the reduction ratio from motor to knee joint
     */
    double ReductionRatio(double theta, int order = 4) const;

    /*!
     * @brief: we do this by polynomial curve fitting(already done in python)
     * @param theta: actual motor position
     * @param order: Polynomial fitting order
     * @return: differentiate theta2 about theta1
     */
    double Theta2Diff(double theta, int order = 3) const;

    /*!
     * @brief: we do this by polynomial curve fitting(already done in python)
     * @param theta: actual motor position
     * @return: differentiate theta3 about theta1
     */
    double Theta3Diff(double theta) const;
};

#endif //MPC_FOUR_LINK_LEG_H
