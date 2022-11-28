/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "SafetyChecker.h"

/*!
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkSafeOrientation() {
    if (abs(data->_stateEstimator->getResult().rpy(0)) >= 0.8 || abs(data->_stateEstimator->getResult().rpy(1)) >= 0.6) {
        printf("Orientation safety check failed!\n");
        return false;
    } else {
        return true;
    }
}

/*!
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkPDesFoot() {
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  double max_leg_length = data->_model->quadruped->maxLegLength;
  double maxPDes = max_leg_length * sin(data->_model->quadruped->maxAngle);

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Keep the foot from going too far from the body in +x
    if (data->_legController->commands[leg].pDes(0) > maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(0)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->commands[leg].pDes(0) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -x
    if (data->_legController->commands[leg].pDes(0) < -maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(0)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->commands[leg].pDes(0) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in +y
    if (data->_legController->commands[leg].pDes(1) > maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(1)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->commands[leg].pDes(1) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -y
    if (data->_legController->commands[leg].pDes(1) < -maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(1)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->commands[leg].pDes(1) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    if (data->_legController->commands[leg].pDes(2) > - max_leg_length / 4) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(2)
                << " | modified: " << -max_leg_length / 4
                << std::endl;
      data->_legController->commands[leg].pDes(2) = -max_leg_length / 4;
      safePDesFoot = false;
    }

    // Keep the foot within the kinematic limits
    if (data->_legController->commands[leg].pDes(2) < -max_leg_length) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(2)
                << " | modified: " << -max_leg_length
                << std::endl;
      data->_legController->commands[leg].pDes(2) = -max_leg_length;
      safePDesFoot = false;
    }
  }
  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkForceFeedForward() {
    // Assumed safe to start
    bool safeForceFeedForward = true;
    double max_lateral_force = data->_model->quadruped->maxLateralForce;
    double max_vertical_force = data->_model->quadruped->maxVerticalForce;

    // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Limit the lateral forces in +x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) > max_lateral_force) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(0)
                << " | modified: " << max_lateral_force << std::endl;
      data->_legController->commands[leg].forceFeedForward(0) = max_lateral_force;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) < -max_lateral_force) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(0)
                << " | modified: " << -max_lateral_force << std::endl;
      data->_legController->commands[leg].forceFeedForward(0) = -max_lateral_force;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in +y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) > max_lateral_force) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(1)
                << " | modified: " << max_lateral_force << std::endl;
      data->_legController->commands[leg].forceFeedForward(1) = max_lateral_force;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) < -max_lateral_force) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(1)
                << " | modified: " << -max_lateral_force << std::endl;
      data->_legController->commands[leg].forceFeedForward(1) = -max_lateral_force;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in +z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) > max_vertical_force) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(2)
                << " | modified: " << -max_vertical_force << std::endl;
      data->_legController->commands[leg].forceFeedForward(2) = max_vertical_force;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in -z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) < -max_vertical_force) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(2)
                << " | modified: " << max_vertical_force << std::endl;
      data->_legController->commands[leg].forceFeedForward(2) = -max_vertical_force;
      safeForceFeedForward = false;
    }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}