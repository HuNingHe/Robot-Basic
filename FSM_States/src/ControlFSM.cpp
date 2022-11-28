/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 * @param model kino-dynamic model of robot
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
ControlFSM::ControlFSM(RobotModel *model,
                       StateEstimatorContainer *_stateEstimator,
                       LegController *_legController,
                       GaitController *_gaitScheduler,
                       GamepadCommand *_desiredStateCommand,
                       RobotControlParameters *controlParameters) {
    // Add the pointers to the ControlFSMData struct
    data._model = model;
    data._stateEstimator = _stateEstimator;
    data._legController = _legController;
    data._gaitScheduler = _gaitScheduler;
    data._desiredStateCommand = _desiredStateCommand;
    data.controlParameters = controlParameters;

    // Initialize and add all of the FSM States to the state list
    statesList.invalid = nullptr;
    statesList.passive = new FSM_State_Passive(&data);
    statesList.standUp = new FSM_State_StandUp(&data);
    statesList.balanceStand = new FSM_State_BalanceStand(&data);
    statesList.locomotion = new FSM_State_Locomotion(&data);
    statesList.recoveryStand = new FSM_State_RecoveryStand(&data);
    statesList.recoverySitDown = new FSM_State_RecoverySitDown(&data);

    safetyChecker = new SafetyChecker(&data);

    // Initialize the FSM with the Passive FSM State
    initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
void ControlFSM::initialize() {
    // Initialize a new FSM State with the control data
    currentState = statesList.passive;
//    printf("%d", currentState->stateName);

    // Enter the new current state cleanly
    currentState->onEnter();

    // Initialize to not be in transition
    nextState = currentState;

    // Initialize FSM mode to normal operation
    operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
void ControlFSM::runFSM() {
    operatingMode = safetyPreCheck();
    
    unsigned int mode = data._desiredStateCommand->mode;

    switch (mode) {
        case K_PASSIVE:
            data.controlParameters->control_mode = K_PASSIVE;
            break;
        case K_STAND_UP:
            data.controlParameters->control_mode = K_STAND_UP;
            break;
        case K_SIT_DOWN:
            data.controlParameters->control_mode = K_SIT_DOWN;
            break;
        case K_BALANCE_STAND:
            data.controlParameters->control_mode = K_BALANCE_STAND;
            break;
        case K_LOCOMOTION:
            data.controlParameters->control_mode = K_LOCOMOTION;
            break;
        case K_RECOVERY_STAND:
            data.controlParameters->control_mode = K_RECOVERY_STAND;
            break;
        default:
            break;
    }

    // Run the robot control code if operating mode is not unsafe
    if (operatingMode != FSM_OperatingMode::ESTOP) {
        // Run normal controls if no transition is detected
        if (operatingMode == FSM_OperatingMode::NORMAL) {
            // Check the current state for any transition
            nextStateName = currentState->checkTransition();

            // Detect a commanded transition
            if (nextStateName != currentState->stateName) {
                // Set the FSM operating mode to transitioning
                operatingMode = FSM_OperatingMode::TRANSITIONING;

                // Get the next FSM State by name
                nextState = getNextState(nextStateName);

            } else {
                // Run the iteration for the current state normally
                currentState->run();
            }
        }

        // Run the transition code while transition is occuring
        if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
            transitionData = currentState->transition();

            // Check the robot state for safe operation
            safetyPostCheck();

            // Run the state transition
            if (transitionData.done) {
                // Exit the current state cleanly
                currentState->onExit();

                // Complete the transition
                currentState = nextState;

                // Enter the new current state cleanly
                currentState->onEnter();

                // Return the FSM to normal operation mode
                operatingMode = FSM_OperatingMode::NORMAL;
            }
        } else {
            // Check the robot state for safe operation
            safetyPostCheck();
        }

    } else { // if ESTOP
        currentState = statesList.passive; // 原本为passive
        currentState->onEnter();
        nextStateName = currentState->stateName;
    }
    // Increase the iteration counter
    iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
FSM_OperatingMode ControlFSM::safetyPreCheck() {
    // Check for safe orientation if the current state requires it
    if (currentState->checkSafeOrientation && data.controlParameters->control_mode != K_RECOVERY_STAND) {
        if (!safetyChecker->checkSafeOrientation()) {
            operatingMode = FSM_OperatingMode::ESTOP;
            std::cout << "broken: Orientation Safety Check FAIL" << std::endl;
        }
    }
    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
FSM_OperatingMode ControlFSM::safetyPostCheck() {
    // Check for safe desired foot positions
    if (currentState->checkPDesFoot) {
        safetyChecker->checkPDesFoot();
    }

    // Check for safe desired feedforward forces
    if (currentState->checkForceFeedForward) {
        safetyChecker->checkForceFeedForward();
    }

    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 *
 * @param  next commanded enumerated state name
 * @return next FSM state
 */
FSM_State *ControlFSM::getNextState(FSM_StateName stateName) {
    // Choose the correct FSM State by enumerated state name
    switch (stateName) {
        case FSM_StateName::INVALID:
            return statesList.invalid;

        case FSM_StateName::PASSIVE:
            return statesList.passive;

        case FSM_StateName::STAND_UP:
            return statesList.standUp;

        case FSM_StateName::BALANCE_STAND:
            return statesList.balanceStand;

        case FSM_StateName::LOCOMOTION:
            return statesList.locomotion;

        case FSM_StateName::RECOVERY_STAND:
            return statesList.recoveryStand;

        case FSM_StateName::SIT_DOWN:
            return statesList.recoverySitDown;

        default:
            return statesList.invalid;
    }
}

ControlFSM::~ControlFSM() {
    delete safetyChecker;
//    delete statesList.passive;
//    delete statesList.standUp;
//    delete statesList.balanceStand;
//    delete statesList.locomotion;
//    delete statesList.recoveryStand;
//    delete statesList.recoverySitDown;
}
