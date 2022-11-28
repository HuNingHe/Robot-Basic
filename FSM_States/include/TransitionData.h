/*============================= FSM State =============================*/
#ifndef TRANSITIONDATA_H
#define TRANSITIONDATA_H

/*!
 * Struct of relevant data that can be used during transition to pass data between states.
 */
struct TransitionData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TransitionData() { zero(); }

    // Zero out all of the data
    void zero() {
        // Flag to mark when transition is done
        done = false;
        // Timing parameters
        t0 = 0.0;         // time that transition started
        tCurrent = 0.0;   // current time since transition started
        tDuration = 0.0;  // overall transition duration

        // Robot state at the beginning of transition
        comState0 = Vec12<double>::Zero();  // center of mass state
        qJoints0 = Vec12<double>::Zero();   // joint positions
        pFoot0 = Mat34<double>::Zero();     // foot positions

        // Current robot state
        comState = Vec12<double>::Zero();  // center of mass state
        qJoints = Vec12<double>::Zero();   // joint positions
        pFoot = Mat34<double>::Zero();     // foot positions
    }

    // Flag to mark when transition is done
    bool done = false;

    // Timing parameters
    double t0;         // time that transition started
    double tCurrent;   // current time since transition started
    double tDuration;  // overall transition duration

    // Robot state at the beginning of transition
    Vec12<double> comState0;  // center of mass state
    Vec12<double> qJoints0;   // joint positions
    Mat34<double> pFoot0;     // foot positions

    // Current robot state
    Vec12<double> comState;  // center of mass state
    Vec12<double> qJoints;   // joint positions
    Mat34<double> pFoot;     // foot positions
};

#endif  // CONTROLFSM_H