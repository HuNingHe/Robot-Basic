/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATE_ESTIMATOR_H
#define PROJECT_STATE_ESTIMATOR_H

#include "LegController.h"
#include "IMUTypes.h"
#include "Quadruped.h"
/*!
 * Result of state estimation
 */
struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec4<double> contactEstimate;
    Vec3<double> position;
    Vec3<double> vBody;
    Quat<double> orientation;
    Vec3<double> omegaBody;
    RotMat<double> rBody;
    Vec3<double> rpy;

    Vec3<double> omegaWorld;
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
struct StateEstimatorData {
    StateEstimate* result;  // where to write the output to
    VectorNavData* vectorNavData;
    LegControllerData* legControllerData;
    Vec4<double>* contactPhase;
    SensorNoise* parameters;
    Quadruped *quadruped;
};

/*!
 * All Estimators should inherit from this class
 */
class GenericEstimator {
public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData data) { _stateEstimatorData = data; }

    virtual ~GenericEstimator() = default;
    StateEstimatorData _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
class StateEstimatorContainer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * Construct a new state estimator container
     */
    StateEstimatorContainer(VectorNavData* vectorNavData,
                            LegControllerData* legControllerData,
                            StateEstimate* stateEstimate,
                            SensorNoise* parameters,
                            Quadruped *quad) {
        _data.vectorNavData = vectorNavData;
        _data.legControllerData = legControllerData;
        _data.result = stateEstimate;
        _phase = Vec4<double>::Zero();
        _data.contactPhase = &_phase;
        _data.parameters = parameters;
        _data.quadruped = quad;
    }

    /*!
     * Run all estimators
     */
    void run() {
        for (auto estimator : _estimators) {
          estimator->run();
        }
    }

    /*!
     * Get the result
     */
    const StateEstimate& getResult() const { return *_data.result; }
    StateEstimate * getResultHandle() const { return _data.result; }

    /*!
     * Set the contact phase
     */
    void setContactPhase(Vec4<double>& phase) {
        *_data.contactPhase = phase;
    }

    /*!
     * Add an estimator of the given type
     * @tparam EstimatorToAdd
     */
    template <typename EstimatorToAdd>
    void addEstimator() {
        auto* estimator = new EstimatorToAdd();
        estimator->setData(_data);
        estimator->setup();
        _estimators.push_back(estimator);
    }

    /*!
     * Remove all estimators of a given type
     * @tparam EstimatorToRemove
     */
    template <typename EstimatorToRemove>
    void removeEstimator() {
        int nRemoved = 0;
        _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator* e) {
                         if (dynamic_cast<EstimatorToRemove*>(e)) {
                           delete e;
                           nRemoved++;
                           return true;
                         } else {
                           return false;
                         }
                       }),
        _estimators.end());
    }

    /*!
     * Remove all estimators
     */
    void removeAllEstimators() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
        _estimators.clear();
    }

    ~StateEstimatorContainer() {
        if (!_estimators.empty()){
            for (auto estimator : _estimators) {
                delete estimator;
            }
        }
    }

private:
    StateEstimatorData _data;
    std::vector<GenericEstimator*> _estimators;
    Vec4<double> _phase;
};

#endif  // PROJECT_STATE_ESTIMATOR_H
