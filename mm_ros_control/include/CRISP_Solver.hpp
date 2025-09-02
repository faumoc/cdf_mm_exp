#ifndef MOBILE_MANIPULATOR_PROBLEM_H
#define MOBILE_MANIPULATOR_PROBLEM_H

#include <solver_core/SolverInterface.h>
#include <memory>
#include <string>

using namespace CRISP;

// Constants for the problem
const size_t N = 20;                // Number of time steps
const size_t num_state = 9;         // Number of state variables
const size_t num_control = 9;       // Number of control variables
const scalar_t dt = 0.1;            // Time step duration

/**
 * @brief Dynamics constraints for the mobile manipulator.
 * @param x The optimization variables.
 * @param y The output constraints.
 */
extern ad_function_t mobileManipulatorDynamicConstraints;

/**
 * @brief Initial constraints for the mobile manipulator.
 * @param x The optimization variables.
 * @param p The parameters (initial state and control values).
 * @param y The output constraints.
 */
extern ad_function_with_param_t mobileManipulatorInitialConstraints;

/**
 * @brief Joint limit constraints for the mobile manipulator.
 * @param x The optimization variables.
 * @param p The parameters (joint limits).
 * @param y The output constraints.
 */
extern ad_function_with_param_t mobileManipulatorJointLimits;

/**
 * @brief Objective function for the mobile manipulator.
 * @param x The optimization variables.
 * @param p The parameters (tracking references).
 * @param y The output cost value.
 */
extern ad_function_with_param_t mobileManipulatorObjective;

/**
 * @brief Creates an optimization problem for the mobile manipulator.
 * @return The constructed optimization problem.
 */
OptimizationProblem makeMobileManipulatorProblem(std::string libFolder);
SolverInterface initializeSolver(OptimizationProblem& problem, SolverParameters& params, vector_t& x0, vector_t& xf, vector_t& jointLimits);


#endif // MOBILE_MANIPULATOR_PROBLEM_H