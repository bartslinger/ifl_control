/**
 * @file ActiveSetAlgorithm.hpp
 *
 * This implements the Efficient Active Set Algorithm.
 *
 * More explanation to come.
 *
 * @author Bart Slinger <bartslinger@gmail.com>
 */

#pragma once

#include "LeastSquaresSolver.hpp"

namespace ifl_control {

/**
 * @brief The ActiveSetAlgorithm class
 *
 * The algorithm gets initialized with a certain effectiveness matrix A.
 * A weighting matrix can be supplied to favor certain outputs over others.
 * Inputs for a calculation are the desired outputs, and the upper and lower
 * bounds available from the actuator.
 * The output is a vector containing the control commands.
 */
template<size_t M, size_t N>
class ActiveSetAlgorithm
{
public:
    ActiveSetAlgorithm() :
        _B{},
        _Wv{}
    {

    }

    int setActuatorEffectiveness(const float B_row_major[]) {
        // Is provided row-major. Convert to column major
        for (size_t i = 0; i < M*N; i++) {
            _B[i] = B_row_major[(i%M)*N + (i/M)];
            //printf("%lu\t%1.2f\n", i, _B[i]);
        }
        return 0;
    }

    int setOutputWeights(const float Wv[]) {
        for (size_t i = 0; i < M; i++) {
            _Wv[i] = Wv[i];
        }
        return 0;
    }

    int setActuatorUpperLimit(const float u_up[]) {
        for (size_t i = 0; i < M; i++) {
            _u_up[i] = u_up[i];
        }
        return 0;
    }

    int setActuatorLowerLimit(const float u_lo[]) {
        for (size_t i = 0; i < M; i++) {
            _u_lo[i] = _u_lo[i];
        }
        return 0;
    }

    int calculateActuatorCommands(const float v[], float out[]) {
        LeastSquaresSolver solver;
        float tau[M];
        float w[M];

        solver.setMatrix(_B, tau, w, M, N);

        solver.solve(v, out);
        return 0;
    }

private:

    /**
     * @brief Control effectiveness matrix
     *
     * Column major for more efficient processing
     *
     */
    float _B[M*N];

    /**
     * @brief Weights given to each degree of freedom.
     */
    float _Wv[M];

    float _u_up[M];
    float _u_lo[M];
};

} // namespace ifl_control
