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

    int setActuatorEffectiveness(const float B[]) {
        // Is provided row-major. Convert to column major

        return 0;
    }

    int setOutputWeights(const float Wv[]) {
        return 0;
    }

    int setActuatorUpperLimit(const float u_up[]) {
        return 0;
    }

    int setActuatorLowerLimit(const float u_lo[]) {
        return 0;
    }

    int calculateActuatorCommands(const float v[], float out[]) {
        out[0] = 123.0f;
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
};
