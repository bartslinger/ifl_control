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
class ActiveSetAlgorithm
{
public:
    ActiveSetAlgorithm() :
        _B{},
        _Wv{}
    {

    }

private:

    /**
     * @brief Control effectiveness matrix
     *
     * Column major for more efficient processing
     *
     */
    float _B[4*4];

    /**
     * @brief Weights given to each degree of freedom.
     */
    float _Wv[4];
};
