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
        applyWeights();
        return 0;
    }

    int setOutputWeights(const float Wv[]) {
        for (size_t i = 0; i < M; i++) {
            _Wv[i] = Wv[i];
        }
        applyWeights();
        return 0;
    }

    int setActuatorUpperLimit(const float u_up[]) {
        for (size_t i = 0; i < N; i++) {
            _u_up[i] = u_up[i];
        }
        return 0;
    }

    int setActuatorLowerLimit(const float u_lo[]) {
        for (size_t i = 0; i < N; i++) {
            _u_lo[i] = u_lo[i];
        }
        return 0;
    }

    int calculateActuatorCommands(const float v[], float u_k[]) {

        checkActuatorLimits();
        printf("\n");
        // multiply virtual control with weights to get b
        for (size_t i = 0; i < M; i++) {
            _b[i] = v[i] * _Wv[i];
        }

        for (int i = 0; i < 10; i++) {
            if (runIteration(u_k) == 0) {
                // optimal solution found
                break;
            }
        }

        return 0;
    }

private:

    int runIteration(float u_k[]) {

        // make sure the initial solution is within bounds
        for (size_t j = 0; j < N; j++) {
            if (u_k[j] > _u_up[j]) {
                u_k[j] = _u_up[j];
            }
            if (u_k[j] < _u_lo[j]) {
                u_k[j] = _u_lo[j];
            }
        }

        float p[M] = {0};

        // Construct Af
        size_t k = 0;
        for (size_t j = 0; j < N; j++) {
            if (_W[j] == 0) {
                // this actuator is not free, add to Af
                for (size_t i = 0; i < M; i++) {
                    _A_f[k*M + i] = _A[j*M + i];
                }
                k++;
            }
        }

        //printf("k: %lu\n", k);

        // If there is more than one free actuator
        if (k > 0) {
            // construct d = b - A*u_k
            float d[M] = {0};
            // d = b
            for (size_t i = 0; i < M; i++) {
                d[i] = _b[i];
            }
            // d -= A*u_k
            for (size_t l = 0; l < M*N; l++) {
                d[l%M] -= _A[l] * u_k[l/M];
            }

            for (size_t i = 0; i < M; i++) {
                //printf("d[%lu] = %1.15f\n", i, d[i]);
            }

            // perturbation of free actuators from least squares solver
            float pp[N] = {0}; // first k are filled, max N

            // working space for solver
            float tau[M] = {0};
            float w[M] = {0};

            _solver.setMatrix(_A_f, tau, w, M, N);
            _solver.solve(d, pp);

            // Construct full perturbation, including constrained ones
            size_t z = 0;
            for (size_t j = 0; j < N; j++) {
                if (_W[j] == 0) {
                    p[j] = pp[z];
                    z++;
                }
                //printf("p[%lu] = %1.5f\n", j, p[j]);
            }
        } else {
            printf("no free actuators\n");
        }

        float smallest_alpha = 1.0f;
        size_t smallest_alpha_idx = 0;

        // iterate through free actuators, check solution feasibility
        for (size_t j = 0; j < N; j++) {
            //printf("u_k[%lu] = %1.5f\n", j, u_k[j]);
            if (_W[j] == 0) {
                float alpha = 1.0f;
                if (u_k[j] + p[j] > _u_up[j]) {
                    alpha = (_u_up[j] - u_k[j]) / p[j];
                    //printf("exceed upper bound %lu\n", j);
                    //printf("div by %1.15f\talpha: %1.30f\n", p[j], (_u_up[j] - u_k[j]));
                }
                else if (u_k[j] + p[j] < _u_lo[j]) {
                    alpha = (_u_lo[j] - u_k[j]) / p[j];
                    //printf("exceed lower bound %lu\n", j);
                    //printf("div by %1.15f\n", p[j]);
                }

                if (alpha < smallest_alpha) {
                    smallest_alpha = alpha;
                    smallest_alpha_idx = j;
                }
            }
        }

        if (smallest_alpha < 1.0f) {
            //printf("smallest alpha at %lu = %1.5f\n", smallest_alpha_idx, smallest_alpha);
            // scale the solution to fit within bounds
            for (size_t j = 0; j < N; j++) {
                u_k[j] += p[j] * smallest_alpha;
            }
            // add constraint to working set
            _W[smallest_alpha_idx] = p[smallest_alpha_idx] > 0.0f ? 1 : -1;
            printf("add %lu to working set\n", smallest_alpha_idx);
        } else {
            // check if an optimal solution was found using lagrangian
            // u_k = u_k + p
            for (size_t j = 0; j < N; j++) {
                u_k[j] += p[j];
            }
            // todo: lagrangian optimality check
            return 0;
        }

        return -1;
    }

    void checkActuatorLimits()
    {
        for (size_t j = 0; j < N; j++) {
            if (_u_lo[j] > _u_up[j]) {
                _u_lo[j] = _u_up[j];
            }
        }
    }

    void applyWeights()
    {
        for (size_t l = 0; l < M*N; l++) {
            _A[l] = _B[l] * _Wv[l%M];
        }
    }

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

    float _A[M*N]; // this is just B with applied weights
    float _u_up[N];
    float _u_lo[N];

    float _A_f[M*N];
    float _b[M];
    int8_t _W[M] = {0};

    LeastSquaresSolver _solver;
};

} // namespace ifl_control
