/**
 * @file LeastSquaresSolver.hpp
 *
 * This is an efficient implementation of a least squares solver using the QR
 * decomposition. The matrix data is stored externally, in a column-major
 * format. This also allows to use the same solver for matrices of different
 * dimensions. The number of rows and columns are parameters for the solver.
 *
 * @author Bart Slinger <bartslinger@gmail.com>
 */

#pragma once

class LeastSquaresSolver
{
public:
    LeastSquaresSolver() = default;

    void setMatrix(float *A, size_t m, size_t n)
    {
        _A = A;
        _m = m;
        _n = n;
    }

    int solve(const float b[], float x_out[])
    {
        return 0;
    }

private:
    float *_A = nullptr;
    size_t _m = 0;
    size_t _n = 0;
};
