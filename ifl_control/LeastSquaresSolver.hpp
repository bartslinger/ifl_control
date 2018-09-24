/**
 * @file LeastSquaresSolver.hpp
 *
 * This is an efficient implementation of a least squares solver using the QR
 * decomposition. The matrix data is stored externally, in a column-major
 * format. This also allows to use the same solver for matrices of different
 * dimensions. The number of rows and columns are parameters for the solver.
 *
 * It will calculate the pseudo-inverse when there are more columns than rows.
 *
 * @author Bart Slinger <bartslinger@gmail.com>
 */

#pragma once

namespace ifl_control {

class LeastSquaresSolver
{
public:
    LeastSquaresSolver() = default;

    int setMatrix(float *A, float *tau, float *w, size_t m, size_t n)
    {
        _A = A;
        _tau = tau;
        _w = w;
        _m = m;
        _n = n;

        // Perform the QR decomposition
        if (decomposeQR() < 0) {
            return -1;
        }

        return 0;
    }

    int solve(const float b[], float x_out[])
    {
        // copy b to x_out
        for (size_t i = 0; i < _m; i++) {
            x_out[i] = b[i];
        }

        for (size_t j = 0; j < _n; j++) {
            for (size_t i = j+1; i < _m; i++) {
                _w[i-j] = _A[j*_m + i];
            }
            float tmp = 0.0f;
            for (size_t i = j; i < _m; i++) {
                tmp += _w[i-j] * x_out[i];
            }

            for (size_t i = j; i < _m; i++) {
                x_out[i] -= _tau[j] * _w[i-j] * tmp;
            }
        }

        for (size_t l = _n; l > 0; l--) {
            size_t i = l - 1;
            for (size_t r = i+1; r < _n; r++) {
                x_out[i] -= _A[r*_m + i] * x_out[r];
            }
            if (abs(_A[i*_m + i]) < 1e-8f) {
                // fill output with zeros
                for (size_t z = 0; z < _n; z++) {
                    x_out[z] = 0.0f;
                }
            }
            x_out[i] /= _A[i*_m + i];
        }

        return 0;
    }

private:
    int decomposeQR() {
        _w[0] = 1.0f;
        for (size_t j = 0; j < _n; j++) {
            float normx = 0.0f;
            for (size_t i = j; i < _m; i++) {
                size_t idx = j*_m + i;
                normx += _A[idx] * _A[idx];
            }
            normx = sqrt(normx);
            float s = _A[j*_m + j] > 0.0f ? -1.0f : 1.0f;
            float u1 = _A[j*_m + j] - s*normx;
            if (normx < 1e-8f) {
                return -1;
            }
            for (size_t i = j+1; i < _m; i++) {
                _w[i-j] = _A[j*_m + i] / u1;
                _A[j*_m + i] = _w[i-j];
            }
            _A[j*_m + j] = s*normx;
            _tau[j] = -s*u1/normx;

            for (size_t k = j+1; k < _n; k++) {
                float tmp = 0.0f;
                for (size_t i = j; i < _m; i++) {
                    tmp += _w[i-j] * _A[k*_m + i];
                }
                for (size_t i = j; i < _m; i++) {
                    _A[k*_m + i] -= _tau[j] * _w[i-j] * tmp;
                }
            }
        }

        return 0;
    }

    float *_A = nullptr;
    float *_tau = nullptr;
    float *_w = nullptr;
    size_t _m = 0;
    size_t _n = 0;
};

} // namespace ifl_control
