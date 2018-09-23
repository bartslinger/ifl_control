#include "test_macros.hpp"

#include "ifl_control/LeastSquaresSolver.hpp"

using namespace ifl_control;

int test_4x3();
int test_4x4();
int test_div_zero();

void to_column_major(const float data_row_major[], size_t rows, size_t columns, float data[]);
bool isEqual(const float actual[], const float expected[], size_t len, float eps = 1e-6f);

int main()
{
    int ret = -1;

    ret = test_4x3();
    if (ret < 0) {
        return ret;
    }

    ret = test_4x4();
    if (ret < 0) {
        return ret;
    }

    ret = test_div_zero();
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int test_4x3()
{
    const size_t m = 4;
    const size_t n = 3;
    const float data_row_major[m*n] = { 20.f, -10.f, -13.f,
                                        17.f,  16.f, -18.f,
                                        0.7f,  -0.8f,   0.9f,
                                        -1.f,  -1.1f,  -1.2f
                                      };

    // These will be available for the solver to use.
    float A[m*n];
    float tau[m];
    float w[m];

    to_column_major(data_row_major, m, n, A);
    float b[m] = {2.0f, 3.0f, 4.0f, 5.0f};
    float x_check[n] = { -0.69168233f,
                         -0.26227593f,
                         -1.03767522f
                        };

    LeastSquaresSolver solver;
    solver.setMatrix(A, tau, w, m, n);

    float x[m] = {};
    solver.solve(b, x);
    TEST(isEqual(x, x_check, n));

    return 0;
}

int test_4x4()
{
    const size_t m = 4;
    const size_t n = 4;
    const float data_row_major[16] = { 20.f, -10.f, -13.f,  21.f,
                                       17.f,  16.f, -18.f, -14.f,
                                       0.7f,  -0.8f,   0.9f,  -0.5f,
                                       -1.f,  -1.1f,  -1.2f,  -1.3f
                                     };

    // These will be available for the solver to use.
    float A[m*n];
    float tau[m];
    float w[m];

    to_column_major(data_row_major, 4, 4, A);
    float b[m] = {2.0f, 3.0f, 4.0f, 5.0f};
    float x_check[n] = { 0.97893433f,
                         -2.80798701f,
                         -0.03175765f,
                         -2.19387649f
                       };

    LeastSquaresSolver solver;
    solver.setMatrix(A, tau, w, m, n);

    float x[m] = {};
    solver.solve(b, x);
    TEST(isEqual(x, x_check, n));

    return 0;
}

int test_div_zero() {
    const size_t m = 2;
    const size_t n = 2;
    float A[m*n] = {0.0f, 0.0f, 0.0f, 0.0f};

    // These will be available for the solver to use.
    float tau[m] = {};
    float w[m] = {};
    float b[m] = {1.0f, 1.0f};

    // Implement such that x returns zeros if it reaches div by zero
    float x_check[n] = {0.0f, 0.0f};

    LeastSquaresSolver solver;
    solver.setMatrix(A, tau, w, m, n);

    float x[m] = {};
    solver.solve(b, x);
    TEST(isEqual(x, x_check, n));

    return 0;
}

bool isEqual(const float actual[], const float expected[], size_t len, float eps)
{
    bool equal = true;
    for (size_t i = 0; i < len; i++) {
        if (fabs(actual[i] - expected[i]) > eps) {
            equal = false;
            break;
        }
    }

    if (!equal) {
        printf("not equal!\n");
        printf("index\tactual\texpected\n");
        for (size_t i = 0; i < len; i++) {
            printf("%lu\t%1.5f\t%1.5f\n", i, actual[i], expected[i]);
        }
    }

    return equal;
}

void to_column_major(const float data_row_major[], size_t rows, size_t columns, float data[])
{
    for (size_t i = 0; i < rows*columns; i++) {
        data[i] = data_row_major[(i%rows)*columns + (i/rows)];
        //printf("%lu\t%1.2f\n", i, data[i]);
    }
}
