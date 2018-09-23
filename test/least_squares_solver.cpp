#include "test_macros.hpp"

#include "ifl_control/LeastSquaresSolver.hpp"

int test_4x4_inverse();
void to_column_major(const float data_row_major[], size_t rows, size_t columns, float data[]);
bool isEqual(const float actual[], const float expected[], size_t len, float eps = 1e-8f);

int main()
{
    int ret = -1;

    ret = test_4x4_inverse();
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int test_4x4_inverse()
{
    const float data_row_major[16] = { 20.f, -10.f, -13.f,  21.f,
                                       17.f,  16.f, -18.f, -14.f,
                                       0.7f,  -0.8f,   0.9f,  -0.5f,
                                       -1.f,  -1.1f,  -1.2f,  -1.3f
                                     };
    float A[16];
    to_column_major(data_row_major, 4, 4, A);
    float b[4] = {2.0f, 3.0f, 4.0f, 5.0f};
    float x_check[4] = { 0.97893433f,
                         -2.80798701f,
                         -0.03175765f,
                         -2.19387649f
                       };

    LeastSquaresSolver solver;
    solver.setMatrix(A, 4, 4);

    float x[4] = {};
    solver.solve(b, x);
    TEST(isEqual(x, x_check, 4));

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
