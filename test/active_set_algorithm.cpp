#include "test_macros.hpp"
#include "ifl_control/stdlib_imports.hpp"

#include "ifl_control/ActiveSetAlgorithm.hpp"

using namespace ifl_control;

int test_just_roll();
bool isEqual(const float actual[], const float expected[], size_t len, float eps = 1e-6f);

int main()
{
    int ret = -1;

    ret = test_just_roll();
    if (ret < 0) {
        return ret;
    }

    return ret;
}

int test_just_roll()
{
    float B[] = {-20.0f, 20.0f, 20.0f, -20.0f,
                 17.0f, -17.0f, 17.0f, -17.0f,
                 0.7f, 0.7f, -0.7f, -0.7f,
                 -1.2f, -1.2f, -1.2f, -1.2f
                };
    float Wv[] = {1000.0f, 1000.0f, 1.0f, 100.0f};
    float u_up[] = {1.0f, 1.0f, 1.0f, 1.0f};
    float u_lo[] = {-1.0f, -1.0f, -1.0f, -1.0f};

    float v[] = {10.0f, 0.0f, 0.0f, 0.0f};
    ActiveSetAlgorithm<4,4> asa;

    asa.setActuatorEffectiveness(B);
    asa.setOutputWeights(Wv);
    asa.setActuatorUpperLimit(u_up);
    asa.setActuatorLowerLimit(u_lo);

    float out[4] = {};
    int ret = asa.calculateActuatorCommands(v, out);
    (void) ret;
    float expected_out[4] = {-0.125f, 0.125f, 0.125f, -0.125f};
    TEST(isEqual(out, expected_out, 4));
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
