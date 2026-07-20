#include "dji_angle.h"

#include <math.h>

int32_t ModularDJIAngleTowards(int32_t dji_angle, int32_t target)
{
    return dji_angle - (int32_t)roundf((dji_angle - target) / 8192.0f) * 8192;
}