#include "degree.h"

#include <math.h>

float ModularDegreeTowards(float degree, float target)
{
    return degree - roundf((degree - target) / 360.0f) * 360.0f;
}
