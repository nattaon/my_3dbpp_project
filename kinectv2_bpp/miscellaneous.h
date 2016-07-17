#ifndef MISCELLANEOUS_H
#define MISCELLANEOUS_H

#include "AllHeader.h"

class Miscellaneous
{
public:
    Miscellaneous();
    int randomcolor();
    double get_r (int color);
    double get_g (int color);
    double get_b (int color);
    int float_scale_to_int(float number);
    float int_scale_to_float(int number);
};

#endif // MISCELLANEOUS_H
