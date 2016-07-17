#include "miscellaneous.h"


Miscellaneous::Miscellaneous()
{

}


int Miscellaneous::randomcolor()
{
    int r=rand()%200+56;
    int g=rand()%200+56;
    int b=rand()%200+56;

    int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);

    //cout << "color = " << rgb << endl;

    return rgb;
}


double Miscellaneous::get_r (int color)
{
    uint8_t r=(color >> 16) & 0x0000ff;

    //cout << "r:" << (int)r <<endl;

    return ((int)r)/255.0;
}
double Miscellaneous::get_g (int color)
{
    uint8_t g = (color >> 8) & 0x0000ff;

    //cout << "g:" << (int)g <<endl;

    return ((int)g)/255.0;
}
double Miscellaneous::get_b (int color)
{
    uint8_t b =(color) & 0x0000ff;

    //cout << "b:" << (int)b <<endl;

    return ((int)b)/255.0;
}
int Miscellaneous::float_scale_to_int(float number)
{
    return (int)(number*100);
}
float Miscellaneous::int_scale_to_float(int number)
{
    float value=number*0.01;
    return value;
}
