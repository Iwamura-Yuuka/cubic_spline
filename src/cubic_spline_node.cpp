#include "cubic_spline/cubic_spline.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cubic_spline");
    CubicSpline cubicspline;
    cubicspline.process();

    return 0;
}