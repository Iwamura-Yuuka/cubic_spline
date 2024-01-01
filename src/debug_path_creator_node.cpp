#include "cubic_spline/debug_path_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "debug_path_creator");
    DebugPathCreator debugpathcreator;
    debugpathcreator.process();

    return 0;
}