
#include <iostream>
#include "config/Config.h"
#include "Pipeline.h"

int main (int argc, char** argv)
{

    p2c::Config::instance()->parse_cmd_line(argc, argv);

    p2c::Pipeline pipeline;
    pipeline.run();
}
