/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <cstdlib>
#include <boost/property_tree/info_parser.hpp>
#include <Application/Application.h>

using namespace ros;

int main() {
    ros::PropertyTree config;
    boost::property_tree::read_info("Config.info", config);
    ros::ApplicationPtr application = ros::Application::Create(config);
    int exitCode = EXIT_FAILURE;
    if (application) {
        exitCode = application->Run();
        application->Uninit();
    }
    return exitCode;
}
