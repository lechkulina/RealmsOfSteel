/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <cstdlib>
#include <boost/property_tree/info_parser.hpp>
#include <Application/Logger.h>
#include <Application/Application.h>

int main() {
    int exitCode = EXIT_FAILURE;

    ros::PropertyTree config;
    boost::property_tree::read_info("Config.info", config);

    if (!ros::Logger::initInstance(config.get_child("Logger")) ||
        !ros::Application::initInstance(config.get_child("application"))) {
        return exitCode;
    }

    exitCode = ros::Application::getInstance()->run();
    ros::Application::getInstance()->uninit();

    return exitCode;
}
