/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <cstdlib>
#include <boost/property_tree/info_parser.hpp>
#include <application/Logger.h>
#include <application/Application.h>
#include <resources/FileSystem.h>
#include <resources/ResourcesCache.h>
#include <graphics/Scene.h>

int main() {
    int exitCode = EXIT_FAILURE;

    ros::PropertyTree config;
    boost::property_tree::read_info("Config.info", config);

    if (!ros::Logger::initInstance(config.get_child("logger")) ||
        !ros::Application::initInstance(config.get_child("application")) ||
        !ros::FileSystem::initInstance(config.get_child("file-system")) ||
        !ros::ResourcesCache::initInstance(config.get_child("resources-cache"))) {
        return exitCode;
    }

    ros::ApplicationPtr application = ros::Application::getInstance();
    if (application) {
        exitCode = application->run();
        application->uninit();
    }

    return exitCode;
}
