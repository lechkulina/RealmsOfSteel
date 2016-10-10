/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/Application.h>
#include "SDLApplication.h"

ros::Application::ApplicationFactory ros::Application::factory;

ros::ApplicationPtr ros::Application::Create(const PropertyTree& config) {
    if (factory.IsEmpty()) {
#ifdef ROS_USING_SDL
        factory.RegisterClass<SDLApplication>("SDL");
#endif
    }

    ApplicationPtr instance;
    try {
        std::string platformName = config.get<std::string>("Application.PlatformName");
        instance.reset(factory.CreateInstance(platformName));
        if (!instance) {
            std::cerr << "Failed to create application: Unknown platform " << platformName << std::endl;
            return ApplicationPtr();
        }
    } catch (const boost::property_tree::ptree_bad_path& exception) {
        std::cerr << "Failed to read application configuration: " << exception.what() << std::endl;
        return ApplicationPtr();
    }

    if (!instance->Init(config)) {
        return ApplicationPtr();
    }

    return instance;
}
