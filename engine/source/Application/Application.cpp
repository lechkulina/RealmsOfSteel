/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/Application.h>
#include "SDLApplication.h"

ros::ApplicationFactory ros::Application::factory;

ros::ApplicationPtr ros::Application::Create(const PropertyTree& config) {
    if (factory.IsEmpty()) {
#ifdef ROS_USING_SDL
        factory.RegisterClass<SDLApplication>("SDL");
#endif
    }

    ApplicationPtr instance;
    try {
        String backendType = config.get<String>("Application");
        instance.reset(factory.CreateInstance(backendType));
        if (!instance) {
            std::cerr << "Failed to create application: Unknown backend type " << backendType << std::endl;
            return ApplicationPtr();
        }
    } catch (const BadPathException& exception) {
        std::cerr << "Failed to read application configuration: " << exception.what() << std::endl;
        return ApplicationPtr();
    }

    if (!instance->Init(config)) {
        return ApplicationPtr();
    }

    return instance;
}

bool ros::Application::Init(const PropertyTree& config) {
    window = Window::Create(config);
    return window;
}
