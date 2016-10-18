/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include <Application/Application.h>
#include "SDLApplication.h"

ros::ApplicationFactory ros::Application::factory;
ros::ApplicationPtr ros::Application::application;

ros::ApplicationPtr ros::Application::Create(const PropertyTree& config) {
    if (application) {
        return application;
    }

    if (factory.IsEmpty()) {
#ifdef ROS_USING_SDL
        factory.RegisterClass<SDLApplication>("SDL");
#endif
    }

    const String& type = config.data();
    application.reset(factory.CreateInstance(type));
    if (!application) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to create application: Unknown type %s") % type);
        return application;
    }

    if (!application->Init(config)) {
        application.reset();
    }

    return application;
}

bool ros::Application::Init(const PropertyTree& config) {
    PropertyConstAssocIter iter = config.find("Window");
    if (iter == config.not_found()) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to initialize application: Missing window configuration"));
        return false;
    }

    WindowPtr window = Window::Create(iter->second);
    if (!window) {
        return false;
    }
    this->window = window;

    return true;
}
