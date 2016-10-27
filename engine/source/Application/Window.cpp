/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include <Application/Window.h>
#include "SDLOpenGLWindow.h"

ros::WindowFactory ros::Window::factory;

ros::WindowPtr ros::Window::Create(const PropertyTree& config) {
    if (factory.IsEmpty()) {
#if defined(ROS_USING_SDL) && defined(ROS_USING_OPENGL)
        factory.RegisterClass<SDLOpenGLWindow>("SDLOpenGL");
#endif
    }

    String type = config.data();
    WindowPtr window(factory.CreateInstance(type));
    if (!window) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to create window: Unknown type %s") % type);
        return window;
    }

    if (!window->Init(config)) {
        window.reset();
    }

    return window;
}
