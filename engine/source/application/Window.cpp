/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include <Application/Window.h>
#if defined(ROS_USING_SDL) && defined(ROS_USING_OPENGL)
    #include "SDLOpenGLWindow.h"
#endif

ros::WindowFactory ros::Window::factory;

ros::WindowPtr ros::Window::create(const PropertyTree& config) {
    if (factory.isEmpty()) {
#if defined(ROS_USING_SDL) && defined(ROS_USING_OPENGL)
        factory.registerClass<SDLOpenGLWindow>("OpenGL");
#endif
    }

    std::string type = config.data();
    WindowPtr window(factory.create(type));
    if (!window) {
        Logger::report(LogLevel_Error, boost::format("Failed to create window: Unknown type %s") % type);
        return WindowPtr();
    }
    if (!window->init(config)) {
        return WindowPtr();
    }

    return window;
}
