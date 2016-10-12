/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/Window.h>
#include "OpenGLWindow.h"

ros::WindowFactory ros::Window::factory;

ros::WindowPtr ros::Window::Create(const PropertyTree& config) {
    if (factory.IsEmpty()) {
#ifdef ROS_USING_OPENGL
        factory.RegisterClass<OpenGLWindow>("OpenGL");
#endif
    }

    String type = config.data();
    WindowPtr window(factory.CreateInstance(type));
    if (!window) {
        std::cerr << "Failed to create window: Unknown type " << type << std::endl;
        return window;
    }

    if (!window->Init(config)) {
        window.reset();
    }

    return window;
}
