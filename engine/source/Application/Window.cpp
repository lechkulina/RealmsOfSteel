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

    WindowPtr instance;
    try {
        String backendType = config.get<String>("Application.Window");
        instance.reset(factory.CreateInstance(backendType));
        if (!instance) {
            std::cerr << "Failed to create window: Unknown backend type " << backendType << std::endl;
            return WindowPtr();
        }
    } catch (const BadPathException& exception) {
        std::cerr << "Failed to read window configuration: " << exception.what() << std::endl;
        return WindowPtr();
    }

    if (!instance->Init(config)) {
        return WindowPtr();
    }

    return instance;
}
