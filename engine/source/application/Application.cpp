/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#if defined(ROS_USING_SDL) && defined(ROS_USING_OPENGL)
    #include "SDLOpenGLApplication.h"
#endif

ros::ApplicationFactory ros::Application::factory;
ros::ApplicationPtr ros::Application::application;

ros::ApplicationPtr ros::Application::create(const PropertyTree& config) {
    if (application) {
        return application;
    }

    if (factory.isEmpty()) {
#if defined(ROS_USING_SDL) && defined(ROS_USING_OPENGL)
        factory.registerClass<SDLOpenGLApplication>("SDLOpenGL");
#endif
    }

    const std::string& type = config.data();
    application.reset(factory.create(type));
    if (!application) {
        Logger::report(LogLevel_Error, boost::format("Failed to create application: Unknown type %s") % type);
        return application;
    }
    if (!application->init(config)) {
        application.reset();
    }

    return application;
}

bool ros::Application::init(const PropertyTree& config) {
    if (!preInit(config)) {
        uninit();
        return false;
    }

    PropertyTree::const_assoc_iterator iter = config.find("Window");
    if (iter == config.not_found()) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize application: Missing window configuration"));
        uninit();
        return false;
    }
    window = createWindow(config);
    if (!window) {
        uninit();
        return false;
    }

    if (!postInit(config)) {
        uninit();
        return false;
    }

    return true;
}

void ros::Application::uninit() {
    if (window) {
        window->uninit();
    }
}

void ros::Application::onKeyboardPressEvent(const KeyboardPressEvent& event) {
    for (ViewList::iterator iter = views.begin(); iter != views.end(); ++iter) {
        ViewPtr view = *iter;
        view->onKeyboardPressEvent(event);
    }
}

void ros::Application::onMouseMotionEvent(const MouseMotionEvent& event) {
    for (ViewList::iterator iter = views.begin(); iter != views.end(); ++iter) {
        ViewPtr view = *iter;
        view->onMouseMotionEvent(event);
    }
}

void ros::Application::onMousePressEvent(const MousePressEvent& event) {
    for (ViewList::iterator iter = views.begin(); iter != views.end(); ++iter) {
        ViewPtr view = *iter;
        view->onMousePressEvent(event);
    }
}
