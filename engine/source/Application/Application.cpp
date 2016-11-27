/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#ifdef ROS_USING_SDL
    #include "SDLApplication.h"
#endif

ros::ApplicationFactory ros::Application::factory;
ros::ApplicationPtr ros::Application::application;

ros::ApplicationPtr ros::Application::create(const PropertyTree& config) {
    if (application) {
        return application;
    }

    if (factory.isEmpty()) {
#ifdef ROS_USING_SDL
        factory.registerClass<SDLApplication>("SDL");
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

void ros::Application::onKeyboardPressEvent(const KeyboardPressEvent& event) {
    for (ViewList::iterator iter = views.begin(); iter != views.end(); ++iter) {
        ViewPtr view = *iter;
        view->onKeyboardPressEvent(event);
    }
}
