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

namespace {
    const float DEFAULT_FRAMES_PER_SECOND = 60.0f;
    const float DEFAULT_MAX_ACCUMULATED_TICKS = 50.0f;
    const bool DEFAULT_QUIT_ON_ESCAPE = false;
}

ros::ApplicationFactory ros::Application::factory;
ros::ApplicationPtr ros::Application::application;

ros::ApplicationPtr ros::Application::initInstance(const PropertyTree& config) {
    if (application) {
        return application;
    }

    if (factory.isEmpty()) {
#if defined(ROS_USING_SDL) && defined(ROS_USING_OPENGL)
        factory.registerClass<SDLOpenGLApplication>(boost::regex("sdl-opengl"));
#endif
    }

    StringOpt type = config.get_optional<std::string>("type");
    if (!type) {
        ROS_ERROR(boost::format("Missing type property in application"));
        return application;
    }
    application.reset(factory.create(*type));
    if (!application) {
        ROS_ERROR(boost::format("Unknown type %s property used for application") % (*type));
        return application;
    }
    if (!application->init(config)) {
        application.reset();
    }

    return application;
}

ros::Application::Application()
    : framesPerSecond(DEFAULT_FRAMES_PER_SECOND)
    , maxAccumulatedTicks(DEFAULT_MAX_ACCUMULATED_TICKS)
    , quitOnEscape(DEFAULT_QUIT_ON_ESCAPE)
    , quitRequested(false) {
}

bool ros::Application::init(const PropertyTree& config) {
    framesPerSecond = config.get("frames-per-second", DEFAULT_FRAMES_PER_SECOND);
    maxAccumulatedTicks = config.get("max-accumulated-ticks", DEFAULT_MAX_ACCUMULATED_TICKS);
    quitOnEscape = config.get("quit-on-escape", DEFAULT_QUIT_ON_ESCAPE);

    if (!initWindow(config)) {
        uninit();
        return false;
    }

    return true;
}

void ros::Application::uninit() {
    window.reset();
}

int ros::Application::run() {
    ROS_TRACE(boost::format("Starting Realms Of Steel %s") % ROS_VERSION);

    onStartEvent();

    float frameDuration = (1 / framesPerSecond) * 1000;
    float accumulatedTicks = 0.0f;
    float previousTicks = getTicks();

    while (!quitRequested) {
        if (translateEvent()) {
            continue;
        }

        float currentTicks = getTicks();
        accumulatedTicks += currentTicks - previousTicks;
        accumulatedTicks = std::max(accumulatedTicks, maxAccumulatedTicks);
        previousTicks = currentTicks;
        while (accumulatedTicks > frameDuration) {
            onUpdateEvent(frameDuration);
            accumulatedTicks -= frameDuration;
        }

        window->clearBuffers();
        onRenderEvent();
        window->swapBuffers();
    }

    onStopEvent();
    return EXIT_SUCCESS;
}

void ros::Application::onQuitEvent() {
    quitRequested = true;
}

void ros::Application::onKeyboardPressEvent(const KeyboardPressEvent& event) {
    if (quitOnEscape && event.button == KeyboardButton_Escape) {
        onQuitEvent();
        return;
    }

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

void ros::Application::onStartEvent() {

}

void ros::Application::onUpdateEvent(float) {

}

void ros::Application::onRenderEvent() {

}

void ros::Application::onStopEvent() {

}

bool ros::Application::initWindow(const PropertyTree& config) {
    PropertyTree::const_assoc_iterator iter = config.find("window");
    if (iter == config.not_found()) {
        ROS_ERROR(boost::format("Missing window property in application"));
        return false;
    }
    window = createWindow();
    if (!window || !window->init(config.to_iterator(iter)->second)) {
        return false;
    }
    return true;
}
