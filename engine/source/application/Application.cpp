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

const float ros::Application::DEFAULT_FRAMES_PER_SECOND = 60.0f;
const float ros::Application::DEFAULT_MAX_ACCUMULATED_TICKS = 50.0f;
const bool ros::Application::DEFAULT_QUIT_ON_ESCAPE = false;

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

ros::Application::Application()
    : framesPerSecond(DEFAULT_FRAMES_PER_SECOND)
    , maxAccumulatedTicks(DEFAULT_MAX_ACCUMULATED_TICKS)
    , quitOnEscape(DEFAULT_QUIT_ON_ESCAPE)
    , quitRequested(false) {
}

bool ros::Application::init(const PropertyTree& config) {
    framesPerSecond = config.get("FramesPerSecond", DEFAULT_FRAMES_PER_SECOND);
    maxAccumulatedTicks = config.get("MaxAccumulatedTicks", DEFAULT_MAX_ACCUMULATED_TICKS);
    quitOnEscape = config.get("QuitOnEscape", DEFAULT_QUIT_ON_ESCAPE);

    PropertyTree::const_assoc_iterator iter = config.find("Window");
    if (iter == config.not_found()) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize the application - missing window definition"));
        uninit();
        return false;
    }
    window = createWindow();
    if (!window || !window->init(config)) {
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

int ros::Application::run() {
    Logger::report(LogLevel_Trace, boost::format("Starting Realms Of Steel %s") % ROS_VERSION);

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

void ros::Application::onUpdateEvent(float) {

}

void ros::Application::onRenderEvent() {

}
