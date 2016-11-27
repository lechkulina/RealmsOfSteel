/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include "SDLApplication.h"

ros::SDLApplication::SDLApplication()
    : hasQuit(false) {
}

ros::SDLApplication::~SDLApplication() {
    uninit();
}

bool ros::SDLApplication::init(const PropertyTree& config) {
    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize SDL application: %s") % SDL_GetError());
        uninit();
        return false;
    }

    SDL_version version;
    memset(&version, 0, sizeof(version));
    SDL_GetVersion(&version);
    Logger::report(LogLevel_Debug, boost::format("Using SDL %d.%d.%d") % (int)version.major % (int)version.minor % (int)version.patch);

    PropertyTree::const_assoc_iterator iter = config.find("Window");
    if (iter == config.not_found()) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize SDL application: Missing window configuration"));
        uninit();
        return false;
    }

    window = Window::create(iter->second);
    if (!window) {
        uninit();
        return false;
    }

    return true;
}

void ros::SDLApplication::uninit() {
    if (window) {
        window->uninit();
    }
    SDL_Quit();
}

int ros::SDLApplication::run() {
    Logger::report(LogLevel_Trace, boost::format("Starting Realms Of Steel %s") % ROS_VERSION);

    float fps = 60.0f;
    float deltaTime = (1 / fps) * 1000;
    float maxAccumulatedTime = 50.0f;
    float accumulatedTime = 0.0f;
    float startTime = (float)SDL_GetTicks();

    while (!hasQuit) {
        SDL_Event event;
        if (SDL_PollEvent(&event) > 0) {
            onEvent(event);
            continue;
        }

        float currentTime = (float)SDL_GetTicks();
        accumulatedTime += currentTime - startTime;
        if (accumulatedTime > maxAccumulatedTime) {
            accumulatedTime = maxAccumulatedTime;
        }
        startTime = currentTime;
        while (accumulatedTime > deltaTime) {
            onUpdate(deltaTime);
            accumulatedTime -= deltaTime;
        }

        window->clearBuffers();
        onRender();
        window->swapBuffers();
    }

    return EXIT_SUCCESS;
}

void ros::SDLApplication::onEvent(const SDL_Event& event) {
    switch (event.type) {
        case SDL_QUIT: {
            hasQuit = true;
        } break;

        case SDL_KEYDOWN: {

        } break;

        default:
            break;
    }

}

void ros::SDLApplication::onUpdate(float) {

}

void ros::SDLApplication::onRender() {

}
