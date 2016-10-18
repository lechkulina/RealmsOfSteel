/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include "SDLApplication.h"

ros::SDLApplication::SDLApplication()
    : hasQuit(false) {
}

ros::SDLApplication::~SDLApplication() {
    Uninit();
}

bool ros::SDLApplication::Init(const PropertyTree& config) {
    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to initialize SDL application: %s") % SDL_GetError());
        Uninit();
        return false;
    }

    SDL_version version;
    memset(&version, 0, sizeof(version));
    SDL_GetVersion(&version);
    Logger::Report(LogLevel_Debug, LogFormat("Using SDL %d.%d.%d") % (int)version.major % (int)version.minor % (int)version.patch);

    PropertyConstAssocIter iter = config.find("Window");
    if (iter == config.not_found()) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to initialize SDL application: Missing window configuration"));
        Uninit();
        return false;
    }

    window = Window::Create(iter->second);
    if (!window) {
        Uninit();
        return false;
    }

    return true;
}

void ros::SDLApplication::Uninit() {
    if (window) {
        window->Uninit();
    }
    SDL_Quit();
}

int ros::SDLApplication::Run() {
    Logger::Report(LogLevel_Trace, LogFormat("Starting Realms Of Steel %s") % ROS_VERSION);

    float fps = 60.0f;
    float deltaTime = (1 / fps) * 1000;
    float maxAccumulatedTime = 50.0f;
    float accumulatedTime = 0.0f;
    float startTime = (float)SDL_GetTicks();

    while (!hasQuit) {
        SDL_Event event;
        if (SDL_PollEvent(&event) > 0) {
            OnEvent(event);
            continue;
        }

        float currentTime = (float)SDL_GetTicks();
        accumulatedTime += currentTime - startTime;
        if (accumulatedTime > maxAccumulatedTime) {
            accumulatedTime = maxAccumulatedTime;
        }
        startTime = currentTime;
        while (accumulatedTime > deltaTime) {
            OnUpdate(deltaTime);
            accumulatedTime -= deltaTime;
        }

        window->ClearBuffers();
        OnRender();
        window->SwapBuffers();
    }

    return EXIT_SUCCESS;
}

void ros::SDLApplication::OnEvent(const SDL_Event& event) {
    switch (event.type) {
        case SDL_QUIT: {
            hasQuit = true;
        } break;

        case SDL_KEYDOWN: {
            switch (event.key.keysym.scancode) {
                case SDL_SCANCODE_ESCAPE: {
                    hasQuit = true;
                    break;
                }
                default:
                    break;
            }
        } break;

        default:
            break;
    }

}

void ros::SDLApplication::OnUpdate(float) {

}

void ros::SDLApplication::OnRender() {

}
