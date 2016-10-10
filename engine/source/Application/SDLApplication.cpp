/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/Logger.h>
#include "SDLApplication.h"

bool ros::SDLApplication::Init(const PropertyTree& config) {
    Logger logger;
    logger.Init(config);
    logger.SendMessage(LogMessage(LogLevel_Trace, "Starting Realms Of Steel %s") % ROS_VERSION);
    logger.SendMessage(LogMessage(LogLevel_Error, "Foo bar"));

    SDL_Init(SDL_INIT_VIDEO);

    return Application::Init(config);
}

int ros::SDLApplication::Run() {
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

void ros::SDLApplication::Uninit() {
    window->Uninit();
    SDL_Quit();
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
