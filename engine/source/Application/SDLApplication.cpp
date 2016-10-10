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

    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    window = SDL_CreateWindow("Realms of Steel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_OPENGL);

    context = SDL_GL_CreateContext(window);
    if (!context) {
        std::cerr << "Failed to create OpenGL context " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        return false;
    }

    GLenum error = glewInit();
    if (error != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW " << glewGetErrorString(error) << std::endl;
        SDL_GL_DeleteContext(context);
        SDL_DestroyWindow(window);
        return false;
    }

    std::cout << "Using OpenGL " << glGetString(GL_VERSION) << " from " << glGetString(GL_VENDOR) << std::endl;
    return true;
}

int ros::SDLApplication::Run() {
    float fps = 60.0f;
    float deltaTime = (1 / fps) * 1000;
    float maxAccumulatedTime = 50.0f;
    float accumulatedTime = 0.0f;
    float startTime = (float)SDL_GetTicks();

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

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

        OnRender();
        SDL_GL_SwapWindow(window);
    }

    return EXIT_SUCCESS;
}

void ros::SDLApplication::Uninit() {
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
