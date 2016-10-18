/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include "OpenGLWindow.h"

ros::OpenGLWindow::OpenGLWindow()
    : window(ROS_NULL)
    , context(ROS_NULL) {
}

ros::OpenGLWindow::~OpenGLWindow() {
    Uninit();
}

bool ros::OpenGLWindow::Init(const PropertyTree& config) {
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    int width = config.get("Application.Window.Width", 640);
    int height = config.get("Application.Window.Height", 480);
    window = SDL_CreateWindow("Realms of Steel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_OPENGL);

    context = SDL_GL_CreateContext(window);
    if (!context) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to initialize OpenGL window: %s") % SDL_GetError());
        SDL_DestroyWindow(window);
        return false;
    }

    GLenum error = glewInit();
    if (error != GLEW_OK) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to initialize OpenGL window: %s") % glewGetErrorString(error));
        SDL_GL_DeleteContext(context);
        SDL_DestroyWindow(window);
        return false;
    }

    Logger::Report(LogLevel_Debug, LogFormat("Using OpenGL %s from %s") % glGetString(GL_VERSION) % glGetString(GL_VENDOR));
    return true;
}

void ros::OpenGLWindow::Uninit() {
    if (context) {
        SDL_GL_DeleteContext(context);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
}

void ros::OpenGLWindow::SwapBuffers() {
    SDL_GL_SwapWindow(window);
}

void ros::OpenGLWindow::ClearBuffers() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
