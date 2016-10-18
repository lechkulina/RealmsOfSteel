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
    // set OpenGL attributes before creating OpenGL window
    int redSizeConfig = config.get("Application.Window.RedSize", 8);
    int greenSizeConfig = config.get("Application.Window.GreenSize", 8);
    int blueSizeConfig = config.get("Application.Window.BlueSize", 8);
    int alphaSizeConfig = config.get("Application.Window.AlphaSize", 8);
    if (SDL_GL_SetAttribute(SDL_GL_RED_SIZE, redSizeConfig) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, greenSizeConfig) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, blueSizeConfig) != 0||
        SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, alphaSizeConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to set OpenGL color buffer channels sizes: %s")
                       % SDL_GetError());
        Uninit();
        return false;
    }

    int depthSizeConfig = config.get("Application.Window.DepthSize", 16);
    if (SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, depthSizeConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to set OpenGL depth buffer size: %s")
                       % SDL_GetError());
        Uninit();
        return false;
    }

    int stencilSizeConfig = config.get("Application.Window.StencilSize", 0);
    if (SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, stencilSizeConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to set OpenGL stencil buffer size: %s")
                       % SDL_GetError());
        Uninit();
        return false;
    }

    bool enableDoubleBufferingConfig = config.get("Application.Window.EnableDoubleBuffering", true);
    if (SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, enableDoubleBufferingConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to %s OpenGL double buffering: %s")
                       % (enableDoubleBufferingConfig ? "enable" : "disable")
                       % SDL_GetError());
        Uninit();
        return false;
    }


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
