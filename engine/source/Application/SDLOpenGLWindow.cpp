/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include "SDLOpenGLWindow.h"

ros::SDLOpenGLWindow::SDLOpenGLWindow()
    : window(ROS_NULL)
    , context(ROS_NULL) {
}

ros::SDLOpenGLWindow::~SDLOpenGLWindow() {
    Uninit();
}

bool ros::SDLOpenGLWindow::Init(const PropertyTree& config) {
    // set OpenGL attributes before creating OpenGL window
    int redSizeConfig = config.get("Application.Window.RedSize", 8);
    int greenSizeConfig = config.get("Application.Window.GreenSize", 8);
    int blueSizeConfig = config.get("Application.Window.BlueSize", 8);
    int alphaSizeConfig = config.get("Application.Window.AlphaSize", 8);
    if (SDL_GL_SetAttribute(SDL_GL_RED_SIZE, redSizeConfig) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, greenSizeConfig) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, blueSizeConfig) != 0||
        SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, alphaSizeConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to set OpenGL color buffer channels sizes to %d.%d.%d.%d: %s")
                       % redSizeConfig % greenSizeConfig % blueSizeConfig % alphaSizeConfig
                       % SDL_GetError());
        Uninit();
        return false;
    }

    int depthSizeConfig = config.get("Application.Window.DepthSize", 16);
    if (SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, depthSizeConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to set OpenGL depth buffer size to %d: %s")
                       % depthSizeConfig
                       % SDL_GetError());
        Uninit();
        return false;
    }

    int stencilSizeConfig = config.get("Application.Window.StencilSize", 0);
    if (SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, stencilSizeConfig) != 0) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to set OpenGL stencil buffer size to %d: %s")
                       % stencilSizeConfig
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

    // create SDL window usable with OpenGL context
    Uint32 flagsConfig = SDL_WINDOW_OPENGL;
    if (config.get("Application.Window.AllowResizing", 0)) {
        flagsConfig |= SDL_WINDOW_RESIZABLE;
    }
    if (config.get("Application.Window.RunInFullscreen", 0)) {
        flagsConfig |= SDL_WINDOW_FULLSCREEN;
    }
    int widthConfig = config.get("Application.Window.Width", 640);
    int heightConfig = config.get("Application.Window.Height", 480);
    window = SDL_CreateWindow("Realms of Steel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, widthConfig, heightConfig, flagsConfig);
    if (!window) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to create SDL window: %s")
                       % SDL_GetError());
        Uninit();
        return false;
    }

    context = SDL_GL_CreateContext(window);
    if (!context) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to create OpenGL context: %s")
                       % SDL_GetError());
        Uninit();
        return false;
    }

    GLenum error = glewInit();
    if (error != GLEW_OK) {
        Logger::Report(LogLevel_Error, LogFormat("Failed to initialize OpenGL Extension Wrangler: %s")
                       % glewGetErrorString(error));
        Uninit();
        return false;
    }

    Logger::Report(LogLevel_Debug, LogFormat("Using OpenGL %s from %s")
                   % glGetString(GL_VERSION)
                   % glGetString(GL_VENDOR));
    return true;
}

void ros::SDLOpenGLWindow::Uninit() {
    if (context) {
        SDL_GL_DeleteContext(context);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
}

void ros::SDLOpenGLWindow::SwapBuffers() {
    SDL_GL_SwapWindow(window);
}

void ros::SDLOpenGLWindow::ClearBuffers() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
