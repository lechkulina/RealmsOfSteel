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
    , context(ROS_NULL)
    , width(0)
    , height(0)
    , doubleBuffered(false)
    , resizable(false)
    , fullscreen(false) {
}

ros::SDLOpenGLWindow::~SDLOpenGLWindow() {
    uninit();
}

bool ros::SDLOpenGLWindow::init(const PropertyTree& config) {
    // set OpenGL attributes before creating OpenGL window
    int redSize = config.get("Application.Window.RedSize", 8);
    int greenSize = config.get("Application.Window.GreenSize", 8);
    int blueSize = config.get("Application.Window.BlueSize", 8);
    int alphaSize = config.get("Application.Window.AlphaSize", 8);
    if (SDL_GL_SetAttribute(SDL_GL_RED_SIZE, redSize) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, greenSize) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, blueSize) != 0||
        SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, alphaSize) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to set color buffer channels sizes to %d.%d.%d.%d: %s")
                            % redSize % greenSize % blueSize % alphaSize % SDL_GetError());
        uninit();
        return false;
    }

    int depthSize = config.get("Application.Window.DepthSize", 16);
    if (SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, depthSize) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to set depth buffer size to %d: %s") % depthSize % SDL_GetError());
        uninit();
        return false;
    }

    int stencilSize = config.get("Application.Window.StencilSize", 0);
    if (SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, stencilSize) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to set stencil buffer size to %d: %s") % stencilSize % SDL_GetError());
        uninit();
        return false;
    }

    doubleBuffered = config.get("Application.Window.DoubleBuffered", true);
    if (SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, doubleBuffered) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to %s double buffering: %s")
                            % (doubleBuffered ? "enable" : "disable")  % SDL_GetError());
        uninit();
        return false;
    }

    // create SDL window usable with OpenGL context
    Uint32 flags = SDL_WINDOW_OPENGL;
    if (config.get("Application.Window.Resizable", 0)) {
        flags |= SDL_WINDOW_RESIZABLE;
        resizable = true;
    }
    if (config.get("Application.Window.Fullscreen", 0)) {
        flags |= SDL_WINDOW_FULLSCREEN;
        fullscreen = true;
    }
    width = config.get("Application.Window.Width", 640);
    height = config.get("Application.Window.Height", 480);
    window = SDL_CreateWindow("Realms of Steel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags);
    if (!window) {
        Logger::report(LogLevel_Error, boost::format("Failed to create SDL window: %s") % SDL_GetError());
        uninit();
        return false;
    }

    context = SDL_GL_CreateContext(window);
    if (!context) {
        Logger::report(LogLevel_Error, boost::format("Failed to create OpenGL context: %s") % SDL_GetError());
        uninit();
        return false;
    }

    GLenum error = glewInit();
    if (error != GLEW_OK) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize OpenGL Extension Wrangler: %s") % glewGetErrorString(error));
        uninit();
        return false;
    }

    Logger::report(LogLevel_Debug, boost::format("Using OpenGL %s from %s") % glGetString(GL_VERSION) % glGetString(GL_VENDOR));
    return true;
}

void ros::SDLOpenGLWindow::uninit() {
    if (context) {
        SDL_GL_DeleteContext(context);
        context = ROS_NULL;
    }
    if (window) {
        SDL_DestroyWindow(window);
        window = ROS_NULL;
    }
    width = 0;
    height = 0;
    doubleBuffered = false;
    resizable = false;
    fullscreen = false;
}

void ros::SDLOpenGLWindow::swapBuffers() {
    SDL_GL_SwapWindow(window);
}

void ros::SDLOpenGLWindow::clearBuffers() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
