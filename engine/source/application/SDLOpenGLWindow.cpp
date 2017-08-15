/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include "SDLOpenGLWindow.h"

namespace {
    const int DEFAULT_RED_SIZE = 8;
    const int DEFAULT_GREEN_SIZE = 8;
    const int DEFAULT_BLUE_SIZE = 8;
    const int DEFAULT_ALPHA_SIZE = 8;
    const int DEFAULT_DEPTH_SIZE = 16;
    const int DEFAULT_STENCIL_SIZE = 0;
    const bool DEFAULT_DOUBLE_BUFFERED = true;

    const bool DEFAULT_RESIZABLE = false;
    const bool DEFAULT_FULLSCREEN = false;
    const int DEFAULT_WIDTH = 640;
    const int DEFAULT_HEIGHT = 480;
}

ros::SDLOpenGLWindow::SDLOpenGLWindow()
    : window(ROS_NULL)
    , context(ROS_NULL)
    , doubleBuffered(DEFAULT_DOUBLE_BUFFERED)
    , resizable(DEFAULT_RESIZABLE)
    , fullscreen(DEFAULT_FULLSCREEN)
    , width(DEFAULT_WIDTH)
    , height(DEFAULT_HEIGHT) {
}

ros::SDLOpenGLWindow::~SDLOpenGLWindow() {
    uninit();
}

bool ros::SDLOpenGLWindow::init(const PropertyTree& config) {
    if (!setAttributes(config) || !initWindow(config) || !initContext()) {
        uninit();
        return false;
    }
    ROS_DEBUG(boost::format("Window initialized successfully - using OpenGL %s from %s")
                % glGetString(GL_VERSION) % glGetString(GL_VENDOR));
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

bool ros::SDLOpenGLWindow::setAttributes(const PropertyTree& config) {
    int redSize = config.get("red-size", DEFAULT_RED_SIZE);
    int greenSize = config.get("green-size", DEFAULT_GREEN_SIZE);
    int blueSize = config.get("blue-size", DEFAULT_BLUE_SIZE);
    int alphaSize = config.get("alpha-size", DEFAULT_ALPHA_SIZE);
    if (SDL_GL_SetAttribute(SDL_GL_RED_SIZE, redSize) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, greenSize) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, blueSize) != 0||
        SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, alphaSize) != 0) {
        ROS_ERROR(boost::format("Failed to set color buffer channels sizes to %d.%d.%d.%d - SDL error occured %s")
                    % redSize % greenSize % blueSize % alphaSize % SDL_GetError());
        return false;
    }

    int depthSize = config.get("depth-size", DEFAULT_DEPTH_SIZE);
    if (SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, depthSize) != 0) {
        ROS_ERROR(boost::format("Failed to set depth buffer size to %d - SDL error occured %s") % depthSize % SDL_GetError());
        return false;
    }

    int stencilSize = config.get("stencil-size", DEFAULT_STENCIL_SIZE);
    if (SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, stencilSize) != 0) {
        ROS_ERROR(boost::format("Failed to set stencil buffer size to %d - SDL error occured %s") % stencilSize % SDL_GetError());
        return false;
    }

    doubleBuffered = config.get("double-buffered", DEFAULT_DOUBLE_BUFFERED);
    if (SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, doubleBuffered) != 0) {
        ROS_ERROR(boost::format("Failed to %s double buffering - SDL error occured %s")
                    % (doubleBuffered ? "enable" : "disable")  % SDL_GetError());
        return false;
    }

    return true;
}

bool ros::SDLOpenGLWindow::initWindow(const PropertyTree& config) {
    Uint32 flags = SDL_WINDOW_OPENGL;
    if (config.get("resizable", DEFAULT_RESIZABLE)) {
        flags |= SDL_WINDOW_RESIZABLE;
        resizable = true;
    }
    if (config.get("fullscreen", DEFAULT_FULLSCREEN)) {
        flags |= SDL_WINDOW_FULLSCREEN;
        fullscreen = true;
    }
    width = config.get("width", DEFAULT_WIDTH);
    height = config.get("height", DEFAULT_HEIGHT);

    window = SDL_CreateWindow("Realms of Steel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags);
    if (!window) {
        ROS_ERROR(boost::format("Failed to create the window - SDL error occured %s") % SDL_GetError());
        return false;
    }

    return true;
}

bool ros::SDLOpenGLWindow::initContext() {
    context = SDL_GL_CreateContext(window);
    if (!context) {
        ROS_ERROR(boost::format("Failed to create OpenGL context - SDL error occured %s") % SDL_GetError());
        return false;
    }

    GLenum error = glewInit();
    if (error != GLEW_OK) {
        ROS_ERROR(boost::format("Failed to initialize OpenGL - GLEW error occured %s") % glewGetErrorString(error));
        return false;
    }

    return true;
}
