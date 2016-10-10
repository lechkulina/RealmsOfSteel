/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include "OpenGLWindow.h"

ros::OpenGLWindow::OpenGLWindow()
    : window(ROS_NULL)
    , context(ROS_NULL) {
}

bool ros::OpenGLWindow::Init(const PropertyTree& config) {
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

void ros::OpenGLWindow::Uninit() {
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
}

void ros::OpenGLWindow::Swap() {
    SDL_GL_SwapWindow(window);
}

void ros::OpenGLWindow::OnRender() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
