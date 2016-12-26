/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/scoped_array.hpp>
#include <Application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLProgram.h"

ros::OpenGLProgram::OpenGLProgram()
    : handle(0) {
}

ros::OpenGLProgram::~OpenGLProgram() {
    uninit();
}

bool ros::OpenGLProgram::init(const PropertyTree& config) {
    if (!createHandle() || !createShaders(config) || !link()) {
        uninit();
        return false;
    }
    return true;
}

void ros::OpenGLProgram::uninit() {
    shaders.clear();
    if (handle) {
        glDeleteProgram(handle);
        handle = 0;
    }
}

bool ros::OpenGLProgram::bind() {
    glUseProgram(handle);
    return !OpenGL_checkForErrors();
}

void ros::OpenGLProgram::unbind() {
    glUseProgram(0);
}

bool ros::OpenGLProgram::createHandle() {
    handle = glCreateProgram();
    if (!handle || OpenGL_checkForErrors()) {
        return false;
    }
    return true;
}

bool ros::OpenGLProgram::createShaders(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "Shader") {
            continue;
        }
        OpenGLShaderPtr shader = boost::make_shared<OpenGLShader>();
        if (!shader || !shader->init(iter->second)) {
            return false;
        }
        glAttachShader(handle, shader->handle);
        if (OpenGL_checkForErrors()) {
            return false;
        }
        shaders.push_back(shader);
    }
    return true;
}

bool ros::OpenGLProgram::link() {
    GLint status = GL_FALSE;
    glLinkProgram(handle);
    glGetProgramiv(handle, GL_LINK_STATUS, &status);
    if (OpenGL_checkForErrors()) {
        return false;
    }

    if (!status) {
        Logger::report(LogLevel_Error, boost::format("Failed to link a program"));

        GLint length = 0;
        glGetProgramiv(handle, GL_INFO_LOG_LENGTH, &length);
        if (length == 0 || OpenGL_checkForErrors()) {
            return false;
        }

        boost::scoped_array<GLchar> buffer(new GLchar[length]);
        memset(buffer.get(), 0, sizeof(GLchar) * length);
        glGetProgramInfoLog(handle, length, NULL, buffer.get());
        if (OpenGL_checkForErrors()) {
            return false;
        }

        Logger::report(LogLevel_Debug, boost::format("Program information log: %s") % buffer.get());
        return false;
    }

    return true;
}
