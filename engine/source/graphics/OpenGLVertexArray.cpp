/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLVertexArray.h"

ros::OpenGLVertexArray::OpenGLVertexArray()
    : handle(0) {
}

ros::OpenGLVertexArray::~OpenGLVertexArray() {
    free();
}

bool ros::OpenGLVertexArray::create() {
    free();
    glGenVertexArrays(1, &handle);
    return !OpenGL_checkForErrors();
}

bool ros::OpenGLVertexArray::bind() {
    glBindVertexArray(handle);
    return !OpenGL_checkForErrors();
}

void ros::OpenGLVertexArray::unbind() {
    glBindVertexArray(0);
}

void ros::OpenGLVertexArray::free() {
    if (handle) {
        glDeleteVertexArrays(1, &handle);
        handle = 0;
    }
}

bool ros::OpenGLVertexArray::isCreated() const {
    return glIsVertexArray(handle);
}

bool ros::OpenGLVertexArray::enableAttribute(U32 index, VertexAttributeType type) {
    GLint size = 0;
    switch(type) {
        case VertexAttributeType_Vec2:
            size = 2;
            break;
        case VertexAttributeType_Vec3:
            size = 3;
            break;
        case VertexAttributeType_Vec4:
            size = 4;
            break;
        default:
            ROS_ERROR(boost::format("Unknown vertex attribute type %d") % type);
            return false;
    }
    glVertexAttribPointer(index, size, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(index);
    return !OpenGL_checkForErrors();
}

void ros::OpenGLVertexArray::disableAttribute(U32 index) {
    glDisableVertexAttribArray(index);
}
