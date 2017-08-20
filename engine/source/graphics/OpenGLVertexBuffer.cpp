/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <Application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLVertexBuffer.h"

static const struct VertexBufferUsageGLenumMapping {
    ros::VertexBufferUsage first;
    GLenum second;
} vertexBufferUsageGLenumMappings[] = {
    {ros::VertexBufferUsage_StreamDraw, GL_STREAM_DRAW},
    {ros::VertexBufferUsage_StreamRead, GL_STREAM_READ},
    {ros::VertexBufferUsage_StreamCopy, GL_STREAM_COPY},
    {ros::VertexBufferUsage_StaticDraw, GL_STATIC_DRAW},
    {ros::VertexBufferUsage_StaticRead, GL_STATIC_READ},
    {ros::VertexBufferUsage_StaticCopy, GL_STATIC_COPY},
    {ros::VertexBufferUsage_DynamicDraw, GL_DYNAMIC_DRAW},
    {ros::VertexBufferUsage_DynamicRead, GL_DYNAMIC_READ},
    {ros::VertexBufferUsage_DynamicCopy, GL_DYNAMIC_COPY}
};

static ros::GLenumOpt VertexBufferUsage_toGLenum(ros::VertexBufferUsage usage) {
    const VertexBufferUsageGLenumMapping* iter = std::find_if(
        boost::begin(vertexBufferUsageGLenumMappings), boost::end(vertexBufferUsageGLenumMappings),
        boost::bind(&VertexBufferUsageGLenumMapping::first, _1) == usage);
    if (iter != boost::end(vertexBufferUsageGLenumMappings)) {
        return iter->second;
    }
    return ros::GLenumOpt();
}

static const struct VertexBufferAccessGLenumMapping {
    ros::VertexBufferAccess first;
    GLenum second;
} vertexBufferAccessGLenumMappings[] = {
    {ros::VertexBufferAccess_Read, GL_READ_ONLY},
    {ros::VertexBufferAccess_Write, GL_WRITE_ONLY},
    {ros::VertexBufferAccess_ReadWrite, GL_READ_WRITE}
};

static ros::GLenumOpt VertexBufferAccess_toGLenum(ros::VertexBufferAccess access) {
    const VertexBufferAccessGLenumMapping* iter = std::find_if(
        boost::begin(vertexBufferAccessGLenumMappings), boost::end(vertexBufferAccessGLenumMappings),
        boost::bind(&VertexBufferAccessGLenumMapping::first, _1) == access);
    if (iter != boost::end(vertexBufferAccessGLenumMappings)) {
        return iter->second;
    }
    return ros::GLenumOpt();
}

ros::OpenGLVertexBuffer::OpenGLVertexBuffer()
    : handle(0)
    , size(0) {
}

ros::OpenGLVertexBuffer::~OpenGLVertexBuffer() {
    free();
}

bool ros::OpenGLVertexBuffer::create() {
    free();
    glGenBuffers(1, &handle);
    return !OpenGL_checkForErrors();
}

bool ros::OpenGLVertexBuffer::bind() {
    glBindBuffer(GL_ARRAY_BUFFER, handle);
    return !OpenGL_checkForErrors();
}

void ros::OpenGLVertexBuffer::unbind() {
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

bool ros::OpenGLVertexBuffer::allocate(U32 size, void* data /*= ROS_NULL */, VertexBufferUsage usage /*= VertexBufferUsage_StaticDraw*/) {
    GLenumOpt glUsage = VertexBufferUsage_toGLenum(usage);
    if (!glUsage) {
        ROS_ERROR(boost::format("Unknown vertex buffer usage %d") % usage);
        return false;
    }
    glBufferData(GL_ARRAY_BUFFER, size, data, *glUsage);
    if (OpenGL_checkForErrors()) {
        return false;
    }
    this->size = size;
    this->usage = usage;
    return true;
}

void ros::OpenGLVertexBuffer::free() {
    if (handle) {
        glDeleteBuffers(1, &handle);
        handle = 0;
    }
    size = 0;
    usage.reset();
}

bool ros::OpenGLVertexBuffer::isAllocated() const {
    return glIsBuffer(handle);
}

void* ros::OpenGLVertexBuffer::map(VertexBufferAccess access /*= VertexBufferAccess_Write*/) {
    GLenumOpt glAccess = VertexBufferAccess_toGLenum(access);
    if (!glAccess) {
        ROS_ERROR(boost::format("Unknown vertex buffer access %d") % access);
        return ROS_NULL;
    }
    void* data = glMapBuffer(GL_ARRAY_BUFFER, *glAccess);
    if (!data || OpenGL_checkForErrors()) {
        return ROS_NULL;
    }
    return data;
}

void ros::OpenGLVertexBuffer::unmap() {
    glUnmapBuffer(GL_ARRAY_BUFFER);
}
