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
#include "OpenGLIndexBuffer.h"

static const struct IndexBufferUsageGLenumMapping {
    ros::IndexBufferUsage first;
    GLenum second;
} indexBufferUsageGLenumMappings[] = {
    {ros::IndexBufferUsage_StreamDraw, GL_STREAM_DRAW},
    {ros::IndexBufferUsage_StreamRead, GL_STREAM_READ},
    {ros::IndexBufferUsage_StreamCopy, GL_STREAM_COPY},
    {ros::IndexBufferUsage_StaticDraw, GL_STATIC_DRAW},
    {ros::IndexBufferUsage_StaticRead, GL_STATIC_READ},
    {ros::IndexBufferUsage_StaticCopy, GL_STATIC_COPY},
    {ros::IndexBufferUsage_DynamicDraw, GL_DYNAMIC_DRAW},
    {ros::IndexBufferUsage_DynamicRead, GL_DYNAMIC_READ},
    {ros::IndexBufferUsage_DynamicCopy, GL_DYNAMIC_COPY}
};

static ros::GLenumOpt IndexBufferUsage_toGLenum(ros::IndexBufferUsage usage) {
    const IndexBufferUsageGLenumMapping* iter = std::find_if(
        boost::begin(indexBufferUsageGLenumMappings), boost::end(indexBufferUsageGLenumMappings),
        boost::bind(&IndexBufferUsageGLenumMapping::first, _1) == usage);
    if (iter != boost::end(indexBufferUsageGLenumMappings)) {
        return iter->second;
    }
    return ros::GLenumOpt();
}

static const struct IndexBufferAccessGLenumMapping {
    ros::IndexBufferAccess first;
    GLenum second;
} indexBufferAccessGLenumMappings[] = {
    {ros::IndexBufferAccess_Read, GL_READ_ONLY},
    {ros::IndexBufferAccess_Write, GL_WRITE_ONLY},
    {ros::IndexBufferAccess_ReadWrite, GL_READ_WRITE}
};

static ros::GLenumOpt IndexBufferAccess_toGLenum(ros::IndexBufferAccess access) {
    const IndexBufferAccessGLenumMapping* iter = std::find_if(
        boost::begin(indexBufferAccessGLenumMappings), boost::end(indexBufferAccessGLenumMappings),
        boost::bind(&IndexBufferAccessGLenumMapping::first, _1) == access);
    if (iter != boost::end(indexBufferAccessGLenumMappings)) {
        return iter->second;
    }
    return ros::GLenumOpt();
}

ros::OpenGLIndexBuffer::OpenGLIndexBuffer()
    : handle(0)
    , indices(0) {
}

ros::OpenGLIndexBuffer::~OpenGLIndexBuffer() {
    free();
}

bool ros::OpenGLIndexBuffer::create() {
    free();
    glGenBuffers(1, &handle);
    return !OpenGL_checkForErrors();
}

bool ros::OpenGLIndexBuffer::bind() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle);
    return !OpenGL_checkForErrors();
}

void ros::OpenGLIndexBuffer::unbind() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool ros::OpenGLIndexBuffer::allocate(U32 indices, U32* data /*= ROS_NULL */, IndexBufferUsage usage /*= IndexBufferUsage_StaticDraw*/) {
    GLenumOpt glUsage = IndexBufferUsage_toGLenum(usage);
    if (!glUsage) {
        ROS_ERROR(boost::format("Unknown index buffer usage %d") % usage);
        return false;
    }
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices * sizeof(U32), data, *glUsage);
    if (OpenGL_checkForErrors()) {
        return false;
    }
    this->indices = indices;
    this->usage = usage;
    return true;
}

void ros::OpenGLIndexBuffer::free() {
    if (handle) {
        glDeleteBuffers(1, &handle);
        handle = 0;
    }
    indices = 0;
    usage.reset();
}

bool ros::OpenGLIndexBuffer::isAllocated() const {
    return glIsBuffer(handle);
}

ros::U32* ros::OpenGLIndexBuffer::map(IndexBufferAccess access /*= IndexBufferAccess_Write*/) {
    GLenumOpt glAccess = IndexBufferAccess_toGLenum(access);
    if (!glAccess) {
        ROS_ERROR(boost::format("Unknown index buffer access %d") % access);
        return ROS_NULL;
    }
    U32* data = (U32*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, *glAccess);
    if (!data || OpenGL_checkForErrors()) {
        return ROS_NULL;
    }
    return data;
}

void ros::OpenGLIndexBuffer::unmap() {
    glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
}

