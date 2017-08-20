/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <Application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLShader.h"

static const struct ShaderTypeGLenumMapping {
    ros::ShaderType first;
    GLenum second;
} shaderTypeGLenumMappings[] = {
    {ros::ShaderType_Compute, GL_COMPUTE_SHADER},
    {ros::ShaderType_Vertex, GL_VERTEX_SHADER},
    {ros::ShaderType_TessControl, GL_TESS_CONTROL_SHADER},
    {ros::ShaderType_TessEvaluation, GL_TESS_EVALUATION_SHADER},
    {ros::ShaderType_Geometry, GL_GEOMETRY_SHADER},
    {ros::ShaderType_Fragment, GL_FRAGMENT_SHADER}
};

static ros::GLenumOpt ShaderType_toGLenum(ros::ShaderType type) {
    const ShaderTypeGLenumMapping* iter = std::find_if(
        boost::begin(shaderTypeGLenumMappings), boost::end(shaderTypeGLenumMappings),
        boost::bind(&ShaderTypeGLenumMapping::first, _1) == type);
    if (iter != boost::end(shaderTypeGLenumMappings)) {
        return iter->second;
    }
    return ros::GLenumOpt();
}

ros::OpenGLShader::OpenGLShader()
    : handle(0) {
}

ros::OpenGLShader::~OpenGLShader() {
    free();
}

bool ros::OpenGLShader::create(ShaderType type) {
    free();
    GLenumOpt glType = ShaderType_toGLenum(type);
    if (!glType) {
        ROS_ERROR(boost::format("Unknown shader type %d") % type);
        return false;
    }
    handle = glCreateShader(*glType);
    if (!handle || OpenGL_checkForErrors()) {
        return false;
    }
    return true;
}

bool ros::OpenGLShader::uploadSource(const fs::path& path) {
    std::ifstream stream(path.string().c_str(), std::ios::in);
    if (!stream.good()) {
        ROS_ERROR(boost::format("Unable to open shader source file %s") % path);
        return false;
    }

    stream.seekg(0, std::ios::end);
    GLint length = stream.tellg();
    stream.seekg(0, std::ios::beg);
    if (length == 0) {
        ROS_ERROR(boost::format("Shader source file %s is empty") % path);
        return false;
    }

    boost::scoped_array<GLchar> buffer(new GLchar[length]);
    memset(buffer.get(), 0, sizeof(GLchar) * length);
    stream.read(buffer.get(), length);
    if (stream.bad()) {
        ROS_ERROR(boost::format("Failed to read %d bytes from shader source file %s") % length % path);
        return false;
    }

    GLchar* sources[] = {buffer.get()};
    GLint lengths[] = {length};
    glShaderSource(handle, 1, sources, lengths);
    if (OpenGL_checkForErrors()) {
        return false;
    }
    this->path = path;

    return true;
}

bool ros::OpenGLShader::compile() {
    GLint status = GL_FALSE;
    glCompileShader(handle);
    glGetShaderiv(handle, GL_COMPILE_STATUS, &status);
    if (OpenGL_checkForErrors()) {
        return false;
    }
    if (!status) {
        ROS_ERROR(boost::format("Failed to compile shader %s") % path);
        dumpInfoLog();
        return false;
    }
    return true;
}

void ros::OpenGLShader::free() {
    if (handle) {
        glDeleteShader(handle);
        handle = 0;
    }
    path.clear();
}

bool ros::OpenGLShader::isCompiled() const {
    return glIsShader(handle);
}

void ros::OpenGLShader::dumpInfoLog() {
    GLint length = 0;
    glGetShaderiv(handle, GL_INFO_LOG_LENGTH, &length);
    if (length == 0 || OpenGL_checkForErrors()) {
        return;
    }

    boost::scoped_array<GLchar> buffer(new GLchar[length]);
    memset(buffer.get(), 0, sizeof(GLchar) * length);
    glGetShaderInfoLog(handle, length, NULL, buffer.get());
    if (OpenGL_checkForErrors()) {
        return;
    }

    ROS_DEBUG(boost::format("Lastest shader %s information log: %s") % path % buffer.get());
}
