/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <fstream>
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <boost/scoped_array.hpp>
#include <Application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLShader.h"

namespace {
    typedef boost::optional<GLenum> ShaderTypeOpt;

    const struct ShaderTypeMapping {
        const char* str;
        GLenum type;
    } shaderTypeMappings[] = {
        {"compute", GL_COMPUTE_SHADER},
        {"vertex", GL_VERTEX_SHADER},
        {"tessellation-control",  GL_TESS_CONTROL_SHADER},
        {"tessellation-evaluation",  GL_TESS_EVALUATION_SHADER},
        {"geometry",  GL_GEOMETRY_SHADER},
        {"fragment",  GL_FRAGMENT_SHADER}
    };

    ShaderTypeOpt ShaderType_fromString(const char* str) {
        const ShaderTypeMapping* iter = std::find_if(boost::begin(shaderTypeMappings), boost::end(shaderTypeMappings),
            boost::bind(strcmp, boost::bind(&ShaderTypeMapping::str, _1), str) == 0);
        if (iter != boost::end(shaderTypeMappings)) {
            return iter->type;
        }
        return ShaderTypeOpt();
    }
}

ros::OpenGLShader::OpenGLShader()
    : handle(0) {
}

ros::OpenGLShader::~OpenGLShader() {
    uninit();
}

bool ros::OpenGLShader::init(const PropertyTree& config) {
    if (!Shader::init(config) || !createHandle(config) || !replaceSource(config) || !compile()) {
        uninit();
        return false;
    }
    return true;
}

void ros::OpenGLShader::uninit() {
    path.clear();
    if (handle) {
        glDeleteShader(handle);
        handle = 0;
    }
    Shader::uninit();
}

bool ros::OpenGLShader::isValid() const {
    return glIsShader(handle);
}

bool ros::OpenGLShader::createHandle(const PropertyTree& config) {
    StringOpt typeStr = config.get_optional<std::string>("type");
    if (!typeStr) {
        Logger::report(LogLevel_Error, boost::format("Missing type property in shader %s") % getName());
        return false;
    }
    ShaderTypeOpt type = ShaderType_fromString(typeStr->c_str());
    if (!type) {
        Logger::report(LogLevel_Error, boost::format("Unknown type %s property used for shader %s")  % getName() % (*typeStr));
        return false;
    }

    handle = glCreateShader(*type);
    if (!handle || OpenGL_checkForErrors()) {
        return false;
    }

    return true;
}

bool ros::OpenGLShader::replaceSource(const PropertyTree& config) {
    StringOpt path = config.get_optional<std::string>("path");
    if (!path) {
        Logger::report(LogLevel_Error, boost::format("Missing path property in shader %s") % getName());
        return false;
    }

    this->path = *path;
    std::ifstream stream(path->c_str(), std::ios::in);
    if (!stream.good()) {
        Logger::report(LogLevel_Error, boost::format("Unable to open source file %s for shader %s") % (*path) % getName());
        return false;
    }

    stream.seekg(0, std::ios::end);
    GLint length = stream.tellg();
    stream.seekg(0, std::ios::beg);
    if (length == 0) {
        Logger::report(LogLevel_Error, boost::format("Source file %s used for shaders %s is empty") % (*path) % getName());
        return false;
    }

    boost::scoped_array<GLchar> buffer(new GLchar[length]);
    memset(buffer.get(), 0, sizeof(GLchar) * length);
    stream.read(buffer.get(), length);
    if (stream.bad()) {
        Logger::report(LogLevel_Error, boost::format("Failed to read %d bytes from source file %s used for shader %s")
                            % length % (*path) % getName());
        return false;
    }

    GLchar* sources[] = {buffer.get()};
    GLint lengths[] = {length};
    glShaderSource(handle, 1, sources, lengths);
    if (OpenGL_checkForErrors()) {
        return false;
    }

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
        Logger::report(LogLevel_Error, boost::format("Failed to compile shader %s using source file %s") % getName() % path);

        GLint length = 0;
        glGetShaderiv(handle, GL_INFO_LOG_LENGTH, &length);
        if (length == 0 || OpenGL_checkForErrors()) {
            return false;
        }

        boost::scoped_array<GLchar> buffer(new GLchar[length]);
        memset(buffer.get(), 0, sizeof(GLchar) * length);
        glGetShaderInfoLog(handle, length, NULL, buffer.get());
        if (OpenGL_checkForErrors()) {
            return false;
        }

        Logger::report(LogLevel_Debug, boost::format("Shader %s information log: %s") % getName() % buffer.get());
        return false;
    }

    Logger::report(LogLevel_Trace, boost::format("Shader %s compiled successfully using source file %s") % getName() % path);
    return true;
}
