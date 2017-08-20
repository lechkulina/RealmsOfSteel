/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <glm/gtc/type_ptr.hpp>
#include <application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLShader.h"
#include "OpenGLProgram.h"

ros::OpenGLProgram::OpenGLProgram()
    : handle(0) {
}

ros::OpenGLProgram::~OpenGLProgram() {
    free();
}

bool ros::OpenGLProgram::create() {
    handle = glCreateProgram();
    if (!handle || OpenGL_checkForErrors()) {
        return false;
    }
    return true;
}

bool ros::OpenGLProgram::attachShader(ShaderPtr shader) {
    OpenGLShaderPtr glShader = boost::dynamic_pointer_cast<OpenGLShader>(shader);
    if (!glIsShader(glShader->getHandle())) {
        ROS_ERROR(boost::format("Unable to attach non OpenGL shader object to program"));
        return false;
    }
    glAttachShader(handle, glShader->getHandle());
    return !OpenGL_checkForErrors();
}

bool ros::OpenGLProgram::link() {
    GLint status = GL_FALSE;
    glLinkProgram(handle);
    glGetProgramiv(handle, GL_LINK_STATUS, &status);
    if (OpenGL_checkForErrors()) {
        return false;
    }
    if (!status) {
        ROS_ERROR(boost::format("Failed to link a program"));
        dumpInfoLog();
        return false;
    }
    return retrieveAttributes() && retrieveUniforms();
}

void ros::OpenGLProgram::free() {
    if (handle) {
        glDeleteProgram(handle);
        handle = 0;
    }
    uniforms.clear();
    attributes.clear();
}

bool ros::OpenGLProgram::isLinked() const {
    return glIsProgram(handle);
}

bool ros::OpenGLProgram::bind() {
    glUseProgram(handle);
    return !OpenGL_checkForErrors();
}

void ros::OpenGLProgram::unbind() {
    glUseProgram(0);
}

bool ros::OpenGLProgram::setUniform(const char* name, int value) {
    OpenGLUniformsMap::const_iterator iter = uniforms.find(name);
    if (iter == uniforms.end()) {
        ROS_ERROR(boost::format("Failed to find uniform %s") % name);
        return false;
    }
    glUniform1i(iter->second.location, value);
    return OpenGL_checkForErrors();
}

bool ros::OpenGLProgram::setUniform(const char* name, const glm::vec4& value) {
    OpenGLUniformsMap::const_iterator iter = uniforms.find(name);
    if (iter == uniforms.end()) {
        ROS_ERROR(boost::format("Failed to find uniform %s") % name);
        return false;
    }
    glUniform4fv(iter->second.location, 1, glm::value_ptr(value));
    return OpenGL_checkForErrors();
}

bool ros::OpenGLProgram::setUniform(const char* name, const glm::mat4& value) {
    OpenGLUniformsMap::const_iterator iter = uniforms.find(name);
    if (iter == uniforms.end()) {
        ROS_ERROR(boost::format("Failed to find uniform %s") % name);
        return false;
    }
    glUniformMatrix4fv(iter->second.location, 1, GL_FALSE, glm::value_ptr(value));
    return OpenGL_checkForErrors();
}

void ros::OpenGLProgram::dumpInfoLog() {
    GLint length = 0;
    glGetProgramiv(handle, GL_INFO_LOG_LENGTH, &length);
    if (length == 0 || OpenGL_checkForErrors()) {
        return;
    }

    boost::scoped_array<GLchar> buffer(new GLchar[length]);
    memset(buffer.get(), 0, sizeof(GLchar) * length);
    glGetProgramInfoLog(handle, length, NULL, buffer.get());
    if (OpenGL_checkForErrors()) {
        return;
    }

    ROS_DEBUG(boost::format("Lastest program information log: %s") % buffer.get());
}

bool ros::OpenGLProgram::retrieveAttributes() {
    GLint count = 0;
    GLsizei maxLength = 0;
    glGetProgramiv(handle, GL_ACTIVE_ATTRIBUTES, &count);
    glGetProgramiv(handle, GL_ACTIVE_ATTRIBUTE_MAX_LENGTH, &maxLength);
    if (maxLength == 0 || OpenGL_checkForErrors()) {
        return false;
    }

    boost::scoped_array<GLchar> buffer(new GLchar[maxLength]);
    memset(buffer.get(), 0, sizeof(GLchar) * maxLength);

    attributes.clear();
    for (GLint index = 0; index < count; ++index) {
        GLint length = 0;
        OpenGLAttribute attribute;
        memset(&attribute, 0, sizeof(OpenGLAttribute));
        attribute.index = index;
        glGetActiveAttrib(handle, index, maxLength, &length, &attribute.size, &attribute.type, buffer.get());
        if (OpenGL_checkForErrors()) {
            return false;
        }
        if (length == 0) {
            ROS_WARNING(boost::format("Failed to retrieve attribute name at index %d") % index);
            continue;
        }
        attributes[buffer.get()] = attribute;
    }

    return true;
}

bool ros::OpenGLProgram::retrieveUniforms() {
    GLint count = 0;
    GLint maxLength = 0;
    glGetProgramiv(handle, GL_ACTIVE_UNIFORMS, &count);
    glGetProgramiv(handle, GL_ACTIVE_UNIFORM_MAX_LENGTH, &maxLength);
    if (maxLength == 0 || OpenGL_checkForErrors()) {
        return false;
    }

    boost::scoped_array<GLchar> buffer(new GLchar[maxLength]);
    memset(buffer.get(), 0, sizeof(GLchar) * maxLength);

    uniforms.clear();
    for (GLint index = 0; index < count; ++index) {
        GLint length = 0;
        OpenGLUniform uniform;
        memset(&uniform, 0, sizeof(OpenGLUniform));
        uniform.index = index;
        glGetActiveUniform(handle, index, maxLength, &length, &uniform.size, &uniform.type, buffer.get());
        if (OpenGL_checkForErrors()) {
            return false;
        }
        if (length == 0) {
            ROS_WARNING(boost::format("Failed to retrieve uniform name at index %d") % index);
            continue;
        }
        uniform.location = glGetUniformLocation(handle, buffer.get());
        if (uniform.location == -1 || OpenGL_checkForErrors()) {
            return false;
        }
        uniforms[buffer.get()] = uniform;
    }

    return true;
}
