/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/scoped_array.hpp>
#include <boost/pointer_cast.hpp>
#include <application/Logger.h>
#include "OpenGLErrors.h"
#include "OpenGLShader.h"
#include "OpenGLProgram.h"

ros::OpenGLProgram::OpenGLProgram()
    : handle(0) {
}

ros::OpenGLProgram::~OpenGLProgram() {
    uninit();
}

bool ros::OpenGLProgram::init(const PropertyTree& config) {
    if (!Program::init(config) || !createHandle() || !attachShaders() || !link() || !retrieveAttributes() || !retrieveUniforms()) {
        uninit();
        return false;
    }
    return true;
}

void ros::OpenGLProgram::uninit() {
    uniforms.clear();
    attributes.clear();
    if (handle) {
        glDeleteProgram(handle);
        handle = 0;
    }
    Program::uninit();
}

bool ros::OpenGLProgram::isValid() const {
    return glIsProgram(handle);
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

bool ros::OpenGLProgram::attachShader(ShaderPtr shader) {
    OpenGLShaderPtr cast = boost::static_pointer_cast<OpenGLShader>(shader);
    if (!glIsShader(cast->getHandle())) {
        ROS_ERROR(boost::format("Unable to attach non-shader object %s to program %s") % shader->getName() % getName());
        return false;
    }
    glAttachShader(handle, cast->getHandle());
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
        ROS_ERROR(boost::format("Failed to link program %s") % getName());

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

        ROS_DEBUG(boost::format("Program %s information log: %s") % getName() % buffer.get());
        return false;
    }

    ROS_TRACE(boost::format("Program %s linked successfully") % getName());
    return true;
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
            ROS_WARNING(boost::format("Failed to retrieve attribute name at index %d in program %s") % index % getName());
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
            ROS_WARNING(boost::format("Failed to retrieve uniform name at index %d in program %s") % index % getName());
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

bool ros::OpenGLProgram::setUniform(const char* name, int value) {
    OpenGLUniformMap::const_iterator iter = uniforms.find(name);
    if (iter == uniforms.end()) {
        ROS_ERROR(boost::format("Failed to find uniform with name %s in program %s") % name % getName());
        return false;
    }
    glUniform1i(iter->second.location, value);
    return OpenGL_checkForErrors();
}

bool ros::OpenGLProgram::setUniform(const char* name, const Vector4D& value) {
    OpenGLUniformMap::const_iterator iter = uniforms.find(name);
    if (iter == uniforms.end()) {
        ROS_ERROR(boost::format("Failed to find uniform with name %s in program %s") % name % getName());
        return false;
    }
    glUniform4f(iter->second.location, value.x, value.y, value.z, value.w);
    return OpenGL_checkForErrors();
}

bool ros::OpenGLProgram::setUniform(const char* name, const Matrix4D& value) {
    OpenGLUniformMap::const_iterator iter = uniforms.find(name);
    if (iter == uniforms.end()) {
        ROS_ERROR(boost::format("Failed to find uniform with name %s in program %s") % name % getName());
        return false;
    }
    const GLfloat data[16] = {
        value.m11, value.m21, value.m31, value.m41,
        value.m12, value.m22, value.m32, value.m42,
        value.m13, value.m23, value.m33, value.m43,
        value.m14, value.m24, value.m34, value.m44
    };
    glUniformMatrix4fv(iter->second.location, 1, GL_FALSE, data);
    return OpenGL_checkForErrors();
}
