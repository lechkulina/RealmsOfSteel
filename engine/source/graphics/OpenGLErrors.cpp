/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <GL/glew.h>
#include <Application/Logger.h>
#include "OpenGLErrors.h"

namespace {
    const struct ErrorMapping {
        const char* str;
        GLenum error;
    } errorMappings[] = {
        {"Invalid enumeration", GL_INVALID_ENUM},
        {"Invalid value", GL_INVALID_VALUE},
        {"Invalid operation", GL_INVALID_OPERATION},
        {"Stack overflow", GL_STACK_OVERFLOW},
        {"Stack underflow", GL_STACK_UNDERFLOW},
        {"Out of memory", GL_OUT_OF_MEMORY}
    };

    const char* Error_toString(GLenum error) {
        const ErrorMapping* iter = std::find_if(boost::begin(errorMappings), boost::end(errorMappings),
            boost::bind(&ErrorMapping::error, _1) == error);
        if (iter != boost::end(errorMappings)) {
            return iter->str;
        }
        return ROS_NULL;
    }
}

bool ros::OpenGL_checkForErrors() {
    bool found = false;
    for (GLenum error = glGetError(); error != GL_NO_ERROR; error = glGetError()) {
        const char* str = Error_toString(error);
        ROS_ERROR(boost::format("OpenGL error occured: %d (%s)") % error % (str ? str : "Unknown"));
        found = true;
    }
    return found;
}

