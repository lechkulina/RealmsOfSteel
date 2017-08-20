/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_ERRORS_H
#define ROS_OPENGL_ERRORS_H

#include <core/Common.h>
#include <GL/glew.h>

namespace ros {
    typedef boost::optional<GLenum> GLenumOpt;

    bool OpenGL_checkForErrors();
}

#endif // ROS_OPENGL_ERRORS_H

