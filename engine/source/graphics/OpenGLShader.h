/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_SHADER_H
#define ROS_OPENGL_SHADER_H

#include <GL/glew.h>
#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    class ROS_API OpenGLShader: public boost::noncopyable {
        public:
            OpenGLShader();
            virtual ~OpenGLShader();

            bool init(const PropertyTree& config);
            void uninit();

        private:
            friend class OpenGLProgram;

            GLuint handle;

            bool createHandle(const PropertyTree& config);
            bool replaceSource(const PropertyTree& config);
            bool compile();
    };
}

#endif // ROS_OPENGL_SHADER_H

