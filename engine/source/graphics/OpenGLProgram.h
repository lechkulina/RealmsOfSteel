/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_PROGRAM_H
#define ROS_OPENGL_PROGRAM_H

#include <GL/glew.h>
#include <graphics/Program.h>

namespace ros {
    class ROS_API OpenGLProgram: public Program {
        public:
            OpenGLProgram();
            virtual ~OpenGLProgram();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool bind();
            virtual void unbind();

        private:
            GLuint handle;

            bool createHandle();
            bool createShaders(const PropertyTree& config);
            bool link();
    };
}

#endif // ROS_OPENGL_PROGRAM_H
