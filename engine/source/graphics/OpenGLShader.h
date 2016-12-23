/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_SHADER_H
#define ROS_OPENGL_SHADER_H

#include <GL/glew.h>
#include <graphics/Shader.h>

namespace ros {
    class ROS_API OpenGLShader: public Shader {
        public:
            OpenGLShader();
            virtual ~OpenGLShader();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual void* getHandle() { return &handle; }

        private:
            GLuint handle;

            bool createHandle(const PropertyTree& config);
            bool replaceSource(const PropertyTree& config);
            bool compile();
    };
}

#endif // ROS_OPENGL_SHADER_H

