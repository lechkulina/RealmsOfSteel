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
#include "OpenGLShader.h"

namespace ros {
    struct OpenGLAttribute {
        GLint size;
        GLenum type;
        GLuint index;
    };
    typedef std::map<std::string, OpenGLAttribute> OpenGLAttributeMap;

    class ROS_API OpenGLProgram: public Program {
        public:
            OpenGLProgram();
            virtual ~OpenGLProgram();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool bind();
            virtual void unbind();

            GLuint getHandle() const { return handle; }
            const OpenGLShaderList& getShaders() const { return shaders; }
            const OpenGLAttributeMap& getAttributes() const { return attributes; }

        private:
            GLuint handle;
            OpenGLShaderList shaders;
            OpenGLAttributeMap attributes;

            bool createHandle();
            bool createShaders(const PropertyTree& config);
            bool link();
            bool retrieveAttributes();
    };
}

#endif // ROS_OPENGL_PROGRAM_H

