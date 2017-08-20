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
    struct OpenGLAttribute {
        GLuint index;
        GLint size;
        GLenum type;
    };
    typedef std::map<std::string, OpenGLAttribute> OpenGLAttributesMap;

    struct OpenGLUniform {
        GLuint index;
        GLint size;
        GLenum type;
        GLint location;
    };
    typedef std::map<std::string, OpenGLUniform> OpenGLUniformsMap;

    class ROS_API OpenGLProgram: public Program {
        public:
            OpenGLProgram();
            virtual ~OpenGLProgram();

            virtual bool create();
            virtual bool attachShader(ShaderPtr shader);
            virtual bool link();
            virtual void free();
            virtual bool isLinked() const;

            virtual bool bind();
            virtual void unbind();

            virtual bool setUniform(const char* name, int value);
            virtual bool setUniform(const char* name, const glm::vec4& value);
            virtual bool setUniform(const char* name, const glm::mat4& value);

            GLuint getHandle() const { return handle; }

        private:
            GLuint handle;
            OpenGLAttributesMap attributes;
            OpenGLUniformsMap uniforms;

            void dumpInfoLog();
            bool retrieveAttributes();
            bool retrieveUniforms();
    };
}

#endif // ROS_OPENGL_PROGRAM_H

