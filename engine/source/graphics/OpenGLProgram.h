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
    typedef std::map<std::string, OpenGLAttribute> OpenGLAttributeMap;

    struct OpenGLUniform {
        GLuint index;
        GLint size;
        GLenum type;
        GLint location;
    };
    typedef std::map<std::string, OpenGLUniform> OpenGLUniformMap;

    class ROS_API OpenGLProgram: public Program {
        public:
            OpenGLProgram();
            virtual ~OpenGLProgram();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool isValid() const;

            virtual bool bind();
            virtual void unbind();

            virtual const ShaderList& getShaders() const { return shaders; }

            virtual bool setUniform(const char* name, int value);
            virtual bool setUniform(const char* name, const Vector4D& value);
            virtual bool setUniform(const char* name, const Matrix4D& value);

            GLuint getHandle() const { return handle; }
            const OpenGLAttributeMap& getAttributes() const { return attributes; }
            const OpenGLUniformMap& getUniforms() const { return uniforms; }

        private:
            GLuint handle;
            ShaderList shaders;
            OpenGLAttributeMap attributes;
            OpenGLUniformMap uniforms;

            bool createHandle();
            bool createShaders(const PropertyTree& config);
            bool link();
            bool retrieveAttributes();
            bool retrieveUniforms();
    };
}

#endif // ROS_OPENGL_PROGRAM_H

