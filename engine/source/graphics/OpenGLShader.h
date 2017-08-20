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
#include <graphics/Shader.h>

namespace ros {
    class ROS_API OpenGLShader: public Shader {
        public:
            OpenGLShader();
            virtual ~OpenGLShader();

            virtual bool create(ShaderType type);
            virtual bool uploadSource(const fs::path& path);
            virtual bool compile();
            virtual void free();
            virtual bool isCompiled() const;
            virtual const fs::path& getPath() const { return path; }

            GLuint getHandle() const { return handle; }

        private:
            GLuint handle;
            fs::path path;

            void dumpInfoLog();
    };

    typedef boost::shared_ptr<OpenGLShader> OpenGLShaderPtr;
}

#endif // ROS_OPENGL_SHADER_H

