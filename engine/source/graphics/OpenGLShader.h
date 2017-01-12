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

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool isValid() const;

            virtual const std::string& getPath() const { return path; }

            GLuint getHandle() const { return handle; }

        private:
            GLuint handle;
            std::string path;

            bool createHandle(const PropertyTree& config);
            bool replaceSource(const PropertyTree& config);
            bool compile();
    };

    typedef boost::shared_ptr<OpenGLShader> OpenGLShaderPtr;
}

#endif // ROS_OPENGL_SHADER_H

