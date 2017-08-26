/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_VERTEX_ARRAY_H
#define ROS_OPENGL_VERTEX_ARRAY_H

#include <GL/glew.h>
#include <graphics/VertexArray.h>

namespace ros {
    class OpenGLVertexArray: public VertexArray {
        public:
            OpenGLVertexArray();
            virtual ~OpenGLVertexArray();

            virtual bool create();
            virtual bool bind();
            virtual void unbind();
            virtual void free();
            virtual bool isCreated() const;

            virtual bool enableAttribute(U32 index, VertexAttributeType type);
            virtual void disableAttribute(U32 index);

        private:
            GLuint handle;
    };
}

#endif // ROS_OPENGL_VERTEX_ARRAY_H

