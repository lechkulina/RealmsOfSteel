/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/VertexBuffer.h>
#ifdef ROS_USING_OPENGL
    #include "OpenGLVertexBuffer.h"
#endif

ros::VertexBufferPtr ros::VertexBuffer::make() {
    VertexBufferPtr vertexBuffer;
#ifdef ROS_USING_OPENGL
    vertexBuffer.reset(new OpenGLVertexBuffer());
#endif
    return vertexBuffer;
}
