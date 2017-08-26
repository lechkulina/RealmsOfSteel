/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/VertexArray.h>
#ifdef ROS_USING_OPENGL
    #include "OpenGLVertexArray.h"
#endif

ros::VertexArrayPtr ros::VertexArray::make() {
    VertexArrayPtr vertexArray;
#ifdef ROS_USING_OPENGL
    vertexArray.reset(new OpenGLVertexArray());
#endif
    return vertexArray;
}
