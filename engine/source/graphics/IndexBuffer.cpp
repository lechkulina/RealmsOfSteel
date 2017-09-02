/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/IndexBuffer.h>
#ifdef ROS_USING_OPENGL
    #include "OpenGLIndexBuffer.h"
#endif

ros::IndexBufferPtr ros::IndexBuffer::make() {
    IndexBufferPtr indexBuffer;
#ifdef ROS_USING_OPENGL
    indexBuffer.reset(new OpenGLIndexBuffer());
#endif
    return indexBuffer;
}
