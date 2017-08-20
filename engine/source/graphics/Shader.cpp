/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Shader.h>
#ifdef ROS_USING_OPENGL
    #include "OpenGLShader.h"
#endif

ros::ShaderPtr ros::Shader::make() {
    ShaderPtr shader;
#ifdef ROS_USING_OPENGL
    shader.reset(new OpenGLShader());
#endif
    return shader;
}
