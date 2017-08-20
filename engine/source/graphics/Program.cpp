/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Program.h>
#ifdef ROS_USING_OPENGL
    #include "OpenGLProgram.h"
#endif

ros::ProgramPtr ros::Program::make() {
    ProgramPtr program;
#ifdef ROS_USING_OPENGL
    program.reset(new OpenGLProgram());
#endif
    return program;
}
