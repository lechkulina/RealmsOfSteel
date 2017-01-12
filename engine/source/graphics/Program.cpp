/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#include <graphics/Program.h>

bool ros::Program::init(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Program name is missing"));
        uninit();
        return false;
    }

    if (!Application::getInstance()->getShaderManager().initShaders(config, shaders)) {
        uninit();
        return false;
    }

    return true;
}

void ros::Program::uninit() {
    name.clear();
    shaders.clear();
}

 bool ros::Program::attachShaders() {
     for (ShaderList::iterator iter = shaders.begin(); iter != shaders.end(); ++iter) {
         if (!attachShader(*iter)) {
             return false;
         }
     }
     return true;
 }
