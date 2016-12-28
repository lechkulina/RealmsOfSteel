/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <graphics/ShadersManager.h>
#include <graphics/Program.h>

bool ros::Program::init(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Program name is missing"));
        uninit();
        return false;
    }
    return true;
}

void ros::Program::uninit() {
    name.clear();
    shaders.clear();
}

bool ros::Program::initShaders(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "shader") {
            continue;
        }
        ShaderPtr shader = ShadersManager::getInstance()->provide(iter->second);
        if (!shader || !attachShader(shader)) {
            uninit();
            return false;
        }
        shaders.push_back(shader);
    }
    return true;
}
