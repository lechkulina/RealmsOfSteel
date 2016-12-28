/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <graphics/Shader.h>

bool ros::Shader::init(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Shader name is missing"));
        uninit();
        return false;
    }
    return true;
}

void ros::Shader::uninit() {
    name.clear();
}
