/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ResourceLoader.h>

bool ros::ResourceLoader::init(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Loader name is missing"));
        uninit();
        return false;
    }
    return true;
}

void ros::ResourceLoader::uninit() {
    name.clear();
}
