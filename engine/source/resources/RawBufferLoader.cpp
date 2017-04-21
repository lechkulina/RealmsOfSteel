/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "RawBufferLoader.h"

bool ros::RawBufferLoader::isLoadable(const std::string&) const {
    return true;
}

ros::ResourcePtr ros::RawBufferLoader::load(const std::string& resourceName) {
    return FileSystem::getInstance()->readFile(resourceName);
}
