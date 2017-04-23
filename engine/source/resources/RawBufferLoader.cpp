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
    return true;  // this loader acts as a fallback and it can basically load any type of resource as raw bits of data
}

ros::ResourcePtr ros::RawBufferLoader::loadResource(const std::string& name) {
    return FileSystem::getInstance()->readFile(name);
}
