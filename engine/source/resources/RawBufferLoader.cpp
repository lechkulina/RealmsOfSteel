/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include "RawBufferLoader.h"

ros::RawBufferLoader::~RawBufferLoader() {
    uninit();
}

bool ros::RawBufferLoader::init(const PropertyTree &config) {
    uninit();
    if (!BufferLoader::init(config)) {
        uninit();
        return false;
    }

    loadable = config.get("loadable", "");

    return true;
}

void ros::RawBufferLoader::uninit() {
    loadable.assign("");
    BufferLoader::uninit();
}

bool ros::RawBufferLoader::isLoadable(const std::string& name) const {
    if (loadable.empty()) {
        return true;
    }
    return boost::regex_match(name, loadable);
}

ros::BufferPtr ros::RawBufferLoader::loadBuffer(RawBufferPtr src) {
    return src;
}
