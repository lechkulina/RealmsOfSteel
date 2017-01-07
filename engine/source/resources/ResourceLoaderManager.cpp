/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ResourceLoaderManager.h>

ros::ResourceLoaderManagerPtr ros::ResourceLoaderManager::manager;

ros::ResourceLoaderManagerPtr ros::ResourceLoaderManager::initInstance(const PropertyTree& config) {
    if (manager) {
        return manager;
    }

    manager.reset(new ResourceLoaderManager());
    if (manager && !manager->init(config)) {
        manager.reset();
    }

    return manager;
}

ros::ResourceLoaderManager::~ResourceLoaderManager() {
    uninit();
}

bool ros::ResourceLoaderManager::init(const PropertyTree& config) {
    uninit();

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "loader" && !initLoader(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::ResourceLoaderManager::uninit() {
    loaders.clear();
    factory.clear();
}

ros::ResourceLoaderPtr ros::ResourceLoaderManager::initLoader(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Loader name is missing"));
        return ResourceLoaderPtr();
    }

    ResourceLoaderMap::iterator iter = loaders.find(name);
    if (iter != loaders.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Loader %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    StringOpt type = config.get_optional<std::string>("type");
    if (!type) {
        Logger::report(LogLevel_Error, boost::format("Missing path property in loader %s") % name);
        return ResourceLoaderPtr();
    }
    ResourceLoaderPtr loader(factory.create(type->c_str()));
    if (!loader) {
        Logger::report(LogLevel_Error, boost::format("Unsupported type %s used for loader %s") % type % name);
        return ResourceLoaderPtr();
    }
    if (!loader->init(config)) {
        return ResourceLoaderPtr();
    }

    loaders[name] = loader;
    return loader;
}
