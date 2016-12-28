/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#include <graphics/ShadersManager.h>

ros::ShadersManager* ros::ShadersManager::getInstance() {
    static ShadersManager instance;
    return &instance;
}

bool ros::ShadersManager::prepare(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "shader" && !provide(iter->second)) {
            clear();
            return false;
        }
    }
    return true;
}

ros::ShaderPtr ros::ShadersManager::provide(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Shader name is missing"));
        return ShaderPtr();
    }

    ShaderMap::iterator iter = shaders.find(name);
    if (iter != shaders.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Shader %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    ShaderPtr shader = Application::getInstance()->createShader();
    if (!shader || !shader->init(config)) {
        return ShaderPtr();
    }
    shaders[name] = shader;
    return shader;
}

void ros::ShadersManager::clear() {
    shaders.empty();
}

