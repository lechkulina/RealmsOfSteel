/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#include <graphics/ShaderManager.h>

ros::ShaderManager::~ShaderManager() {
    uninit();
}

bool ros::ShaderManager::init(const PropertyTree& config) {
    uninit();

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "shader" && !initShader(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::ShaderManager::uninit() {
    shaders.clear();
}

ros::ShaderPtr ros::ShaderManager::initShader(const PropertyTree& config) {
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

bool ros::ShaderManager::initShaders(const PropertyTree& config, ShaderList& dst) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "shader") {
            continue;
        }
        ShaderPtr shader = initShader(iter->second);
        if (!shader) {
            return false;
        }
        dst.push_back(shader);
    }
    return true;
}
