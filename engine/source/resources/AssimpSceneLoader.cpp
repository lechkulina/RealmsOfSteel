/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <application/Logger.h>
#include "AssimpIOSystem.h"
#include "AssimpSceneLoader.h"

ros::AssimpSceneLoader::AssimpSceneLoader()
    : importer(ROS_NULL) {
    importer = new Assimp::Importer;
    importer->SetIOHandler(new AssimpIOSystem());
}

ros::AssimpSceneLoader::~AssimpSceneLoader() {
    delete importer;
}

bool ros::AssimpSceneLoader::isLoadable(const std::string& name) const {
    if (name.length() <= MAX_EXTENSION_LENGTH) {
        return false;
    }
    const std::string extension = name.substr(name.length() - MAX_EXTENSION_LENGTH);
    return importer->IsExtensionSupported(extension);
}

ros::ResourcePtr ros::AssimpSceneLoader::loadResource(const std::string& name) {
    const aiScene* const srcScene = importer->ReadFile(name.c_str(), aiProcess_Triangulate|aiProcess_SortByPType);
    if (!srcScene) {
        Logger::report(LogLevel_Error, boost::format("Failed to load scene for resource %s - Assimp error occured %s")
                            % name % importer->GetErrorString());
        return ScenePtr();
    }

    // TODO

    importer->FreeScene();
    return ScenePtr();
}
