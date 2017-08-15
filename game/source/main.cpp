/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <cstdlib>
#include <boost/property_tree/info_parser.hpp>
#include <application/Logger.h>
#include <application/Application.h>
#include <resources/FileSystem.h>
#include <resources/ResourcesCache.h>
#include <graphics/Scene.h>

void dumpScene(ros::SceneNodePtr node) {
    if (!node) {
        return;
    }
    std::cout << "node name=" << node->getName() << " meshes=" << node->getMeshes().size() << " children=" << node->getChildren().size() << std::endl;
    for (ros::MeshesVector::const_iterator iter = node->getMeshes().begin(); iter != node->getMeshes().end(); ++iter) {
        ros::MeshPtr mesh = *iter;
        std::cout << "    mesh name=" << mesh->getName() << " vertices=" << mesh->getVertices().size() << " indices=" << mesh->getIndices().size() << std::endl;
    }
    for (ros::SceneNodesList::const_iterator iter = node->getChildren().begin(); iter != node->getChildren().end(); ++iter) {
        dumpScene(*iter);
    }
}

int main() {
    int exitCode = EXIT_FAILURE;

    ros::PropertyTree config;
    boost::property_tree::read_info("Config.info", config);

    if (!ros::Logger::initInstance(config.get_child("logger")) ||
        !ros::Application::initInstance(config.get_child("application")) ||
        !ros::FileSystem::initInstance(config.get_child("file-system")) ||
        !ros::ResourcesCache::initInstance(config.get_child("resources-cache"))) {
        return exitCode;
    }

    ros::ScenePtr scene = ros::ResourcesCache::getInstance()->acquireResource<ros::Scene>("suzanne.obj");
    if (scene) {
        dumpScene(scene->getRoot());
    }

    ros::ApplicationPtr application = ros::Application::getInstance();
    if (application) {
        exitCode = application->run();
        application->uninit();
    }

    return exitCode;
}
