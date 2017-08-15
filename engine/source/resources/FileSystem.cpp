/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <core/Factory.h>
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "ArchiveFileSystem.h"

const ros::fs::path ros::FileSystem::DEFAULT_ROOT("data");

ros::Factory<ros::FileSystem> ros::FileSystem::factory;
ros::FileSystemPtr ros::FileSystem::instance;

ros::FileSystemPtr ros::FileSystem::initInstance(const pt::ptree& config) {
    if (instance) {
        return instance;
    }

    if (factory.isEmpty()) {
        factory.registerClass<ArchiveFileSystem>(boost::regex("archive"));
    }

    const std::string classId = config.data();
    instance.reset(factory.create(classId));
    if (!instance) {
        ROS_ERROR(boost::format("Unknown file system class ID %s") % classId);
        return FileSystemPtr();
    }
    if (!instance->setRoot(config.get("root", DEFAULT_ROOT))) {
        ROS_ERROR(boost::format("Failed to initialize file system with class ID %s") % classId);
        return FileSystemPtr();
    }

    ROS_TRACE(boost::format("File system with class ID %s initialized") % classId);
    return instance;
}
