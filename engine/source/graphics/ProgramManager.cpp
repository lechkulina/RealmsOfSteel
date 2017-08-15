/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#include <graphics/ProgramManager.h>

ros::ProgramManager::~ProgramManager() {
    uninit();
}

bool ros::ProgramManager::init(const PropertyTree& config) {
    uninit();

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "program" && !initProgram(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::ProgramManager::uninit() {
    programs.clear();
}

ros::ProgramPtr ros::ProgramManager::initProgram(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        ROS_ERROR(boost::format("Program name is missing"));
        return ProgramPtr();
    }

    ProgramMap::iterator iter = programs.find(name);
    if (iter != programs.end()) {
        if (!config.empty()) {
            ROS_WARNING(boost::format("Program %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    ProgramPtr program = Application::getInstance()->createProgram();
    if (!program || !program->init(config)) {
        return ProgramPtr();
    }

    programs[name] = program;
    return program;
}

bool ros::ProgramManager::initPrograms(const PropertyTree& config, ProgramList& dst) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "program") {
            continue;
        }
        ProgramPtr program = initProgram(iter->second);
        if (!program) {
            return false;
        }
        dst.push_back(program);
    }
    return true;
}
