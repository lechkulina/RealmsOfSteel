/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <application/Application.h>
#include <graphics/ProgramsManager.h>

ros::ProgramsManager* ros::ProgramsManager::getInstance() {
    static ProgramsManager instance;
    return &instance;
}

bool ros::ProgramsManager::prepare(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "program" && !provide(iter->second)) {
            clear();
            return false;
        }
    }
    return true;
}

ros::ProgramPtr ros::ProgramsManager::provide(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Program name is missing"));
        return ProgramPtr();
    }

    ProgramMap::iterator iter = programs.find(name);
    if (iter != programs.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Program %s has already been defined - ignoring redefinition") % name);
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

void ros::ProgramsManager::clear() {
    programs.empty();
}

