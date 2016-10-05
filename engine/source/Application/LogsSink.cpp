/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Core/Factory.h>
#include "LogsLevelFilter.h"
#include <Application/LogsSink.h>

ros::LogsSink::LogsSink() {
    filtersFactory.RegisterClass<LogsLevelFilter>("Level");
}

bool ros::LogsSink::Init(const PropertyTree& config) {
    for (PropertyConstIter iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "Filter") {
            const std::string& id = iter->second.data();
            LogsFilterPtr filter = filtersFactory.MakeShared(id);
            if (!filter || !filter->Init(iter->second)) {
                std::cerr << "Failed to create filter " << id << std::endl;
                continue;
            }
            filters.push_back(filter);
        }
    }
    return true;
}

bool ros::LogsSink::IsMessageAccepted(const LogMessage& message) const {
    for (LogsFiltersConstIter iter = filters.begin(); iter != filters.end(); ++iter) {
        if (!(*iter)->IsMessageAccepted(message)) {
            return false;
        }
    }
    return true;
}

void ros::LogsSink::AddFilter(LogsFilterPtr filter) {
    filters.push_back(filter);
}

void ros::LogsSink::RemoveFilter(LogsFilterPtr filter) {
    filters.remove(filter);
}
