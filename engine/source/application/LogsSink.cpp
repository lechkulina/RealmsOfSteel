/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/LogsSink.h>
#include "LogsConsoleSink.h"
#include "LogsFileSink.h"

ros::Factory<ros::LogsSink> ros::LogsSink::factory;

ros::LogsSinkPtr ros::LogsSink::create(const std::string& classId) {
    if (factory.isEmpty()) {
        factory.registerClass<LogsConsoleSink>(boost::regex("console"));
        factory.registerClass<LogsFileSink>(boost::regex("file"));
    }
    LogsSinkPtr sink(factory.create(classId));
    return sink;
}

bool ros::LogsSink::init(const pt::ptree& config) {
    for (pt::ptree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "filter") {
            const pt::ptree& filterConfig = iter->second;
            const std::string& filterClassId = filterConfig.data();
            LogsFilterPtr filter = LogsFilter::create(filterClassId);
            if (!filter) {
                std::cerr << "Unknown logs filter class ID " << filterClassId << std::endl;
                continue;
            }
            if (!filter->init(filterConfig)) {
                continue;
            }
            addFilter(filter);
        }
    }
    return true;
}

bool ros::LogsSink::isEntryAccepted(const LogEntry& entry) const {
    for (LogsFiltersList::const_iterator iter = filters.begin(); iter != filters.end(); ++iter) {
        LogsFilterPtr filter = *iter;
        if (!filter->isEntryAccepted(entry)) {
            return false;
        }
    }
    return true;
}

void ros::LogsSink::addFilter(LogsFilterPtr filter) {
    filters.push_back(filter);
}
