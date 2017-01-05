/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/LogsSink.h>
#include "LogsConsoleSink.h"
#include "LogsFileSink.h"

ros::LogsSinkFactory ros::LogsSink::factory;

ros::LogsSinkPtr ros::LogsSink::create(const PropertyTree& config) {
    if (factory.isEmpty()) {
        factory.registerClass<LogsConsoleSink>(boost::regex("Console"));
        factory.registerClass<LogsFileSink>(boost::regex("File"));
    }

    const std::string& type = config.data();
    LogsSinkPtr sink(factory.create(type.c_str()));
    if (!sink) {
        std::cerr << "Failed to create sink: Unknown type " << type << std::endl;
        return LogsSinkPtr();
    }
    if (!sink->init(config)) {
        return LogsSinkPtr();
    }

    return sink;
}

bool ros::LogsSink::init(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "Filter") {
            LogsFilterPtr filter = LogsFilter::create(iter->second);
            if (!filter) {
                continue;
            }
            filters.push_back(filter);
        }
    }
    return true;
}

void ros::LogsSink::uninit() {
    filters.clear();
}

bool ros::LogsSink::isMessageAccepted(const LogMessage& message) const {
    for (LogsFilterList::const_iterator iter = filters.begin(); iter != filters.end(); ++iter) {
        LogsFilterPtr filter = *iter;
        if (!filter->isMessageAccepted(message)) {
            return false;
        }
    }
    return true;
}
