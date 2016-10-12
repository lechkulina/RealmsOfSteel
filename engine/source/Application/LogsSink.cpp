/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/LogsSink.h>
#include "LogsConsoleSink.h"
#include "LogsFileSink.h"

ros::LogsSinkFactory ros::LogsSink::factory;

ros::LogsSinkPtr ros::LogsSink::Create(const PropertyTree& config) {
    if (factory.IsEmpty()) {
        factory.RegisterClass<LogsConsoleSink>("Console");
        factory.RegisterClass<LogsFileSink>("File");
    }

    const String& type = config.data();
    LogsSinkPtr sink(factory.CreateInstance(type));
    if (!sink) {
        std::cerr << "Failed to create sink: Unknown type " << type << std::endl;
        return sink;
    }

    if (!sink->Init(config)) {
        sink.reset();
    }

    return sink;
}

bool ros::LogsSink::Init(const PropertyTree& config) {
    for (PropertyConstIter iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "Filter") {
            LogsFilterPtr filter = LogsFilter::Create(iter->second);
            if (filter) {
                filters.push_back(filter);
            }
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
