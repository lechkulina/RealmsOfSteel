/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/LogsFilter.h>
#include "LogsLevelFilter.h"

ros::LogsFilterFactory ros::LogsFilter::factory;

boost::shared_ptr<ros::LogsFilter> ros::LogsFilter::create(const PropertyTree& config) {
    if (factory.isEmpty()) {
        factory.registerClass<LogsLevelFilter>("Level");
    }

    const std::string& type = config.data();
    LogsFilterPtr filter(factory.create(type));
    if (!filter) {
        std::cerr << "Failed to create filter: Unknown type " << type << std::endl;
        return LogsFilterPtr();
    }
    if (!filter->init(config)) {
        return LogsFilterPtr();
    }

    return filter;
}
