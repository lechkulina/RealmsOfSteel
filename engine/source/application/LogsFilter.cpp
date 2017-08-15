/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/LogsFilter.h>
#include "LogsLevelFilter.h"

ros::Factory<ros::LogsFilter> ros::LogsFilter::factory;

boost::shared_ptr<ros::LogsFilter> ros::LogsFilter::create(const std::string& classId) {
    if (factory.isEmpty()) {
        factory.registerClass<LogsLevelFilter>(boost::regex("level"));
    }
    LogsFilterPtr filter(factory.create(classId));
    return filter;
}
