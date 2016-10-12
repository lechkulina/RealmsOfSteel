/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/LogsFilter.h>
#include "LogsLevelFilter.h"

ros::LogsFilterFactory ros::LogsFilter::factory;

ros::LogsFilterPtr ros::LogsFilter::Create(const PropertyTree& config) {
    if (factory.IsEmpty()) {
        factory.RegisterClass<LogsLevelFilter>("Level");
    }

    const String& type = config.data();
    LogsFilterPtr filter(factory.CreateInstance(type));
    if (!filter) {
        std::cerr << "Failed to create filter: Unknown type " << type << std::endl;
        return filter;
    }

    if (!filter->Init(config)) {
        filter.reset();
    }

    return filter;
}
