/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/Logger.h>
#include "LogsConsoleSink.h"
#include "LogsFileSink.h"

ros::LoggerPtr ros::Logger::instance;

ros::LoggerPtr ros::Logger::CreateInstance(const PropertyTree& config) {
    if (instance) {
        return instance;
    }

    try {
        const PropertyTree& subConfig = config.get_child("Logger");
        instance.reset(new Logger());
        if (instance && !instance->Init(subConfig)) {
            instance.reset();
        }
    } catch (const BadPathException& exception) {
        std::cerr << "Failed to read logger configuration: " << exception.what() << std::endl;
        instance.reset();
    }

    return instance;
}

ros::Logger::Logger() {
    sinksFactory.RegisterClass<LogsConsoleSink>("Console");
    sinksFactory.RegisterClass<LogsFileSink>("File");
}

bool ros::Logger::Init(const PropertyTree& config) {
    if (!LogsSink::Init(config)) {
        return false;
    }

    for (PropertyConstIter iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "Sink") {
            const std::string& id = iter->second.data();
            LogsSinkPtr sink = sinksFactory.MakeShared(id);
            if (!sink || !sink->Init(iter->second)) {
                std::cerr << "Failed to create sink " << id << std::endl;
                continue;
            }
            sinks.push_back(sink);
        }
    }

    return true;
}

bool ros::Logger::SendMessage(const LogMessage& message) {
    if (!IsMessageAccepted(message)) {
        return false;
    }
    for (LogsSinksIter iter = sinks.begin(); iter != sinks.end(); ++iter) {
        LogsSinkPtr sink = *iter;
        if (sink->IsMessageAccepted(message) && !sink->SendMessage(message)) {
            return false;
        }
    }
    return true;
}

void ros::Logger::FlushMessages() {
    for (LogsSinksIter iter = sinks.begin(); iter != sinks.end(); ++iter) {
        (*iter)->FlushMessages();
    }
}
