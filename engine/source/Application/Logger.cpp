/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <Application/Logger.h>

ros::LoggerPtr ros::Logger::logger;

ros::LoggerPtr ros::Logger::Create(const PropertyTree& config) {
    if (logger) {
        return logger;
    }

    logger.reset(new Logger());
    if (!logger->Init(config)) {
        logger.reset();
    }

    return logger;
}

bool ros::Logger::Init(const PropertyTree& config) {
    if (!LogsSink::Init(config)) {
        return false;
    }

    for (PropertyConstIter iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "Sink") {
            LogsSinkPtr sink = LogsSink::Create(iter->second);
            if (sink) {
                sinks.push_back(sink);
            }
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
