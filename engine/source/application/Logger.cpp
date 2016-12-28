/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>

ros::LoggerPtr ros::Logger::logger;

ros::LoggerPtr ros::Logger::initInstance(const PropertyTree& config) {
    if (logger) {
        return logger;
    }

    logger.reset(new Logger());
    if (!logger->init(config)) {
        logger.reset();
    }

    return logger;
}

bool ros::Logger::init(const PropertyTree& config) {
    if (!LogsSink::init(config)) {
        return false;
    }

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "Sink") {
            LogsSinkPtr sink = LogsSink::create(iter->second);
            if (!sink) {
                continue;
            }
            sinks.push_back(sink);
        }
    }

    if (sinks.empty()) {
        std::cerr << "Failed to initialize logger: There are no sinks" << std::endl;
        uninit();
        return false;
    }

    return true;
}

void ros::Logger::uninit() {
    sinks.clear();
    LogsSink::uninit();
}

bool ros::Logger::sendMessage(const LogMessage& message) {
    if (!isMessageAccepted(message)) {
        return false;
    }

    for (LogsSinkList::iterator iter = sinks.begin(); iter != sinks.end(); ++iter) {
        LogsSinkPtr sink = *iter;
        if (sink->isMessageAccepted(message) && !sink->sendMessage(message)) {
            return false;
        }
    }

    return true;
}

void ros::Logger::flushMessages() {
    for (LogsSinkList::iterator iter = sinks.begin(); iter != sinks.end(); ++iter) {
        LogsSinkPtr sink = *iter;
        sink->flushMessages();
    }
}
