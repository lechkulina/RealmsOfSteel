/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>

ros::LoggerPtr ros::Logger::instance;

ros::LoggerPtr ros::Logger::initInstance(const pt::ptree& config) {
    if (instance) {
        return instance;
    }

    instance.reset(new (std::nothrow) Logger());
    if (!instance) {
        std::cerr << "Failed to create logger instance" << std::endl;
        return LoggerPtr();
    }

    for (pt::ptree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "sink") {
            const pt::ptree& sinkConfig = iter->second;
            const std::string& sinkClassId = sinkConfig.data();
            LogsSinkPtr sink = LogsSink::create(sinkClassId);
            if (!sink) {
                std::cerr << "Unknown logs sink class ID " << sinkClassId << std::endl;
                continue;
            }
            if (!sink->init(sinkConfig)) {
                continue;
            }
            instance->addSink(sink);
        }
    }

    return instance;
}

void ros::Logger::addSink(LogsSinkPtr sink) {
    sinks.push_back(sink);
}

bool ros::Logger::sendEntry(const LogEntry& entry) {
    for (LogsSinksList::iterator iter = sinks.begin(); iter != sinks.end(); ++iter) {
        LogsSinkPtr sink = *iter;
        if (sink->isEntryAccepted(entry) && !sink->sendEntry(entry)) {
            return false;
        }
    }
    return true;
}

bool ros::Logger::sendEntry(LogLevel level, const char* fileName, U32 lineNumber, const boost::format& message) {
    try {
        LogEntry entry(level, fileName, lineNumber, message.str());
        return sendEntry(entry);
    } catch(boost::io::format_error& error) {
        std::cerr << "Failed to parse the log message - format error occured: " << error.what() << std::endl;
        return false;
    }
}

void ros::Logger::flushEntries() {
    for (LogsSinksList::iterator iter = sinks.begin(); iter != sinks.end(); ++iter) {
        LogsSinkPtr sink = *iter;
        sink->flushEntries();
    }
}
