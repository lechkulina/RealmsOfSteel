/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <application/LogMessage.h>

namespace {
    struct LogLevelMapping {
        const char* str;
        ros::LogLevel level;
    };

    const LogLevelMapping logLevelMappings[] = {
        {"Trace", ros::LogLevel_Trace},
        {"Debug", ros::LogLevel_Debug},
        {"Warning", ros::LogLevel_Warning},
        {"Error", ros::LogLevel_Error},
        {"Critical", ros::LogLevel_Critical}
    };
}

ros::LogLevelOpt ros::LogLevel_fromString(const char* str) {
    const LogLevelMapping* iter = std::find_if(boost::begin(logLevelMappings), boost::end(logLevelMappings),
        boost::bind(strcmp, boost::bind(&LogLevelMapping::str, _1), str) == 0);
    if (iter != boost::end(logLevelMappings)) {
        return iter->level;
    }
    return LogLevelOpt();
}


const char* ros::LogLevel_toString(LogLevel level) {
    const LogLevelMapping* iter = std::find_if(boost::begin(logLevelMappings), boost::end(logLevelMappings),
        boost::bind(&LogLevelMapping::level, _1) == level);
    if (iter != boost::end(logLevelMappings)) {
        return iter->str;
    }
    return ROS_NULL;
}

std::ostream& ros::operator<<(std::ostream& stream, LogLevel level) {
    return stream << LogLevel_toString(level);
}

std::ostream& ros::operator<<(std::ostream& stream, const LogMessage& message) {
    return stream << "[" << message.getLevel() << "] " << message.getMessage() << std::endl;
}

ros::LogMessage::LogMessage(LogLevel level, const std::string& message)
    : level(level)
    , message(message)
    , timePoint(SystemClock::now()) {
}
