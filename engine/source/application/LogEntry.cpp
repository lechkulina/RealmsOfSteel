/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <application/LogEntry.h>

namespace {
    const struct LogLevelMapping {
        const char* str;
        ros::LogLevel level;
    } logLevelMappings[] = {
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

std::ostream& ros::operator<<(std::ostream& stream, const LogEntry& entry) {
    return stream << "[" << entry.getLevel() << "] " << entry.getMessage() << std::endl;
}

ros::LogEntry::LogEntry(LogLevel level, const char* fileName, U32 lineNumber, const std::string& message)
    : level(level)
    , fileName(fileName)
    , lineNumber(lineNumber)
    , message(message)
    , time(chr::system_clock::now()) {
}
