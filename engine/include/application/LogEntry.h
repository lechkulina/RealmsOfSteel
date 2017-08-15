/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOG_ENTRY_H
#define ROS_LOG_ENTRY_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum LogLevel {
        LogLevel_Trace,
        LogLevel_Debug,
        LogLevel_Warning,
        LogLevel_Error,
        LogLevel_Critical
    };

    typedef boost::optional<LogLevel> LogLevelOpt;

    LogLevelOpt LogLevel_fromString(const char* str);
    const char* LogLevel_toString(LogLevel level);
    std::ostream& operator<<(std::ostream& stream, LogLevel level);

    class ROS_API LogEntry {
        public:
            LogEntry(LogLevel level, const char* fileName, U32 lineNumber, const std::string& message);

            LogLevel getLevel() const { return level; }
            const char* getFileName() const { return fileName; }
            U32 getLineNumber() const { return lineNumber; }
            const std::string& getMessage() const { return message; }
            chr::system_clock::time_point getTime() const { return time; }

        private:
            LogLevel level;
            const char* fileName;
            U32 lineNumber;
            std::string message;
            chr::system_clock::time_point time;
    };

    std::ostream& operator<<(std::ostream& stream, const LogEntry& entry);
}

#endif // ROS_LOG_ENTRY_H

