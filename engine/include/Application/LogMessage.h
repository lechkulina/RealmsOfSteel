/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOG_MESSAGE_H
#define ROS_LOG_MESSAGE_H

#include <ostream>
#include <boost/optional.hpp>
#include <boost/format.hpp>
#include <boost/chrono.hpp>
#include <Core/Common.h>
#include <Core/Environment.h>

namespace ros {

    enum LogLevel {
        LogLevel_Trace,
        LogLevel_Debug,
        LogLevel_Warning,
        LogLevel_Error,
        LogLevel_Critical
    };

    typedef boost::optional<LogLevel> LogLevelOpt;

    const char* LogLevel_ToString(LogLevel level);
    LogLevelOpt LogLevel_FromString(const char* str);
    std::ostream& operator<<(std::ostream& stream, LogLevel level);

    class ROS_API LogMessage : public boost::format {
            typedef boost::chrono::system_clock Clock;

        public:
            typedef Clock::time_point TimePoint;

            LogMessage(LogLevel level, const char* format = ROS_NULL)
                : boost::format(format)
                , level(level)
                , timePoint(Clock::now()) {
            }

            LogMessage(LogLevel level, const std::string& format)
                : boost::format(format)
                , level(level)
                , timePoint(Clock::now()) {
            }

            LogLevel GetLevel() const { return level; }
            TimePoint GetTimePoint() const { return timePoint; }

            template<class Arg>
            inline LogMessage& operator%(const Arg& arg) {
                boost::format::operator%(arg);
                return *this;
            }

        private:
            LogLevel level;
            TimePoint timePoint;
    };

    std::ostream& operator<<(std::ostream& stream, const LogMessage& message);

}

#endif // ROS_LOG_MESSAGE_H

