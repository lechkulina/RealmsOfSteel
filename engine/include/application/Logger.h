/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGGER_H
#define ROS_LOGGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <application/LogsSink.h>

namespace ros {
    class Logger;
    typedef boost::shared_ptr<Logger> LoggerPtr;

    class ROS_API Logger : public boost::noncopyable {
        public:
            static LoggerPtr initInstance(const pt::ptree& config);
            static LoggerPtr getInstance() { return instance; }

            void addSink(LogsSinkPtr sink);

            bool sendEntry(const LogEntry& entry);
            bool sendEntry(LogLevel level, const char* fileName, U32 lineNumber, const boost::format& message);
            void flushEntries();

        private:
            static LoggerPtr instance;
            LogsSinksList sinks;
    };
}

#ifndef ROS_DISABLE_LOGS
    #define ROS_LOG(level, message) ros::Logger::getInstance()->sendEntry((level), __FILE__, __LINE__, (message))
#else
    #define ROS_LOG(level, message) ROS_NOOP
#endif

#define ROS_TRACE(message) ROS_LOG(ros::LogLevel_Trace, message)
#define ROS_DEBUG(message) ROS_LOG(ros::LogLevel_Debug, message)
#define ROS_WARNING(message) ROS_LOG(ros::LogLevel_Warning, message)
#define ROS_ERROR(message) ROS_LOG(ros::LogLevel_Error, message)
#define ROS_CRITICAL(message) ROS_LOG(ros::LogLevel_Critical, message)

#endif // ROS_LOGGER_H

