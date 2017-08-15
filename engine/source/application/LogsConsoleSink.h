/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_CONSOLE_SINK_H
#define ROS_LOGS_CONSOLE_SINK_H

#include <application/LogsSink.h>

namespace ros {
    class ROS_API LogsConsoleSink : public LogsSink {
        public:
            static const std::string DEFAULT_STREAM;

            LogsConsoleSink();

            void setStream(std::ostream* stream);
            bool hasStream() const { return stream != ROS_NULL; }

            virtual bool init(const pt::ptree& config);

            virtual bool sendEntry(const LogEntry& entry);
            virtual void flushEntries();

        private:
            std::ostream* stream;
    };
}

#endif // ROS_LOGS_CONSOLE_SINK_H

