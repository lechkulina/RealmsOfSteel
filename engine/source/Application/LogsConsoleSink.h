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
            LogsConsoleSink();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual bool sendMessage(const LogMessage& message);
            virtual void flushMessages();

        private:
            std::ostream* stream;
    };
}

#endif // ROS_LOGS_CONSOLE_SINK_H

