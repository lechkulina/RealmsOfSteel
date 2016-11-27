/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_FILE_SINK_H
#define ROS_LOGS_FILE_SINK_H

#include <fstream>
#include <application/LogsSink.h>

namespace ros {
    class LogsFileSink : public LogsSink {
        public:
            virtual ~LogsFileSink();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual bool sendMessage(const LogMessage& message);
            virtual void flushMessages();

        private:
            std::ofstream stream;
    };
}

#endif // ROS_LOGS_FILE_SINK_H

