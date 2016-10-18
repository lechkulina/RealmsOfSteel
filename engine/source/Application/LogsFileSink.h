/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_FILE_SINK_H
#define ROS_LOGS_FILE_SINK_H

#include <fstream>
#include <Application/LogsSink.h>

namespace ros {

    class LogsFileSink : public LogsSink {
        public:
            virtual ~LogsFileSink();

            virtual bool Init(const PropertyTree& config);
            virtual void Uninit();

            virtual bool SendMessage(const LogMessage& message);
            virtual void FlushMessages();

        private:
            std::ofstream stream;
    };

}

#endif // ROS_LOGS_FILE_SINK_H

