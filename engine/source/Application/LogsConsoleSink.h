/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_CONSOLE_SINK_H
#define ROS_LOGS_CONSOLE_SINK_H

#include <ostream>
#include <Application/LogsSink.h>

namespace ros {

    class ROS_API LogsConsoleSink : public LogsSink {
        public:
            LogsConsoleSink();

            virtual bool Init(const PropertyTree& config);

            virtual bool SendMessage(const LogMessage& message);
            virtual void FlushMessages();

        private:
            std::ostream* stream;
    };

}

#endif // ROS_LOGS_CONSOLE_SINK_H

