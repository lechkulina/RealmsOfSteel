/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_LEVEL_FILTER_H
#define ROS_LOGS_LEVEL_FILTER_H

#include <Application/LogsFilter.h>

namespace ros {

    class ROS_API LogsLevelFilter : public LogsFilter {
        public:
            virtual bool Init(const PropertyTree& config);
            virtual bool IsMessageAccepted(const LogMessage& message) const;

        private:
            LogLevelOpt threshold;
    };

}

#endif // ROS_LOGS_LEVEL_FILTER_H

