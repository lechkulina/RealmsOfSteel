/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_LEVEL_FILTER_H
#define ROS_LOGS_LEVEL_FILTER_H

#include <application/LogsFilter.h>

namespace ros {
    class ROS_API LogsLevelFilter : public LogsFilter {
        public:
            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool isMessageAccepted(const LogMessage& message) const;

        private:
            LogLevelOpt thresholdLevel;
    };
}

#endif // ROS_LOGS_LEVEL_FILTER_H

