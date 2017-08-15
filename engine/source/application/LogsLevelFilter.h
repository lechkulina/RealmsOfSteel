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
            void setThreshold(LogLevelOpt threshold);
            LogLevelOpt getThreshold() const { return threshold; }

            virtual bool init(const pt::ptree& config);
            virtual bool isEntryAccepted(const LogEntry& entry) const;

        private:
            LogLevelOpt threshold;
    };
}

#endif // ROS_LOGS_LEVEL_FILTER_H

