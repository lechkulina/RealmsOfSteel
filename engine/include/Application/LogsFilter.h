/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_FILTER_H
#define ROS_LOGS_FILTER_H

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <Core/Environment.h>
#include <Application/LogMessage.h>

namespace ros {

    typedef boost::property_tree::ptree PropertyTree;
    typedef PropertyTree::iterator PropertyIter;
    typedef PropertyTree::const_iterator PropertyConstIter;

    typedef boost::optional<std::string> StringOpt;
    typedef boost::optional<int> IntOpt;
    typedef boost::optional<float> FloatOpt;

    class ROS_API LogsFilter : public boost::noncopyable {
        public:
            virtual ~LogsFilter() {}

            virtual bool Init(const PropertyTree& config) =0;
            virtual bool IsMessageAccepted(const LogMessage& message) const =0;
    };

    typedef boost::shared_ptr<LogsFilter> LogsFilterPtr;

}

#endif // ROS_LOGS_FILTER_H

