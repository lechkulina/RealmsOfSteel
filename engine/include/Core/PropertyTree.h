/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_PROPERTY_TREE_H
#define ROS_PROPERTY_TREE_H

#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>

namespace ros {

    typedef boost::property_tree::ptree PropertyTree;

    typedef PropertyTree::iterator PropertyIter;
    typedef PropertyTree::const_iterator PropertyConstIter;
    typedef PropertyTree::assoc_iterator PropertyAssocIter;
    typedef PropertyTree::const_assoc_iterator PropertyConstAssocIter;

    typedef std::string String;

    typedef boost::optional<String> StringOpt;
    typedef boost::optional<int> IntOpt;
    typedef boost::optional<float> FloatOpt;

}

#endif // ROS_PROPERTY_TREE_H

