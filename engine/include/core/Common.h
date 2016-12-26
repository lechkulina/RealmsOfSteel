/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_COMMON_H
#define ROS_COMMON_H

#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cassert>
#include <string>
#include <algorithm>
#include <utility>
#include <ostream>
#include <iostream>
#include <new>
#include <map>
#include <list>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/integer.hpp>
#include <boost/chrono.hpp>
#include <boost/property_tree/ptree.hpp>

#define ROS_NOOP ((void)0)
#define ROS_NULL NULL

#ifndef ROS_DISABLE_ASSERTS
    #define ROS_ASSERT assert
#else
    #define ROS_ASSERT ROS_NOOP
#endif

namespace ros {
    typedef boost::int_t<8>::exact S8;
    typedef boost::int_t<16>::exact S16;
    typedef boost::int_t<32>::exact S32;
    typedef boost::int_t<64>::exact S64;
    typedef boost::uint_t<8>::exact U8;
    typedef boost::uint_t<16>::exact U16;
    typedef boost::uint_t<32>::exact U32;
    typedef boost::uint_t<64>::exact U64;

    typedef boost::property_tree::ptree PropertyTree;
    typedef boost::chrono::system_clock SystemClock;

    typedef boost::optional<std::string> StringOpt;

    const size_t KB = 1024;
    const size_t MB = 1024 * 1024;

    template<class Type>
    inline Type alignment(const Type& value, size_t multiple) {
        return (multiple - (value % multiple)) % multiple;
    }

    template<class Type>
    inline Type align(const Type& value, size_t multiple) {
        return value + alignment(value, multiple);
    }
}

#endif // ROS_COMMON_H
