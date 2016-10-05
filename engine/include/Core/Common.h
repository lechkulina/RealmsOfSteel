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

#define ROS_NOOP ((void)0)
#define ROS_NULL NULL

#ifndef ROS_DISABLE_ASSERTS
    #define ROS_ASSERT assert
#else
    #define ROS_ASSERT ROS_NOOP
#endif

namespace ros {

    const size_t KB = 1024;
    const size_t MB = 1024 * 1024;

    template<class Type>
    inline Type Alignment(const Type& value, size_t multiple) {
        return (multiple - (value % multiple)) % multiple;
    }

    template<class Type>
    inline Type Align(const Type& value, size_t multiple) {
        return value + alignment(value, multiple);
    }

}

#endif // ROS_COMMON_H

