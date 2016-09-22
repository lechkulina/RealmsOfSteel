/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_INTEGER_H
#define ROS_INTEGER_H

#include <boost/integer.hpp>

namespace ros {

    typedef boost::int_t<8>::exact S8;
    typedef boost::int_t<16>::exact S16;
    typedef boost::int_t<32>::exact S32;
    typedef boost::int_t<64>::exact S64;

    typedef boost::uint_t<8>::exact U8;
    typedef boost::uint_t<16>::exact U16;
    typedef boost::uint_t<32>::exact U32;
    typedef boost::uint_t<64>::exact U64;

}

#endif // ROS_INTEGER_H


