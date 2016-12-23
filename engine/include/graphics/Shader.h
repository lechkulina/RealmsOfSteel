/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SHADER_H
#define ROS_SHADER_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    class ROS_API Shader: public boost::noncopyable {
        public:
            virtual ~Shader() {}

            virtual bool init(const PropertyTree& config) =0;
            virtual void uninit() =0;

            virtual void* getHandle() =0;
    };

    typedef boost::shared_ptr<Shader> ShaderPtr;
}

#endif // ROS_SHADER_H

