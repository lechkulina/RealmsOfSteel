/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_PROGRAM_H
#define ROS_PROGRAM_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Vector4D.h>
#include <math/Matrix4D.h>

namespace ros {
    class ROS_API Program: public boost::noncopyable {
        public:
            virtual ~Program() {}

            virtual bool init(const PropertyTree& config) =0;
            virtual void uninit() =0;
            virtual bool isValid() const =0;

            virtual bool bind() =0;
            virtual void unbind() =0;

            virtual bool setUniform(const char* name, int value) =0;
            virtual bool setUniform(const char* name, const Vector4D& value) =0;
            virtual bool setUniform(const char* name, const Matrix4D& value) =0;
    };

    typedef boost::shared_ptr<Program> ProgramPtr;
}

#endif // ROS_PROGRAM_H

