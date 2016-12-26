/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_EULER_ANGLES_H
#define ROS_EULER_ANGLES_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>

namespace ros {
    class Matrix3D;
    class Matrix4D;
    class Quaternion;

    class ROS_API EulerAngles {
        public:
            float heading;
            float pitch;
            float bank;

            static const EulerAngles ZERO;

            EulerAngles();
            EulerAngles(float heading, float pitch, float bank);
            EulerAngles(const Matrix3D& orientation);
            EulerAngles(const Matrix4D& orientation);
            EulerAngles(const Quaternion& orientation);
    };
}

#endif // ROS_EULER_ANGLES_H

