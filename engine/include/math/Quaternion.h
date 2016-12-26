/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_QUATERNION_H
#define ROS_QUATERNION_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>

namespace ros {
    class Vector3D;
    class Matrix3D;
    class Matrix4D;
    class EulerAngles;

    class ROS_API Quaternion {
        public:
            float w, x, y, z;

            static const Quaternion ZERO;
            static const Quaternion IDENTITY;

            Quaternion();
            Quaternion(float w, float x, float y, float z);
            Quaternion(const Quaternion& start, const Quaternion& end);
            Quaternion(const Vector3D& axis, float angle);
            Quaternion(const Matrix3D& orientation);
            Quaternion(const Matrix4D& orientation);
            Quaternion(const EulerAngles& orientation);

            static Quaternion lerp(const Quaternion& start, const Quaternion& end, float factor);
            static Quaternion slerp(const Quaternion& start, const Quaternion& end, float factor);

            Quaternion operator-() const;
            Quaternion operator*(float factor) const;
            Quaternion& operator*=(float factor);
            Quaternion operator*(const Quaternion& right) const;
            Quaternion& operator*=(const Quaternion& right);
            float dot(const Quaternion& right) const;
            float magnitude() const;
            Quaternion conjugated() const;
            Quaternion& conjugate();
            Quaternion inversed() const;
            Quaternion& inverse();
            Quaternion exponentiate(float exponent) const;
            Quaternion& exponentiated(float exponent);
    };
}

#endif // ROS_QUATERNION_H

