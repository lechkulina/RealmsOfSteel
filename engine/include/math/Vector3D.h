/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VECTOR3D_H
#define ROS_VECTOR3D_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>

namespace ros {
    class Vector2D;
    class Vector4D;
    class Matrix3D;

    class ROS_API Vector3D {
        public:
            float x, y, z;

            static const Vector3D ZERO;
            static const Vector3D RIGHT_AXIS;
            static const Vector3D UP_AXIS;
            static const Vector3D FORWARD_AXIS;

            Vector3D();
            Vector3D(float x, float y, float z);
            Vector3D(const Vector3D& start, const Vector3D& end);
            Vector3D(const Vector2D& source);
            Vector3D(const Vector4D& source);

            Vector3D operator-() const;
            Vector3D operator+(const Vector3D& right) const;
            Vector3D& operator+=(const Vector3D& right);
            Vector3D operator-(const Vector3D& right) const;
            Vector3D& operator-=(const Vector3D& right);
            Vector3D operator*(float factor) const;
            Vector3D& operator*=(float factor);
            Vector3D operator*(const Vector3D& right) const;
            Vector3D& operator*=(const Vector3D& right);
            float dot(const Vector3D& right) const;
            float angel(const Vector3D& right) const;
            float angelBetweenUnitVectors(const Vector3D& right) const;
            Vector3D cross(const Vector3D& right) const;
            Vector3D normalized() const;
            Vector3D& normalize();
            float squaredLength() const;
            float length() const;
            Vector3D parallel(const Vector3D& adjacent) const;
            Vector3D parallelToUnitVector(const Vector3D& adjacent) const;
            Vector3D perpendicular(const Vector3D& adjacent) const;
            Vector3D perpendicularToUnitVector(const Vector3D& adjacent) const;
            Vector3D reflect(const Vector3D& normal) const;
            Vector3D reflectAgainstUnitVector(const Vector3D& normal) const;
    };
}

#endif // ROS_VECTOR3D_H
