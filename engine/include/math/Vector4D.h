/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VECTOR4D_H
#define ROS_VECTOR4D_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>

namespace ros {
    class Vector2D;
    class Vector3D;
    class Matrix4D;

    class ROS_API Vector4D {
        public:
            float x, y, z, w;

            static const Vector4D ZERO;
            static const Vector4D RIGHT_AXIS;
            static const Vector4D UP_AXIS;
            static const Vector4D FORWARD_AXIS;

            Vector4D();
            Vector4D(float x, float y, float z, float w);
            Vector4D(const Vector4D& start, const Vector4D& end);
            Vector4D(const Vector2D& source);
            Vector4D(const Vector3D& source);

            Vector4D operator-() const;
            Vector4D operator+(const Vector4D& right) const;
            Vector4D& operator+=(const Vector4D& right);
            Vector4D operator-(const Vector4D& right) const;
            Vector4D& operator-=(const Vector4D& right);
            Vector4D operator*(float factor) const;
            Vector4D& operator*=(float factor);
            Vector4D operator*(const Vector4D& right) const;
            Vector4D& operator*=(const Vector4D& right);
            float dot(const Vector4D& right) const;
            float angel(const Vector4D& right) const;
            float angelBetweenUnitVectors(const Vector4D& right) const;
            Vector4D cross(const Vector4D& right) const;
            Vector4D normalized() const;
            Vector4D& normalize();
            float squaredLength() const;
            float length() const;
            Vector4D parallel(const Vector4D& adjacent) const;
            Vector4D parallelToUnitVector(const Vector4D& adjacent) const;
            Vector4D perpendicular(const Vector4D& adjacent) const;
            Vector4D perpendicularToUnitVector(const Vector4D& adjacent) const;
            Vector4D reflect(const Vector4D& normal) const;
            Vector4D reflectAgainstUnitVector(const Vector4D& normal) const;
    };
}

#endif // ROS_VECTOR4D_H
