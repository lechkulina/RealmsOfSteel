/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VECTOR2D_H
#define ROS_VECTOR2D_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>

namespace ros {
    class Vector3D;
    class Vector4D;
    class Matrix2D;

    class ROS_API Vector2D {
        public:
            float x, y;

            static const Vector2D ZERO;
            static const Vector2D RIGHT_AXIS;
            static const Vector2D UP_AXIS;

            Vector2D();
            Vector2D(float x, float y);
            Vector2D(const Vector2D& start, const Vector2D& end);
            Vector2D(const Vector3D& source);
            Vector2D(const Vector4D& source);

            Vector2D operator-() const;
            Vector2D operator+(const Vector2D& right) const;
            Vector2D& operator+=(const Vector2D& right);
            Vector2D operator-(const Vector2D& right) const;
            Vector2D& operator-=(const Vector2D& right);
            Vector2D operator*(float factor) const;
            Vector2D& operator*=(float factor);
            Vector2D operator*(const Vector2D& right) const;
            Vector2D& operator*=(const Vector2D& right);
            float dot(const Vector2D& right) const;
            float angel(const Vector2D& right) const;
            float angelBetweenUnitVectors(const Vector2D& right) const;
            float cross(const Vector2D& right) const;
            Vector2D normalized() const;
            Vector2D& normalize();
            float squaredLength() const;
            float length() const;
            Vector2D parallel(const Vector2D& adjacent) const;
            Vector2D parallelToUnitVector(const Vector2D& adjacent) const;
            Vector2D perpendicular(const Vector2D& adjacent) const;
            Vector2D perpendicularToUnitVector(const Vector2D& adjacent) const;
            Vector2D reflect(const Vector2D& normal) const;
            Vector2D reflectAgainstUnitVector(const Vector2D& normal) const;
    };
}

#endif // ROS_VECTOR2D_H
