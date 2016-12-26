/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_MATRIX2D_H
#define ROS_MATRIX2D_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>
#include <math/Vector2D.h>

namespace ros {
    class Matrix3D;
    class Matrix4D;

    class ROS_API Matrix2D {
        public:
            float m11, m12;
            float m21, m22;

            static const Matrix2D ZERO;
            static const Matrix2D IDENTITY;

            Matrix2D();
            Matrix2D(float diagonal);
            Matrix2D(float m11, float m12,
                     float m21, float m22);
            Matrix2D(const Matrix3D& source);
            Matrix2D(const Matrix4D& source);

            static Matrix2D scaling(float factor);
            static Matrix2D rotation(float angle);

            Matrix2D operator+(const Matrix2D& right) const;
            Matrix2D& operator+=(const Matrix2D& right);
            Matrix2D operator-(const Matrix2D& right) const;
            Matrix2D& operator-=(const Matrix2D& right);
            Matrix2D operator*(float factor) const;
            Matrix2D& operator*=(float factor);
            Matrix2D operator*(const Matrix2D& right) const;
            Matrix2D& operator*=(const Matrix2D& right);
            Vector2D operator*(const Vector2D& right) const;
            Matrix2D transposed() const;
            Matrix2D& transpose();
            Matrix2D adjoint() const;
            Matrix2D inversed() const;
            Matrix2D& inverse();
            float determinant() const;
    };
}

#endif // ROS_MATRIX2D_H

