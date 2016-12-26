/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_MATRIX3D_H
#define ROS_MATRIX3D_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>
#include <math/Vector3D.h>

namespace ros {
    class Matrix2D;
    class Matrix4D;
    class EulerAngles;
    class Quaternion;

    class ROS_API Matrix3D {
        public:
            float m11, m12, m13;
            float m21, m22, m23;
            float m31, m32, m33;

            static const Matrix3D ZERO;
            static const Matrix3D IDENTITY;

            Matrix3D();
            Matrix3D(float diagonal);
            Matrix3D(float m11, float m12, float m13,
                     float m21, float m22, float m23,
                     float m31, float m32, float m33);
            Matrix3D(const Matrix2D& source);
            Matrix3D(const Matrix4D& source);
            Matrix3D(const EulerAngles& orientation);
            Matrix3D(const Quaternion& orientation);

            static Matrix3D scaling(float factor);
            static Matrix3D rotationX(float angle);
            static Matrix3D rotationY(float angle);
            static Matrix3D rotationZ(float angle);

            Matrix3D operator+(const Matrix3D& right) const;
            Matrix3D& operator+=(const Matrix3D& right);
            Matrix3D operator-(const Matrix3D& right) const;
            Matrix3D& operator-=(const Matrix3D& right);
            Matrix3D operator*(float factor) const;
            Matrix3D& operator*=(float factor);
            Matrix3D operator*(const Matrix3D& right) const;
            Matrix3D& operator*=(const Matrix3D& right);
            Vector3D operator*(const Vector3D& right) const;
            Matrix3D transposed() const;
            Matrix3D& transpose();
            Matrix3D adjoint() const;
            Matrix3D inversed() const;
            Matrix3D& inverse();
            float determinant() const;
    };
}

#endif // ROS_MATRIX3D_H
