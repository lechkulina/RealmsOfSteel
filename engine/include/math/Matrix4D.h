/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_MATRIX4D_H
#define ROS_MATRIX4D_H

#include <core/Common.h>
#include <core/Environment.h>
#include <math/Scalar.h>
#include <math/Vector4D.h>

namespace ros {
    class Matrix2D;
    class Matrix3D;
    class EulerAngles;
    class Quaternion;

    class ROS_API Matrix4D {
        public:
            float m11, m12, m13, m14;
            float m21, m22, m23, m24;
            float m31, m32, m33, m34;
            float m41, m42, m43, m44;

            static const Matrix4D ZERO;
            static const Matrix4D IDENTITY;

            Matrix4D();
            Matrix4D(float diagonal);
            Matrix4D(float m11, float m12, float m13, float m14,
                     float m21, float m22, float m23, float m24,
                     float m31, float m32, float m33, float m34,
                     float m41, float m42, float m43, float m44);
            Matrix4D(const Matrix2D& source);
            Matrix4D(const Matrix3D& source);
            Matrix4D(const EulerAngles& orientation);
            Matrix4D(const Quaternion& orientation);

            static Matrix4D translation(float x, float y, float z);
            static Matrix4D scaling(float factor);
            static Matrix4D rotationX(float angle);
            static Matrix4D rotationY(float angle);
            static Matrix4D rotationZ(float angle);
            static Matrix4D perspectiveProjection(float zoom, float aspectRatio, float front, float back);
            static Matrix4D perspectiveProjection(float left, float right, float bottom,
                                                  float top, float front, float back);
            static Matrix4D orthographicProjection(float zoom, float aspectRatio, float front, float back);
            static Matrix4D orthographicProjection(float left, float right, float bottom,
                                                   float top, float front, float back);

            Matrix4D operator+(const Matrix4D& right) const;
            Matrix4D& operator+=(const Matrix4D& right);
            Matrix4D operator-(const Matrix4D& right) const;
            Matrix4D& operator-=(const Matrix4D& right);
            Matrix4D operator*(float factor) const;
            Matrix4D& operator*=(float factor);
            Matrix4D operator*(const Matrix4D& right) const;
            Matrix4D& operator*=(const Matrix4D& right);
            Vector4D operator*(const Vector4D& right) const;
            Matrix4D transposed() const;
            Matrix4D& transpose();
            Matrix4D adjoint() const;
            Matrix4D inversed() const;
            Matrix4D& inverse();
            float determinant() const;
    };
}

#endif // ROS_MATRIX4D_H
