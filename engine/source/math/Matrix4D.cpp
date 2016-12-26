/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Vector4D.h>
#include <math/Matrix2D.h>
#include <math/Matrix3D.h>
#include <math/EulerAngles.h>
#include <math/Quaternion.h>
#include <math/Matrix4D.h>

const ros::Matrix4D ros::Matrix4D::ZERO(0.0f);
const ros::Matrix4D ros::Matrix4D::IDENTITY(1.0f);

ros::Matrix4D::Matrix4D() {
}

ros::Matrix4D::Matrix4D(float diagonal)
    : m11(diagonal), m12(0.0f), m13(0.0f), m14(0.0f)
    , m21(0.0f), m22(diagonal), m23(0.0f), m24(0.0f)
    , m31(0.0f), m32(0.0f), m33(diagonal), m34(0.0f)
    , m41(0.0f), m42(0.0f), m43(0.0f), m44(diagonal) {
}

ros::Matrix4D::Matrix4D(float m11, float m12, float m13, float m14,
                        float m21, float m22, float m23, float m24,
                        float m31, float m32, float m33, float m34,
                        float m41, float m42, float m43, float m44)
    : m11(m11), m12(m12), m13(m13), m14(m14)
    , m21(m21), m22(m22), m23(m23), m24(m24)
    , m31(m31), m32(m32), m33(m33), m34(m34)
    , m41(m41), m42(m42), m43(m43), m44(m44) {
}

ros::Matrix4D::Matrix4D(const Matrix2D& source)
    : m11(source.m11), m12(source.m12), m13(0.0f), m14(0.0f)
    , m21(source.m21), m22(source.m22), m23(0.0f), m24(0.0f)
    , m31(0.0f), m32(0.0f), m33(1.0f), m34(0.0f)
    , m41(0.0f), m42(0.0f), m43(0.0f), m44(1.0f) {
}

ros::Matrix4D::Matrix4D(const Matrix3D& source)
    : m11(source.m11) , m12(source.m12) , m13(source.m13) , m14(0.0f)
    , m21(source.m21) , m22(source.m22) , m23(source.m23) , m24(0.0f)
    , m31(source.m31) , m32(source.m32) , m33(source.m33) , m34(0.0f)
    , m41(0.0f) , m42(0.0f) , m43(0.0f) , m44(1.0f) {
}

ros::Matrix4D::Matrix4D(const EulerAngles& orientation)
    : m11(1.0f), m12(0.0f), m13(0.0f), m14(0.0f)
    , m21(0.0f), m22(1.0f), m23(0.0f), m24(0.0f)
    , m31(0.0f), m32(0.0f), m33(1.0f), m34(0.0f)
    , m41(0.0f), m42(0.0f), m43(0.0f), m44(1.0f) {

    // compute the object-to-upright space rotation matrix
    float headingSin = sinf(orientation.heading);
    float headingCos = cosf(orientation.heading);
    float pitchSin = sinf(orientation.pitch);
    float pitchCos = cosf(orientation.pitch);
    float bankSin = sinf(orientation.bank);
    float bankCos = cosf(orientation.bank);

    // combine all three rotation matrices that represent the individual Euler rotations
    m11 = headingCos * bankCos + headingSin * pitchSin * bankSin;
    m12 = headingSin * pitchSin * bankCos - headingCos * bankSin;
    m13 = headingSin * pitchCos;
    m21 = bankSin * pitchCos;
    m22 = bankCos * pitchCos;
    m23 = -pitchSin;
    m31 = headingCos * pitchSin * bankSin - headingSin * bankCos;
    m32 = bankSin * headingSin + headingCos * pitchSin * bankCos;
    m33 = headingCos * pitchCos;
}

ros::Matrix4D::Matrix4D(const Quaternion& orientation)
    : m11(1.0f), m12(0.0f), m13(0.0f), m14(0.0f)
    , m21(0.0f), m22(1.0f), m23(0.0f), m24(0.0f)
    , m31(0.0f), m32(0.0f), m33(1.0f), m34(0.0f)
    , m41(0.0f), m42(0.0f), m43(0.0f), m44(1.0f) {

    // first compute the diagonal elements
    float xxCoefficient = 2.0f * orientation.x * orientation.x;
    float yyCoefficient = 2.0f * orientation.y * orientation.y;
    float zzCoefficient = 2.0f * orientation.z * orientation.z;
    m11 = 1.0f - yyCoefficient - zzCoefficient;
    m22 = 1.0f - xxCoefficient - zzCoefficient;
    m33 = 1.0f - xxCoefficient - yyCoefficient;

    // then the non-diagonal elements
    float xyCoefficient = 2.0f * orientation.x * orientation.y;
    float wzCoefficient = 2.0f * orientation.w * orientation.z;
    float xzCoefficient = 2.0f * orientation.x * orientation.z;
    float wyCoefficient = 2.0f * orientation.w * orientation.y;
    float yzCoefficient = 2.0f * orientation.y * orientation.z;
    float wxCoefficient = 2.0f * orientation.w * orientation.x;
    m12 = xyCoefficient - wzCoefficient;
    m13 = xzCoefficient + wyCoefficient;
    m21 = xyCoefficient + wzCoefficient;
    m23 = yzCoefficient - wxCoefficient;
    m31 = xzCoefficient - wyCoefficient;
    m32 = yzCoefficient + wxCoefficient;
}

ros::Matrix4D ros::Matrix4D::translation(float x, float y, float z) {
    return Matrix4D(
        1.0f, 0.0f, 0.0f, x,
        0.0f, 1.0f, 0.0f, y,
        0.0f, 0.0f, 1.0f, z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::scaling(float factor) {
    return Matrix4D(factor);
}

ros::Matrix4D ros::Matrix4D::rotationX(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix4D(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, angleCos, -angleSin, 0.0f,
        0.0f, angleSin,  angleCos, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::rotationY(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix4D(
        angleCos, 0.0f, angleSin, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        -angleSin, 0.0f, angleCos, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::rotationZ(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix4D(
        angleCos, -angleSin, 0.0f, 0.0f,
        angleSin, angleCos, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::perspectiveProjection(float zoom, float aspectRatio, float front, float back) {
    // compute frustum dimensions and check divide-by-zero situations
    float depth = back - front;
    if (isEqual(aspectRatio, 0.0f) || isEqual(depth, 0.0f)) {
        return Matrix4D(1.0f);
    }

    // compute the projection matrix assuming clip space z range [-1, +1]
    return Matrix4D(
        zoom, 0.0f, 0.0f, 0.0f,
        0.0f, zoom / aspectRatio, 0.0f, 0.0f,
        0.0f, 0.0f, -(back + front) / depth, -(2.0f * front * back) / depth,
        0.0f, 0.0f, -1.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::perspectiveProjection(float left, float right, float bottom,
                                                   float top, float front, float back) {
    // compute frustum dimensions and check divide-by-zero situations
    float halfWidth = (right - left) / 2;
    float halfHeight = (top - bottom) / 2;
    float depth = back - front;
    if (isEqual(halfWidth, 0.0f) || isEqual(halfHeight, 0.0f) || isEqual(depth, 0.0f)) {
        return Matrix4D(1.0f);
    }

    // compute the projection matrix assuming clip space z range [-1, +1]
    return Matrix4D(
        front / halfWidth, 0.0f, 0.0f, 0.0f,
        0.0f, front / halfHeight, 0.0f, 0.0f,
        0.0f, 0.0f, -(back + front) / depth, -(2.0f * front * back) / depth,
        0.0f, 0.0f, -1.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::orthographicProjection(float zoom, float aspectRatio, float front, float back) {
    // compute depth of the frustum and check divide-by-zero situations
    float depth = back - front;
    if (isEqual(aspectRatio, 0.0f) || isEqual(depth, 0.0f)) {
        return Matrix4D(1.0f);
    }

    // compute the projection matrix assuming clip space z range [-1, +1]
    return Matrix4D(
        zoom, 0.0f, 0.0f, 0.0f,
        0.0f, zoom / aspectRatio, 0.0f, 0.0f,
        0.0f, 0.0f, -2.0f / depth, -(back + front) / depth,
        0.0f, 0.0f, -1.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::orthographicProjection(float left, float right, float bottom,
                                                    float top, float front, float back) {
    // compute frustum dimensions and check divide-by-zero situations
    float halfWidth = (right - left) / 2;
    float halfHeight = (top - bottom) / 2;
    float depth = back - front;
    if (!isEqual(halfWidth, 0.0f) && !isEqual(halfHeight, 0.0f) && !isEqual(depth, 0.0f)) {
        return Matrix4D(1.0f);
    }

    // compute the projection matrix assuming clip space z range [-1, +1]
    return Matrix4D(
        1.0f / halfWidth, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f / halfHeight, 0.0f, 0.0f,
        0.0f, 0.0f, -2.0f / depth, -(back + front) / depth,
        0.0f, 0.0f, -1.0f, 1.0f
    );
}

ros::Matrix4D ros::Matrix4D::operator+(const Matrix4D& right) const {
    return Matrix4D(
        m11 + right.m11,
        m12 + right.m12,
        m13 + right.m13,
        m14 + right.m14,
        m21 + right.m21,
        m22 + right.m22,
        m23 + right.m23,
        m24 + right.m24,
        m31 + right.m31,
        m32 + right.m32,
        m33 + right.m33,
        m34 + right.m34,
        m41 + right.m41,
        m42 + right.m42,
        m43 + right.m43,
        m44 + right.m44
    );
}

ros::Matrix4D& ros::Matrix4D::operator+=(const Matrix4D& right) {
    m11 += right.m11;
    m12 += right.m12;
    m13 += right.m13;
    m14 += right.m14;
    m21 += right.m21;
    m22 += right.m22;
    m23 += right.m23;
    m24 += right.m24;
    m31 += right.m31;
    m32 += right.m32;
    m33 += right.m33;
    m34 += right.m34;
    m41 += right.m41;
    m42 += right.m42;
    m43 += right.m43;
    m44 += right.m44;

    return *this;
}

ros::Matrix4D ros::Matrix4D::operator-(const Matrix4D& right) const {
    return Matrix4D(
        m11 - right.m11,
        m12 - right.m12,
        m13 - right.m13,
        m14 - right.m14,
        m21 - right.m21,
        m22 - right.m22,
        m23 - right.m23,
        m24 - right.m24,
        m31 - right.m31,
        m32 - right.m32,
        m33 - right.m33,
        m34 - right.m34,
        m41 - right.m41,
        m42 - right.m42,
        m43 - right.m43,
        m44 - right.m44
    );
}

ros::Matrix4D& ros::Matrix4D::operator-=(const Matrix4D& right) {
    m11 -= right.m11;
    m12 -= right.m12;
    m13 -= right.m13;
    m14 -= right.m14;
    m21 -= right.m21;
    m22 -= right.m22;
    m23 -= right.m23;
    m24 -= right.m24;
    m31 -= right.m31;
    m32 -= right.m32;
    m33 -= right.m33;
    m34 -= right.m34;
    m41 -= right.m41;
    m42 -= right.m42;
    m43 -= right.m43;
    m44 -= right.m44;

    return *this;
}

ros::Matrix4D ros::Matrix4D::operator*(float factor) const {
    return Matrix4D(
        m11 * factor,
        m12 * factor,
        m13 * factor,
        m14 * factor,
        m21 * factor,
        m22 * factor,
        m23 * factor,
        m24 * factor,
        m31 * factor,
        m32 * factor,
        m33 * factor,
        m34 * factor,
        m41 * factor,
        m42 * factor,
        m43 * factor,
        m44 * factor
    );
}

ros::Matrix4D& ros::Matrix4D::operator*=(float factor) {
    m11 *= factor;
    m12 *= factor;
    m13 *= factor;
    m14 *= factor;
    m21 *= factor;
    m22 *= factor;
    m23 *= factor;
    m24 *= factor;
    m31 *= factor;
    m32 *= factor;
    m33 *= factor;
    m34 *= factor;
    m41 *= factor;
    m42 *= factor;
    m43 *= factor;
    m44 *= factor;

    return *this;
}

ros::Matrix4D ros::Matrix4D::operator*(const Matrix4D& right) const {
    return Matrix4D(
        m11 * right.m11 + m12 * right.m21 + m13 * right.m31 + m14 * right.m41,
        m11 * right.m12 + m12 * right.m22 + m13 * right.m32 + m14 * right.m42,
        m11 * right.m13 + m12 * right.m23 + m13 * right.m33 + m14 * right.m43,
        m11 * right.m14 + m12 * right.m24 + m13 * right.m34 + m14 * right.m44,
        m21 * right.m11 + m22 * right.m21 + m23 * right.m31 + m24 * right.m41,
        m21 * right.m12 + m22 * right.m22 + m23 * right.m32 + m24 * right.m42,
        m21 * right.m13 + m22 * right.m23 + m23 * right.m33 + m24 * right.m43,
        m21 * right.m14 + m22 * right.m24 + m23 * right.m34 + m24 * right.m44,
        m31 * right.m11 + m32 * right.m21 + m33 * right.m31 + m34 * right.m41,
        m31 * right.m12 + m32 * right.m22 + m33 * right.m32 + m34 * right.m42,
        m31 * right.m13 + m32 * right.m23 + m33 * right.m33 + m34 * right.m43,
        m31 * right.m14 + m32 * right.m24 + m33 * right.m34 + m34 * right.m44,
        m41 * right.m11 + m42 * right.m21 + m43 * right.m31 + m44 * right.m41,
        m41 * right.m12 + m42 * right.m22 + m43 * right.m32 + m44 * right.m42,
        m41 * right.m13 + m42 * right.m23 + m43 * right.m33 + m44 * right.m43,
        m41 * right.m14 + m42 * right.m24 + m43 * right.m34 + m44 * right.m44
    );
}

ros::Matrix4D& ros::Matrix4D::operator*=(const Matrix4D& right) {
    Matrix4D left(*this);
    m11 = left.m11 * right.m11 + left.m12 * right.m21 + left.m13 * right.m31 + left.m14 * right.m41;
    m12 = left.m11 * right.m12 + left.m12 * right.m22 + left.m13 * right.m32 + left.m14 * right.m42;
    m13 = left.m11 * right.m13 + left.m12 * right.m23 + left.m13 * right.m33 + left.m14 * right.m43;
    m14 = left.m11 * right.m14 + left.m12 * right.m24 + left.m13 * right.m34 + left.m14 * right.m44;
    m21 = left.m21 * right.m11 + left.m22 * right.m21 + left.m23 * right.m31 + left.m24 * right.m41;
    m22 = left.m21 * right.m12 + left.m22 * right.m22 + left.m23 * right.m32 + left.m24 * right.m42;
    m23 = left.m21 * right.m13 + left.m22 * right.m23 + left.m23 * right.m33 + left.m24 * right.m43;
    m24 = left.m21 * right.m14 + left.m22 * right.m24 + left.m23 * right.m34 + left.m24 * right.m44;
    m31 = left.m31 * right.m11 + left.m32 * right.m21 + left.m33 * right.m31 + left.m34 * right.m41;
    m32 = left.m31 * right.m12 + left.m32 * right.m22 + left.m33 * right.m32 + left.m34 * right.m42;
    m33 = left.m31 * right.m13 + left.m32 * right.m23 + left.m33 * right.m33 + left.m34 * right.m43;
    m34 = left.m31 * right.m14 + left.m32 * right.m24 + left.m33 * right.m34 + left.m34 * right.m44;
    m41 = left.m41 * right.m11 + left.m42 * right.m21 + left.m43 * right.m31 + left.m44 * right.m41;
    m42 = left.m41 * right.m12 + left.m42 * right.m22 + left.m43 * right.m32 + left.m44 * right.m42;
    m43 = left.m41 * right.m13 + left.m42 * right.m23 + left.m43 * right.m33 + left.m44 * right.m43;
    m44 = left.m41 * right.m14 + left.m42 * right.m24 + left.m43 * right.m34 + left.m44 * right.m44;

    return *this;
}

ros::Vector4D ros::Matrix4D::operator*(const Vector4D& right) const {
    return Vector4D(
        m11 * right.x + m12 * right.y + m13 * right.z + m14 * right.w,
        m21 * right.x + m22 * right.y + m23 * right.z + m24 * right.w,
        m31 * right.x + m32 * right.y + m33 * right.z + m34 * right.w,
        m41 * right.x + m42 * right.y + m43 * right.z + m44 * right.w
    );
}

ros::Matrix4D ros::Matrix4D::transposed() const {
    return Matrix4D(
        m11, m21, m31, m41,
        m12, m22, m32, m42,
        m13, m23, m33, m43,
        m14, m24, m34, m44
    );
}

ros::Matrix4D& ros::Matrix4D::transpose() {
    Matrix4D left(*this);
    m11 = left.m11;
    m12 = left.m21;
    m13 = left.m31;
    m14 = left.m41;
    m21 = left.m12;
    m22 = left.m22;
    m23 = left.m32;
    m24 = left.m42;
    m31 = left.m13;
    m32 = left.m23;
    m33 = left.m33;
    m34 = left.m43;
    m41 = left.m14;
    m42 = left.m24;
    m43 = left.m34;
    m44 = left.m44;

    return *this;
}

ros::Matrix4D ros::Matrix4D::adjoint() const {
    return Matrix4D(
         m22 * (m33 * m44 - m34 * m43) - m23 * (m32 * m44 - m34 * m42) + m24 * (m32 * m43 - m33 * m42),
        -m21 * (m33 * m44 - m34 * m43) + m23 * (m31 * m44 - m34 * m41) - m24 * (m31 * m43 - m33 * m41),
         m21 * (m32 * m44 - m34 * m42) - m22 * (m31 * m44 - m34 * m41) + m24 * (m31 * m42 - m32 * m41),
        -m21 * (m32 * m43 - m33 * m42) + m22 * (m31 * m43 - m33 * m41) - m23 * (m31 * m42 - m32 * m41),
        -m12 * (m33 * m44 - m34 * m43) + m13 * (m32 * m44 - m34 * m42) - m14 * (m32 * m43 - m33 * m42),
         m11 * (m33 * m44 - m34 * m43) - m13 * (m31 * m44 - m34 * m41) + m14 * (m31 * m43 - m33 * m41),
        -m11 * (m32 * m44 - m34 * m42) + m12 * (m31 * m44 - m34 * m41) - m14 * (m31 * m42 - m32 * m41),
         m11 * (m32 * m43 - m33 * m42) - m12 * (m31 * m43 - m33 * m41) + m13 * (m31 * m42 - m32 * m41),
         m12 * (m23 * m44 - m24 * m43) - m13 * (m22 * m44 - m24 * m42) + m14 * (m22 * m43 - m23 * m42),
        -m11 * (m23 * m44 - m24 * m43) + m13 * (m21 * m44 - m24 * m41) - m14 * (m21 * m43 - m23 * m41),
         m11 * (m22 * m44 - m24 * m42) - m12 * (m21 * m44 - m24 * m41) + m14 * (m21 * m42 - m22 * m41),
        -m11 * (m22 * m43 - m23 * m42) + m12 * (m21 * m43 - m23 * m41) - m13 * (m21 * m42 - m22 * m41),
        -m12 * (m23 * m34 - m24 * m33) + m13 * (m22 * m34 - m24 * m32) - m14 * (m22 * m33 - m23 * m32),
         m11 * (m23 * m34 - m24 * m33) - m13 * (m21 * m34 - m24 * m31) + m14 * (m21 * m33 - m23 * m31),
        -m11 * (m22 * m34 - m24 * m32) + m12 * (m21 * m34 - m24 * m31) - m14 * (m21 * m32 - m22 * m31),
         m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31)
    );
}

ros::Matrix4D ros::Matrix4D::inversed() const {
    // check if this matrix is singular
    Matrix4D cofactors = adjoint();
    float det = m11 * cofactors.m11 + m12 * cofactors.m12 + m13 * cofactors.m13 + m14 * cofactors.m14;
    if (isEqual(det, 0.0f)) {
         return Matrix4D(1.0f);
    }

    // divide the classical adjoint members by determinant the and transpose the result in one step
    float inversedDet = 1.0f / det;
    return Matrix4D(
        cofactors.m11 * inversedDet,
        cofactors.m21 * inversedDet,
        cofactors.m31 * inversedDet,
        cofactors.m41 * inversedDet,
        cofactors.m12 * inversedDet,
        cofactors.m22 * inversedDet,
        cofactors.m32 * inversedDet,
        cofactors.m42 * inversedDet,
        cofactors.m13 * inversedDet,
        cofactors.m23 * inversedDet,
        cofactors.m33 * inversedDet,
        cofactors.m43 * inversedDet,
        cofactors.m14 * inversedDet,
        cofactors.m24 * inversedDet,
        cofactors.m34 * inversedDet,
        cofactors.m44 * inversedDet
    );
}

ros::Matrix4D& ros::Matrix4D::inverse() {
    // check if this matrix is singular
    Matrix4D cofactors = adjoint();
    float det = m11 * cofactors.m11 + m12 * cofactors.m12 + m13 * cofactors.m13 + m14 * cofactors.m14;
    if (isEqual(det, 0.0f)) {
        return *this;
    }

    // divide the classical adjoint members by determinant the and transpose the result in one step
    float inversedDet = 1.0f / det;
    m11 = cofactors.m11 * inversedDet;
    m12 = cofactors.m21 * inversedDet;
    m13 = cofactors.m31 * inversedDet;
    m14 = cofactors.m41 * inversedDet;
    m21 = cofactors.m12 * inversedDet;
    m22 = cofactors.m22 * inversedDet;
    m23 = cofactors.m32 * inversedDet;
    m24 = cofactors.m42 * inversedDet;
    m31 = cofactors.m13 * inversedDet;
    m32 = cofactors.m23 * inversedDet;
    m33 = cofactors.m33 * inversedDet;
    m34 = cofactors.m43 * inversedDet;
    m41 = cofactors.m14 * inversedDet;
    m42 = cofactors.m24 * inversedDet;
    m43 = cofactors.m34 * inversedDet;
    m44 = cofactors.m44 * inversedDet;
    return *this;
}

float ros::Matrix4D::determinant() const {
    // compute cofactors only for the first row and lifrontly combine the them with the members
    float cofactor11 =  m22 * (m33 * m44 - m34 * m43) -
                        m23 * (m32 * m44 - m34 * m42) +
                        m24 * (m32 * m43 - m33 * m42);
    float cofactor12 = -m21 * (m33 * m44 - m34 * m43) +
                        m23 * (m31 * m44 - m34 * m41) -
                        m24 * (m31 * m43 - m33 * m41);
    float cofactor13 =  m21 * (m32 * m44 - m34 * m42) -
                        m22 * (m31 * m44 - m34 * m41) +
                        m24 * (m31 * m42 - m32 * m41);
    float cofactor14 = -m21 * (m32 * m43 - m33 * m42) +
                        m22 * (m31 * m43 - m33 * m41) -
                        m23 * (m31 * m42 - m32 * m41);
    return m11 * cofactor11 + m12 * cofactor12 + m13 * cofactor13 + m14 * cofactor14;
}
