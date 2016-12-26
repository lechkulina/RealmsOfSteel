/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Vector3D.h>
#include <math/Matrix2D.h>
#include <math/Matrix4D.h>
#include <math/EulerAngles.h>
#include <math/Quaternion.h>
#include <math/Matrix3D.h>

const ros::Matrix3D ros::Matrix3D::ZERO(0.0f);
const ros::Matrix3D ros::Matrix3D::IDENTITY(1.0f);

ros::Matrix3D::Matrix3D() {
}

ros::Matrix3D::Matrix3D(float diagonal)
  : m11(diagonal), m12(0.0f), m13(0.0f)
  , m21(0.0f), m22(diagonal), m23(0.0f)
  , m31(0.0f), m32(0.0f), m33(diagonal) {
}

ros::Matrix3D::Matrix3D(float m11, float m12, float m13,
                        float m21, float m22, float m23,
                        float m31, float m32, float m33)
    : m11(m11), m12(m12), m13(m13)
    , m21(m21), m22(m22), m23(m23)
    , m31(m31), m32(m32), m33(m33) {
}

ros::Matrix3D::Matrix3D(const Matrix2D& source)
    : m11(source.m11), m12(source.m12), m13(0.0f)
    , m21(source.m21), m22(source.m22), m23(0.0f)
    , m31(0.0f), m32(0.0f), m33(1.0f) {
}

ros::Matrix3D::Matrix3D(const Matrix4D& source)
    : m11(source.m11), m12(source.m12), m13(source.m13)
    , m21(source.m21), m22(source.m22), m23(source.m23)
    , m31(source.m31), m32(source.m32), m33(source.m33) {
}

ros::Matrix3D::Matrix3D(const EulerAngles& orientation)
    : m11(1.0f), m12(0.0f), m13(0.0f)
    , m21(0.0f), m22(1.0f), m23(0.0f)
    , m31(0.0f), m32(0.0f), m33(1.0f) {

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

ros::Matrix3D::Matrix3D(const Quaternion& orientation)
    : m11(1.0f), m12(0.0f), m13(0.0f)
    , m21(0.0f), m22(1.0f), m23(0.0f)
    , m31(0.0f), m32(0.0f), m33(1.0f) {

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

ros::Matrix3D ros::Matrix3D::scaling(float factor) {
    return Matrix3D(factor);
}

ros::Matrix3D ros::Matrix3D::rotationX(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix3D(
        1.0f, 0.0f, 0.0f,
        0.0f, angleCos, -angleSin,
        0.0f, angleSin, angleCos
    );
}

ros::Matrix3D ros::Matrix3D::rotationY(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix3D(
        angleCos, 0.0f, angleSin,
        0.0f, 1.0f, 0.0f,
        -angleSin, 0.0f, angleCos
    );
}

ros::Matrix3D ros::Matrix3D::rotationZ(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix3D(
        angleCos, -angleSin, 0.0f,
        angleSin, angleCos, 0.0f,
        0.0f, 0.0f, 1.0f
    );
}

ros::Matrix3D ros::Matrix3D::operator+(const Matrix3D& right) const {
    return Matrix3D(
        m11 + right.m11,
        m12 + right.m12,
        m13 + right.m13,
        m21 + right.m21,
        m22 + right.m22,
        m23 + right.m23,
        m31 + right.m31,
        m32 + right.m32,
        m33 + right.m33
    );
}

ros::Matrix3D& ros::Matrix3D::operator+=(const Matrix3D& right) {
    m11 += right.m11;
    m12 += right.m12;
    m13 += right.m13;
    m21 += right.m21;
    m22 += right.m22;
    m23 += right.m23;
    m31 += right.m31;
    m32 += right.m32;
    m33 += right.m33;

    return *this;
}

ros::Matrix3D ros::Matrix3D::operator-(const Matrix3D& right) const {
    return Matrix3D(
        m11 - right.m11,
        m12 - right.m12,
        m13 - right.m13,
        m21 - right.m21,
        m22 - right.m22,
        m23 - right.m23,
        m31 - right.m31,
        m32 - right.m32,
        m33 - right.m33
    );
}

ros::Matrix3D& ros::Matrix3D::operator-=(const Matrix3D& right) {
    m11 -= right.m11;
    m12 -= right.m12;
    m13 -= right.m13;
    m21 -= right.m21;
    m22 -= right.m22;
    m23 -= right.m23;
    m31 -= right.m31;
    m32 -= right.m32;
    m33 -= right.m33;

    return *this;
}

ros::Matrix3D ros::Matrix3D::operator*(float factor) const {
    return Matrix3D(
        m11 * factor,
        m12 * factor,
        m13 * factor,
        m21 * factor,
        m22 * factor,
        m23 * factor,
        m31 * factor,
        m32 * factor,
        m33 * factor
    );
}

ros::Matrix3D& ros::Matrix3D::operator*=(float factor) {
    m11 *= factor;
    m12 *= factor;
    m13 *= factor;
    m21 *= factor;
    m22 *= factor;
    m23 *= factor;
    m31 *= factor;
    m32 *= factor;
    m33 *= factor;

    return *this;
}


ros::Matrix3D ros::Matrix3D::operator*(const Matrix3D& right) const {
    return Matrix3D(
        m11 * right.m11 + m12 * right.m21 + m13 * right.m31,
        m11 * right.m12 + m12 * right.m22 + m13 * right.m32,
        m11 * right.m13 + m12 * right.m23 + m13 * right.m33,
        m21 * right.m11 + m22 * right.m21 + m23 * right.m31,
        m21 * right.m12 + m22 * right.m22 + m23 * right.m32,
        m21 * right.m13 + m22 * right.m23 + m23 * right.m33,
        m31 * right.m11 + m32 * right.m21 + m33 * right.m31,
        m31 * right.m12 + m32 * right.m22 + m33 * right.m32,
        m31 * right.m13 + m32 * right.m23 + m33 * right.m33
    );
}

ros::Matrix3D& ros::Matrix3D::operator*=(const Matrix3D& right) {
    Matrix3D left(*this);
    m11 = left.m11 * right.m11 + left.m12 * right.m21 + left.m13 * right.m31;
    m12 = left.m11 * right.m12 + left.m12 * right.m22 + left.m13 * right.m32;
    m13 = left.m11 * right.m13 + left.m12 * right.m23 + left.m13 * right.m33;
    m21 = left.m21 * right.m11 + left.m22 * right.m21 + left.m23 * right.m31;
    m22 = left.m21 * right.m12 + left.m22 * right.m22 + left.m23 * right.m32;
    m23 = left.m21 * right.m13 + left.m22 * right.m23 + left.m23 * right.m33;
    m31 = left.m31 * right.m11 + left.m32 * right.m21 + left.m33 * right.m31;
    m32 = left.m31 * right.m12 + left.m32 * right.m22 + left.m33 * right.m32;
    m33 = left.m31 * right.m13 + left.m32 * right.m23 + left.m33 * right.m33;

    return *this;
}

ros::Vector3D ros::Matrix3D::operator*(const Vector3D& right) const {
    return Vector3D(
        m11 * right.x + m12 * right.y + m13 * right.z,
        m21 * right.x + m22 * right.y + m23 * right.z,
        m31 * right.x + m32 * right.y + m33 * right.z
    );
}

ros::Matrix3D ros::Matrix3D::transposed() const {
    return Matrix3D(
        m11, m21, m31,
        m12, m22, m32,
        m13, m23, m33
    );
}

ros::Matrix3D& ros::Matrix3D::transpose() {
    Matrix3D left(*this);
    m11 = left.m11;
    m12 = left.m21;
    m13 = left.m31;
    m21 = left.m12;
    m22 = left.m22;
    m23 = left.m32;
    m31 = left.m13;
    m32 = left.m23;
    m33 = left.m33;

    return *this;
}

ros::Matrix3D ros::Matrix3D::adjoint() const {
    return Matrix3D(
         m22 * m33 - m23 * m32,
        -m21 * m33 + m23 * m31,
         m21 * m32 - m22 * m31,
        -m12 * m33 + m13 * m32,
         m11 * m33 - m13 * m31,
        -m11 * m32 + m12 * m31,
         m12 * m23 - m13 * m22,
        -m11 * m23 + m13 * m21,
         m11 * m22 - m12 * m21
    );
}

ros::Matrix3D ros::Matrix3D::inversed() const {
    // check if this matrix is singular
    Matrix3D cofactors = adjoint();
    float det = m11 * cofactors.m11 + m12 * cofactors.m12 + m13 * cofactors.m13;
    if (isEqual(det, 0.0f)) {
        return Matrix3D(1.0f);
    }

    // divide the classical adjoint members by determinant the and transpose the result in one step
    float inversedDet = 1.0f / det;
    return Matrix3D(
        cofactors.m11 * inversedDet,
        cofactors.m21 * inversedDet,
        cofactors.m31 * inversedDet,
        cofactors.m12 * inversedDet,
        cofactors.m22 * inversedDet,
        cofactors.m32 * inversedDet,
        cofactors.m13 * inversedDet,
        cofactors.m23 * inversedDet,
        cofactors.m33 * inversedDet
    );
}

ros::Matrix3D& ros::Matrix3D::inverse() {
    // check if this matrix is singular
    Matrix3D cofactors = adjoint();
    float det = m11 * cofactors.m11 + m12 * cofactors.m12 + m13 * cofactors.m13;
    if (isEqual(det, 0.0f)) {
        return *this;
    }

    // divide the classical adjoint members by determinant the and transpose the result in one step
    float inversedDet = 1.0f / det;
    m11 = cofactors.m11 * inversedDet;
    m12 = cofactors.m21 * inversedDet;
    m13 = cofactors.m31 * inversedDet;
    m21 = cofactors.m12 * inversedDet;
    m22 = cofactors.m22 * inversedDet;
    m23 = cofactors.m32 * inversedDet;
    m31 = cofactors.m13 * inversedDet;
    m32 = cofactors.m23 * inversedDet;
    m33 = cofactors.m33 * inversedDet;
    return *this;
}

float ros::Matrix3D::determinant() const {
    // compute cofactors only for the first row and linearly combine the them with the members
    float cofactor11 =  m22 * m33 - m23 * m32;
    float cofactor12 = -m21 * m33 + m23 * m31;
    float cofactor13 =  m21 * m32 - m22 * m31;
    return m11 * cofactor11 + m12 * cofactor12 + m13 * cofactor13;
}
