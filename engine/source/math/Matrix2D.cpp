/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Vector2D.h>
#include <math/Matrix3D.h>
#include <math/Matrix4D.h>
#include <math/Matrix2D.h>

const ros::Matrix2D ros::Matrix2D::ZERO(0.0f);
const ros::Matrix2D ros::Matrix2D::IDENTITY(1.0f);

ros::Matrix2D::Matrix2D() {
}

ros::Matrix2D::Matrix2D(float diagonal)
    : m11(diagonal), m12(0.0f)
    , m21(0.0f), m22(diagonal) {
}

ros::Matrix2D::Matrix2D(float m11, float m12,
                        float m21, float m22)
    : m11(m11), m12(m12)
    , m21(m21), m22(m22) {
}

ros::Matrix2D::Matrix2D(const Matrix3D& source)
    : m11(source.m11), m12(source.m12)
    , m21(source.m21), m22(source.m22) {
}

ros::Matrix2D::Matrix2D(const Matrix4D& source)
    : m11(source.m11), m12(source.m12)
    , m21(source.m21), m22(source.m22) {
}

ros::Matrix2D ros::Matrix2D::scaling(float factor) {
    return Matrix2D(factor);
}

ros::Matrix2D ros::Matrix2D::rotation(float angle) {
    float angleCos = cosf(angle);
    float angleSin = sinf(angle);
    return Matrix2D(
        angleCos, angleSin,
        -angleSin, angleCos
    );
}

ros::Matrix2D ros::Matrix2D::operator+(const Matrix2D& right) const {
    return Matrix2D(
        m11 + right.m11,
        m12 + right.m12,
        m21 + right.m21,
        m22 + right.m22
    );
}

ros::Matrix2D& ros::Matrix2D::operator+=(const Matrix2D& right) {
    m11 += right.m11;
    m12 += right.m12;
    m21 += right.m21;
    m22 += right.m22;

    return *this;
}

ros::Matrix2D ros::Matrix2D::operator-(const Matrix2D& right) const {
    return Matrix2D(
        m11 - right.m11,
        m12 - right.m12,
        m21 - right.m21,
        m22 - right.m22
    );
}

ros::Matrix2D& ros::Matrix2D::operator-=(const Matrix2D& right) {
    m11 -= right.m11;
    m12 -= right.m12;
    m21 -= right.m21;
    m22 -= right.m22;

    return *this;
}

ros::Matrix2D ros::Matrix2D::operator*(float factor) const {
    return Matrix2D(
        m11 * factor,
        m12 * factor,
        m21 * factor,
        m22 * factor
    );
}

ros::Matrix2D& ros::Matrix2D::operator*=(float factor) {
    m11 *= factor;
    m12 *= factor;
    m21 *= factor;
    m22 *= factor;

    return *this;
}


ros::Matrix2D ros::Matrix2D::operator*(const Matrix2D& right) const {
    return Matrix2D(
        m11 * right.m11 + m12 * right.m21,
        m11 * right.m12 + m12 * right.m22,
        m21 * right.m11 + m22 * right.m21,
        m21 * right.m12 + m22 * right.m22
    );
}

ros::Matrix2D& ros::Matrix2D::operator*=(const Matrix2D& right) {
    Matrix2D left(*this);
    m11 = left.m11 * right.m11 + left.m12 * right.m21;
    m12 = left.m11 * right.m12 + left.m12 * right.m22;
    m21 = left.m21 * right.m11 + left.m22 * right.m21;
    m22 = left.m21 * right.m12 + left.m22 * right.m22;

    return *this;
}

ros::Vector2D ros::Matrix2D::operator*(const Vector2D& right) const {
    return Vector2D(
        m11 * right.x + m12 * right.y,
        m21 * right.x + m22 * right.y
    );
}

ros::Matrix2D ros::Matrix2D::transposed() const {
    return Matrix2D(
        m11, m21,
        m12, m22
    );
}

ros::Matrix2D& ros::Matrix2D::transpose() {
    Matrix2D left(*this);
    m11 = left.m11;
    m12 = left.m21;
    m21 = left.m12;
    m22 = left.m22;

    return *this;
}

ros::Matrix2D ros::Matrix2D::adjoint() const {
    return Matrix2D(
         m22, -m21,
        -m12, m11
    );
}

ros::Matrix2D ros::Matrix2D::inversed() const {
    // check if this matrix is singular
    Matrix2D cofactors = adjoint();
    float det = m11 * cofactors.m11 + m12 * cofactors.m12;
    if (isEqual(det, 0.0f)) {
        return Matrix2D(1.0f);
    }

    // divide the classical adjoint members by determinant the and transpose the result in one step
    float inversedDet = 1.0f / det;
    return Matrix2D(
       cofactors.m11 * inversedDet,
       cofactors.m21 * inversedDet,
       cofactors.m12 * inversedDet,
       cofactors.m22 * inversedDet
    );
}

ros::Matrix2D& ros::Matrix2D::inverse() {
    // check if this matrix is singular
    Matrix2D cofactors = adjoint();
    float det = m11 * cofactors.m11 + m12 * cofactors.m12;
    if (isEqual(det, 0.0f)) {
        return *this;
    }

    // divide the classical adjoint members by determinant the and transpose the result in one step
    float inversedDet = 1.0f / det;
    m11 = cofactors.m11 * inversedDet;
    m12 = cofactors.m21 * inversedDet;
    m21 = cofactors.m12 * inversedDet;
    m22 = cofactors.m22 * inversedDet;
    return *this;
}

float ros::Matrix2D::determinant() const {
    return m11 * m22 - m12 * m21;
}
