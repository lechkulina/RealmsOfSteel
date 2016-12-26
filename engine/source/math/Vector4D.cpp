/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Matrix4D.h>
#include <math/Vector2D.h>
#include <math/Vector3D.h>
#include <math/Vector4D.h>

const ros::Vector4D ros::Vector4D::ZERO(0.0f, 0.0f, 0.0f, 1.0f);
const ros::Vector4D ros::Vector4D::RIGHT_AXIS(1.0f, 0.0f, 0.0f, 1.0f);
const ros::Vector4D ros::Vector4D::UP_AXIS(0.0f, 1.0f, 0.0f, 1.0f);
const ros::Vector4D ros::Vector4D::FORWARD_AXIS(0.0f, 0.0f, -1.0f, 1.0f);

ros::Vector4D::Vector4D() {
}

ros::Vector4D::Vector4D(float x, float y, float z, float w)
    : x(x)
    , y(y)
    , z(z)
    , w(w) {
}

ros::Vector4D::Vector4D(const Vector4D& start, const Vector4D& end)
    : x(end.x - start.x)
    , y(end.y - start.y)
    , z(end.z - start.z)
    , w(1.0f) {
}

ros::Vector4D::Vector4D(const Vector2D& source)
    : x(source.x)
    , y(source.y)
    , z(0.0f)
    , w(1.0f) {
}

ros::Vector4D::Vector4D(const Vector3D& source)
    : x(source.x)
    , y(source.y)
    , z(source.z)
    , w(1.0f) {
}

ros::Vector4D ros::Vector4D::operator-() const {
    return Vector4D(-x, -y, -z, -w);
}

ros::Vector4D ros::Vector4D::operator+(const Vector4D& right) const {
    return Vector4D(
        x + right.x,
        y + right.y,
        z + right.z,
        w + right.w
    );
}

ros::Vector4D& ros::Vector4D::operator+=(const Vector4D& right) {
    x += right.x;
    y += right.y;
    z += right.z;
    w += right.w;

    return *this;
}

ros::Vector4D ros::Vector4D::operator-(const Vector4D& right) const {
    return Vector4D(
        x - right.x,
        y - right.y,
        z - right.z,
        w - right.w
    );
}

ros::Vector4D& ros::Vector4D::operator-=(const Vector4D& right) {
    x -= right.x;
    y -= right.y;
    z -= right.z;
    w -= right.w;

    return *this;
}

ros::Vector4D ros::Vector4D::operator*(float factor) const {
    return Vector4D(
        x * factor,
        y * factor,
        z * factor,
        w * factor
    );
}

ros::Vector4D& ros::Vector4D::operator*=(float factor) {
    x *= factor;
    y *= factor;
    z *= factor;
    w *= factor;

    return *this;
}

ros::Vector4D ros::Vector4D::operator*(const Vector4D& right) const {
    return Vector4D(
        x * right.x,
        y * right.y,
        z * right.z,
        w * right.w
    );
}

ros::Vector4D& ros::Vector4D::operator*=(const Vector4D& right) {
    x *= right.x;
    y *= right.y;
    z *= right.z;
    w *= right.w;

    return *this;
}

float ros::Vector4D::dot(const Vector4D& right) const {
    return x * right.x + y * right.y + z * right.z + w * right.w;
}

float ros::Vector4D::angel(const Vector4D& right) const {
    float factor = length() * right.length();
    if (isEqual(factor, 0.0f)) {
        return 0.0f;
    }
    return acosf(dot(right) / factor);
}

float ros::Vector4D::angelBetweenUnitVectors(const Vector4D& right) const {
    return acosf(dot(right));
}

ros::Vector4D ros::Vector4D::cross(const Vector4D& right) const {
    return Vector4D(
        y * right.z - z * right.y,
        z * right.x - x * right.z,
        x * right.y - y * right.x,
        1.0f
    );
}

ros::Vector4D ros::Vector4D::normalized() const {
    float len = length();
    if (isEqual(len, 0.0f)) {
        return Vector4D(0.0f, 0.0f, 0.0f, 1.0f);
    }

    float inversedLen = 1.0f / len;
    return Vector4D(
        x * inversedLen,
        y * inversedLen,
        z * inversedLen,
        w * inversedLen
    );
}

ros::Vector4D& ros::Vector4D::normalize() {
    float len = length();
    if (isEqual(len, 0.0f)) {
        return *this;
    }

    float inversedLen = 1.0f / len;
    x *= inversedLen;
    y *= inversedLen;
    z *= inversedLen;
    w *= inversedLen;
    return *this;
}

float ros::Vector4D::squaredLength() const {
    return x * x + y * y + z * z + w * w;
}

float ros::Vector4D::length() const {
    return sqrtf(x * x + y * y + z * z + w * w);
}

ros::Vector4D ros::Vector4D::parallel(const Vector4D& adjacent) const {
    float adjacentLen = adjacent.length();
    if (isEqual(adjacentLen, 0.0f)) {
        return Vector4D(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // assume that adjacent vector is not normalized
    float adjacentFactor = dot(adjacent) / adjacentLen * adjacentLen;
    return Vector4D(
        adjacent.x * adjacentFactor,
        adjacent.y * adjacentFactor,
        adjacent.z * adjacentFactor,
        adjacent.w * adjacentFactor
    );
}

ros::Vector4D ros::Vector4D::parallelToUnitVector(const Vector4D& adjacent) const {
    float adjacentFactor = dot(adjacent);
    return Vector4D(
        adjacent.x * adjacentFactor,
        adjacent.y * adjacentFactor,
        adjacent.z * adjacentFactor,
        adjacent.w * adjacentFactor
    );
}

ros::Vector4D ros::Vector4D::perpendicular(const Vector4D& adjacent) const {
    Vector4D projected = parallel(adjacent);
    return Vector4D(
        x - projected.x,
        y - projected.y,
        z - projected.z,
        w - projected.w
    );
}

ros::Vector4D ros::Vector4D::perpendicularToUnitVector(const Vector4D& adjacent) const {
    Vector4D projected = parallelToUnitVector(adjacent);
    return Vector4D(
        x - projected.x,
        y - projected.y,
        z - projected.z,
        w - projected.w
    );
}

ros::Vector4D ros::Vector4D::reflect(const Vector4D& normal) const {
    float normalLen = normal.length();
    if (isEqual(normalLen, 0.0f)) {
        return Vector4D(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // assume that normal vector is not normalized
    float normalFactor = 2.0f * dot(normal) / normalLen * normalLen;
    return Vector4D(
        normal.x * normalFactor - x,
        normal.y * normalFactor - y,
        normal.z * normalFactor - z,
        normal.w * normalFactor - w
    );
}

ros::Vector4D ros::Vector4D::reflectAgainstUnitVector(const Vector4D& normal) const {
    float normalFactor = 2.0f * dot(normal);
    return Vector4D(
        normal.x * normalFactor - x,
        normal.y * normalFactor - y,
        normal.z * normalFactor - z,
        normal.w * normalFactor - w
    );
}
