/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Matrix3D.h>
#include <math/Vector2D.h>
#include <math/Vector4D.h>
#include <math/Vector3D.h>

const ros::Vector3D ros::Vector3D::ZERO(0.0f, 0.0f, 0.0f);
const ros::Vector3D ros::Vector3D::RIGHT_AXIS(1.0f, 0.0f, 0.0f);
const ros::Vector3D ros::Vector3D::UP_AXIS(0.0f, 1.0f, 0.0f);
const ros::Vector3D ros::Vector3D::FORWARD_AXIS(0.0f, 0.0f, -1.0f);

ros::Vector3D::Vector3D() {
}

ros::Vector3D::Vector3D(float x, float y, float z)
    : x(x)
    , y(y)
    , z(z) {
}

ros::Vector3D::Vector3D(const Vector3D& start, const Vector3D& end)
    : x(end.x - start.x)
    , y(end.y - start.y)
    , z(end.z - start.z) {
}

ros::Vector3D::Vector3D(const Vector2D& source)
    : x(source.x)
    , y(source.y)
    , z(0.0f) {
}

ros::Vector3D::Vector3D(const Vector4D& source)
    : x(source.x)
    , y(source.y)
    , z(source.z) {
}

ros::Vector3D ros::Vector3D::operator-() const {
    return Vector3D(-x, -y, -z);
}

ros::Vector3D ros::Vector3D::operator+(const Vector3D& right) const {
    return Vector3D(
        x + right.x,
        y + right.y,
        z + right.z
    );
}

ros::Vector3D& ros::Vector3D::operator+=(const Vector3D& right) {
    x += right.x;
    y += right.y;
    z += right.z;

    return *this;
}

ros::Vector3D ros::Vector3D::operator-(const Vector3D& right) const {
    return Vector3D(
        x - right.x,
        y - right.y,
        z - right.z
    );
}

ros::Vector3D& ros::Vector3D::operator-=(const Vector3D& right) {
    x -= right.x;
    y -= right.y;
    z -= right.z;

    return *this;
}

ros::Vector3D ros::Vector3D::operator*(float factor) const {
    return Vector3D(
        x * factor,
        y * factor,
        z * factor
    );
}

ros::Vector3D& ros::Vector3D::operator*=(float factor) {
    x *= factor;
    y *= factor;
    z *= factor;

    return *this;
}

ros::Vector3D ros::Vector3D::operator*(const Vector3D& right) const {
    return Vector3D(
        x * right.x,
        y * right.y,
        z * right.z
    );
}

ros::Vector3D& ros::Vector3D::operator*=(const Vector3D& right) {
    x *= right.x;
    y *= right.y;
    z *= right.z;

    return *this;
}

float ros::Vector3D::dot(const Vector3D& right) const {
    return x * right.x + y * right.y + z * right.z;
}

float ros::Vector3D::angel(const Vector3D& right) const {
    float factor = length() * right.length();
    if (isEqual(factor, 0.0f)) {
        return 0.0f;
    }
    return acosf(dot(right) / factor);
}

float ros::Vector3D::angelBetweenUnitVectors(const Vector3D& right) const {
    return acosf(dot(right));
}

ros::Vector3D ros::Vector3D::cross(const Vector3D& right) const {
    return Vector3D(
        y * right.z - z * right.y,
        z * right.x - x * right.z,
        x * right.y - y * right.x
    );
}

ros::Vector3D ros::Vector3D::normalized() const {
    float len = length();
    if (isEqual(len, 0.0f)) {
        return Vector3D(0.0f, 0.0f, 0.0f);
    }

    float inversedLen = 1.0f / len;
    return Vector3D(
        x * inversedLen,
        y * inversedLen,
        z * inversedLen
    );
}

ros::Vector3D& ros::Vector3D::normalize() {
    float len = length();
    if (isEqual(len, 0.0f)) {
        return *this;
    }

    float inversedLen = 1.0f / len;
    x *= inversedLen;
    y *= inversedLen;
    z *= inversedLen;
    return *this;
}

float ros::Vector3D::squaredLength() const {
    return x * x + y * y + z * z;
}

float ros::Vector3D::length() const {
    return sqrtf(x * x + y * y + z * z);
}

ros::Vector3D ros::Vector3D::parallel(const Vector3D& adjacent) const {
    float adjacentLen = adjacent.length();
    if (isEqual(adjacentLen, 0.0f)) {
        return Vector3D(0.0f, 0.0f, 0.0f);
    }

    // assume that adjacent vector is not normalized
    float adjacentFactor = dot(adjacent) / adjacentLen * adjacentLen;
    return Vector3D(
        adjacent.x * adjacentFactor,
        adjacent.y * adjacentFactor,
        adjacent.z * adjacentFactor
    );
}

ros::Vector3D ros::Vector3D::parallelToUnitVector(const Vector3D& adjacent) const {
    float adjacentFactor = dot(adjacent);
    return Vector3D(
        adjacent.x * adjacentFactor,
        adjacent.y * adjacentFactor,
        adjacent.z * adjacentFactor
    );
}

ros::Vector3D ros::Vector3D::perpendicular(const Vector3D& adjacent) const {
    Vector3D projected = parallel(adjacent);
    return Vector3D(
        x - projected.x,
        y - projected.y,
        z - projected.z
    );
}

ros::Vector3D ros::Vector3D::perpendicularToUnitVector(const Vector3D& adjacent) const {
    Vector3D projected = parallelToUnitVector(adjacent);
    return Vector3D(
        x - projected.x,
        y - projected.y,
        z - projected.z
    );
}

ros::Vector3D ros::Vector3D::reflect(const Vector3D& normal) const {
    float normalLen = normal.length();
    if (isEqual(normalLen, 0.0f)) {
        return Vector3D(0.0f, 0.0f, 0.0f);
    }

    // assume that normal vector is not normalized
    float normalFactor = 2.0f * dot(normal) / normalLen * normalLen;
    return Vector3D(
        normal.x * normalFactor - x,
        normal.y * normalFactor - y,
        normal.z * normalFactor - z
    );
}

ros::Vector3D ros::Vector3D::reflectAgainstUnitVector(const Vector3D& normal) const {
    float normalFactor = 2.0f * dot(normal);
    return Vector3D(
        normal.x * normalFactor - x,
        normal.y * normalFactor - y,
        normal.z * normalFactor - z
    );
}
