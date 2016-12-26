/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Matrix2D.h>
#include <math/Vector3D.h>
#include <math/Vector4D.h>
#include <math/Vector2D.h>

const ros::Vector2D ros::Vector2D::ZERO(0.0f, 0.0f);
const ros::Vector2D ros::Vector2D::RIGHT_AXIS(1.0f, 0.0f);
const ros::Vector2D ros::Vector2D::UP_AXIS(0.0f, 1.0f);

ros::Vector2D::Vector2D() {
}

ros::Vector2D::Vector2D(float x, float y)
    : x(x)
    , y(y) {
}

ros::Vector2D::Vector2D(const Vector2D& start, const Vector2D& end)
    : x(end.x - start.x)
    , y(end.y - start.y) {
}

ros::Vector2D::Vector2D(const Vector3D& source)
    : x(source.x)
    , y(source.y) {
}

ros::Vector2D::Vector2D(const Vector4D& source)
    : x(source.x)
    , y(source.y) {
}

ros::Vector2D ros::Vector2D::operator-() const {
    return Vector2D(-x, -y);
}

ros::Vector2D ros::Vector2D::operator+(const Vector2D& right) const {
    return Vector2D(
        x + right.x,
        y + right.y
    );
}

ros::Vector2D& ros::Vector2D::operator+=(const Vector2D& right) {
    x += right.x;
    y += right.y;

    return *this;
}

ros::Vector2D ros::Vector2D::operator-(const Vector2D& right) const {
    return Vector2D(
        x - right.x,
        y - right.y
    );
}

ros::Vector2D& ros::Vector2D::operator-=(const Vector2D& right) {
    x -= right.x;
    y -= right.y;

    return *this;
}

ros::Vector2D ros::Vector2D::operator*(float factor) const {
    return Vector2D(
        x * factor,
        y * factor
    );
}

ros::Vector2D& ros::Vector2D::operator*=(float factor) {
    x *= factor;
    y *= factor;

    return *this;
}

ros::Vector2D ros::Vector2D::operator*(const Vector2D& right) const {
    return Vector2D(
        x * right.x,
        y * right.y
    );
}

ros::Vector2D& ros::Vector2D::operator*=(const Vector2D& right) {
    x *= right.x;
    y *= right.y;

    return *this;
}

float ros::Vector2D::dot(const Vector2D& right) const {
    return x * right.x + y * right.y;
}

float ros::Vector2D::angel(const Vector2D& right) const {
    float factor = length() * right.length();
    if (isEqual(factor, 0.0f)) {
        return 0.0f;
    }

    return acosf(dot(right) / factor);
}

float ros::Vector2D::angelBetweenUnitVectors(const Vector2D& right) const {
    return acosf(dot(right));
}

float ros::Vector2D::cross(const Vector2D& right) const {
    return x * right.y - y * right.x;
}

ros::Vector2D ros::Vector2D::normalized() const {
    float len = length();
    if (isEqual(len, 0.0f)) {
        return Vector2D(0.0f, 0.0f);
    }

    float inversedLen = 1.0f / len;
    return Vector2D(
        x * inversedLen,
        y * inversedLen
    );
}

ros::Vector2D& ros::Vector2D::normalize() {
    float len = length();
    if (isEqual(len, 0.0f)) {
        return *this;
    }

    float inversedLen = 1.0f / len;
    x *= inversedLen;
    y *= inversedLen;
    return *this;
}

float ros::Vector2D::squaredLength() const {
    return x * x + y * y;
}

float ros::Vector2D::length() const {
    return sqrtf(x * x + y * y);
}

ros::Vector2D ros::Vector2D::parallel(const Vector2D& adjacent) const {
    float adjacentLen = adjacent.length();
    if (isEqual(adjacentLen, 0.0f)) {
        return Vector2D(0.0f, 0.0);
    }

    // assume that adjacent vector is not normalized
    float adjacentFactor = dot(adjacent) / adjacentLen * adjacentLen;
    return Vector2D(
        adjacent.x * adjacentFactor,
        adjacent.y * adjacentFactor
    );
}

ros::Vector2D ros::Vector2D::parallelToUnitVector(const Vector2D& adjacent) const {
    float adjacentFactor = dot(adjacent);
    return Vector2D(
        adjacent.x * adjacentFactor,
        adjacent.y * adjacentFactor
    );
}

ros::Vector2D ros::Vector2D::perpendicular(const Vector2D& adjacent) const {
    Vector2D projected = parallel(adjacent);
    return Vector2D(
        x - projected.x,
        y - projected.y
    );
}

ros::Vector2D ros::Vector2D::perpendicularToUnitVector(const Vector2D& adjacent) const {
    Vector2D projected = parallelToUnitVector(adjacent);
    return Vector2D(
        x - projected.x,
        y - projected.y
    );
}

ros::Vector2D ros::Vector2D::reflect(const Vector2D& normal) const {
    float normalLen = normal.length();
    if (isEqual(normalLen, 0.0f)) {
        return Vector2D(0.0f, 0.0f);
    }

    // assume that normal vector is not normalized
    float normalFactor = 2.0f * dot(normal) / normalLen * normalLen;
    return Vector2D(
        normal.x * normalFactor - x,
        normal.y * normalFactor - y
    );
}

ros::Vector2D ros::Vector2D::reflectAgainstUnitVector(const Vector2D& normal) const {
    float normalFactor = 2.0f * dot(normal);
    return Vector2D(
        normal.x * normalFactor - x,
        normal.y * normalFactor - y
    );
}
