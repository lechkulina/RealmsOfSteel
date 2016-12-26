/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Vector3D.h>
#include <math/Matrix3D.h>
#include <math/Matrix4D.h>
#include <math/EulerAngles.h>
#include <math/Quaternion.h>

const ros::Quaternion ros::Quaternion::ZERO(0.0f, 0.0f, 0.0f, 0.0f);
const ros::Quaternion ros::Quaternion::IDENTITY(1.0f, 0.0f, 0.0f, 0.0f);

ros::Quaternion::Quaternion() {
}

ros::Quaternion::Quaternion(float w, float x, float y, float z)
    : w(w)
    , x(x)
    , y(y)
    , z(z) {
}

ros::Quaternion::Quaternion(const Quaternion& start, const Quaternion& end)
    : w(1.0f)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f) {

    // multiply the right side by the inverted quaternion to find out the angular displacement between them
    Quaternion distance = end * start.inversed();
    w = distance.w;
    x = distance.x;
    y = distance.y;
    z = distance.z;
}

ros::Quaternion::Quaternion(const Vector3D& axis, float angle)
    : w(1.0f)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f) {

    float halfAngle = angle / 2.0f;
    float halfAngleSin = sinf(halfAngle);
    w = cosf(halfAngle);
    x = halfAngleSin * axis.x;
    y = halfAngleSin * axis.y;
    z = halfAngleSin * axis.z;
}

ros::Quaternion::Quaternion(const Matrix3D& orientation)
    : w(1.0f)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f) {

    // use Shoemake strategy to find the largest absolute value of the trace coefficients for quaternion component
    float wCoefficient = orientation.m11 + orientation.m22 + orientation.m33;
    float xCoefficient = orientation.m11 - orientation.m22 - orientation.m33;
    float yCoefficient = orientation.m22 - orientation.m11 - orientation.m33;
    float zCoefficient = orientation.m33 - orientation.m11 - orientation.m22;
    float coefficient = wCoefficient;
    int selector = 0;
    if (xCoefficient > coefficient) {
        coefficient = xCoefficient;
        selector = 1;
    }
    if (yCoefficient > coefficient) {
        coefficient = yCoefficient;
        selector = 2;
    }
    if (zCoefficient > coefficient) {
        coefficient = zCoefficient;
        selector = 3;
    }

    // compute the largest component from the trace and the rest of the components based on the selector
    float value = sqrtf(coefficient + 1.0f) / 2.0f;
    float factor = 0.25f / value;
    switch (selector) {
        case 0:
            // use w component from the trace
            w = value;
            x = (orientation.m32 - orientation.m23) * factor;
            y = (orientation.m13 - orientation.m31) * factor;
            z = (orientation.m21 - orientation.m12) * factor;
            break;
        case 1:
            // use x component from the trace
            w = (orientation.m32 - orientation.m23) * factor;
            x = value;
            y = (orientation.m21 + orientation.m12) * factor;
            z = (orientation.m13 + orientation.m31) * factor;
            break;
        case 2:
            // use y component from the trace
            w = (orientation.m13 - orientation.m31) * factor;
            x = (orientation.m21 + orientation.m12) * factor;
            y = value;
            z = (orientation.m32 + orientation.m23) * factor;
            break;
        case 3:
            // use x component from the trace
            w = (orientation.m21 - orientation.m12) * factor;
            x = (orientation.m13 + orientation.m31) * factor;
            y = (orientation.m32 + orientation.m23) * factor;
            z = value;
            break;
    }
}

ros::Quaternion::Quaternion(const Matrix4D& orientation)
    : w(1.0f)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f) {

    // use Shoemake strategy to find the largest absolute value of the trace coefficients for quaternion component
    float wCoefficient = orientation.m11 + orientation.m22 + orientation.m33;
    float xCoefficient = orientation.m11 - orientation.m22 - orientation.m33;
    float yCoefficient = orientation.m22 - orientation.m11 - orientation.m33;
    float zCoefficient = orientation.m33 - orientation.m11 - orientation.m22;
    float coefficient = wCoefficient;
    int selector = 0;
    if (xCoefficient > coefficient) {
        coefficient = xCoefficient;
        selector = 1;
    }
    if (yCoefficient > coefficient) {
        coefficient = yCoefficient;
        selector = 2;
    }
    if (zCoefficient > coefficient) {
        coefficient = zCoefficient;
        selector = 3;
    }

    // compute the largest component from the trace and the rest of the components based on the selector
    float value = sqrtf(coefficient + 1.0f) / 2.0f;
    float factor = 0.25f / value;
    switch (selector) {
        case 0:
            // use w component from the trace
            w = value;
            x = (orientation.m32 - orientation.m23) * factor;
            y = (orientation.m13 - orientation.m31) * factor;
            z = (orientation.m21 - orientation.m12) * factor;
            break;
        case 1:
            // use x component from the trace
            w = (orientation.m32 - orientation.m23) * factor;
            x = value;
            y = (orientation.m21 + orientation.m12) * factor;
            z = (orientation.m13 + orientation.m31) * factor;
            break;
        case 2:
            // use y component from the trace
            w = (orientation.m13 - orientation.m31) * factor;
            x = (orientation.m21 + orientation.m12) * factor;
            y = value;
            z = (orientation.m32 + orientation.m23) * factor;
            break;
        case 3:
            // use x component from the trace
            w = (orientation.m21 - orientation.m12) * factor;
            x = (orientation.m13 + orientation.m31) * factor;
            y = (orientation.m32 + orientation.m23) * factor;
            z = value;
            break;
    }
}

ros::Quaternion::Quaternion(const EulerAngles& orientation)
    : w(1.0f)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f) {

    // compute the object-to-upright space quaternion
    float halfHeading = orientation.heading / 2.0f;
    float halfHeadingSin = sinf(halfHeading);
    float halfHeadingCos = cosf(halfHeading);
    float halfPitch = orientation.pitch / 2.0f;
    float halfPitchSin = sinf(halfPitch);
    float halfPitchCos = cosf(halfPitch);
    float halfBank = orientation.bank / 2.0f;
    float halfBankSin = sinf(halfBank);
    float halfBankCos = cosf(halfBank);

    // combine all three rotation quaternions that represent the individual Euler rotations
    w = halfHeadingCos * halfPitchCos * halfBankCos + halfHeadingSin * halfPitchSin * halfBankSin;
    x = halfHeadingCos * halfPitchSin * halfBankCos + halfHeadingSin * halfPitchCos * halfBankSin;
    y = halfHeadingSin * halfPitchCos * halfBankCos - halfHeadingCos * halfPitchSin * halfBankSin;
    z = halfHeadingCos * halfPitchCos * halfBankSin - halfHeadingSin * halfPitchSin * halfBankCos;
}

ros::Quaternion ros::Quaternion::lerp(const Quaternion& start, const Quaternion& end, float factor) {
    float complement = 1 - factor;
    return Quaternion(
        start.w * complement + end.w * factor,
        start.x * complement + end.x * factor,
        start.y * complement + end.y * factor,
        start.z * complement + end.z * factor
    );
}

ros::Quaternion ros::Quaternion::slerp(const Quaternion& start, const Quaternion& end, float factor) {
    // make sure to take the shortest 4D arc and compute the cosine of the angle between the quaternions
    Quaternion right(end);
    float angleCos = start.dot(right);
    if (angleCos < 0.0f) {
        right = -end;
        angleCos = -angleCos;
    }

    // use linear interpolation if the two orientations are close to each other
    if (isEqual(angleCos, 1.0f)) {
        return lerp(start, right, factor);
    }

    // compute interpolation coefficients for spherical linear interpolation
    float angleSin = sqrtf(1.0f - angleCos * angleCos);
    float angle = atan2f(angleSin, angleCos);
    float sphericalComplement = sinf((1.0f - factor) * angle) / angleSin;
    float sphericalFactor = sinf(factor * angle) / angleSin;
    return Quaternion(
        start.w * sphericalComplement + right.w * sphericalFactor,
        start.x * sphericalComplement + right.x * sphericalFactor,
        start.y * sphericalComplement + right.y * sphericalFactor,
        start.z * sphericalComplement + right.z * sphericalFactor
    );
}

ros::Quaternion ros::Quaternion::operator-() const {
    return Quaternion(-w, -x, -y, -z);
}

ros::Quaternion ros::Quaternion::operator*(float factor) const {
    return Quaternion(
        w * factor,
        x * factor,
        y * factor,
        z * factor
    );
}

ros::Quaternion& ros::Quaternion::operator*=(float factor) {
    w *= factor;
    x *= factor;
    y *= factor;
    z *= factor;

    return *this;
}

ros::Quaternion ros::Quaternion::operator*(const Quaternion& right) const {
    return Quaternion(
        w * right.w - x * right.x - y * right.y - z * right.z,
        w * right.x + x * right.w + y * right.z - z * right.y,
        w * right.y + y * right.w + z * right.x - x * right.z,
        w * right.z + z * right.w + x * right.y - y * right.x
    );
}

ros::Quaternion& ros::Quaternion::operator*=(const Quaternion& right) {
    Quaternion left(*this);
    w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
    x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
    y = left.w * right.y + left.y * right.w + left.z * right.x - left.x * right.z;
    z = left.w * right.z + left.z * right.w + left.x * right.y - left.y * right.x;

    return *this;
}

float ros::Quaternion::dot(const Quaternion& right) const {
    return w * right.w + x * right.x + y * right.y + z * right.z;
}

float ros::Quaternion::magnitude() const {
    return sqrtf(w * w + x * x + y * y + z * z);
}

ros::Quaternion ros::Quaternion::conjugated() const {
    return Quaternion(w, -x, -y, -z);
}

ros::Quaternion& ros::Quaternion::conjugate() {
    x = -x;
    y = -y;
    z = -z;

    return *this;
}

ros::Quaternion ros::Quaternion::inversed() const {
    // check for the zero quaternion
    float mag = magnitude();
    if (isEqual(mag, 0.0f)) {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    float inversedMag = 1.0f / mag;
    return Quaternion(
         w * inversedMag,
        -x * inversedMag,
        -y * inversedMag,
        -z * inversedMag
    );
}

ros::Quaternion& ros::Quaternion::inverse() {
    // check for the zero quaternion
    float mag = magnitude();
    if (isEqual(mag, 0.0f)) {
        return *this;
    }

    float inversedMag = 1.0f / mag;
    w *= inversedMag;
    x = -x * inversedMag;
    y = -y * inversedMag;
    z = -z * inversedMag;
    return *this;
}

ros::Quaternion ros::Quaternion::exponentiate(float exponent) const {
    if (isEqual(w, 1.0f)) {
        return *this;
    }

    // extract and scale the scalar component
    float alpha = acosf(w);
    float scaledAlpha = alpha * exponent;
    float alphaFactor = sinf(scaledAlpha) / sinf(alpha);
    return Quaternion(
        w * cosf(scaledAlpha),
        x * alphaFactor,
        y * alphaFactor,
        z * alphaFactor
    );
}

ros::Quaternion& ros::Quaternion::exponentiated(float exponent) {
    if (isEqual(w, 1.0f)) {
        return *this;
    }

    // extract and scale the scalar component
    float alpha = acosf(w);
    float scaledAlpha = alpha * exponent;
    float alphaFactor = sinf(scaledAlpha) / sinf(alpha);
    w *= cosf(scaledAlpha);
    x *= alphaFactor;
    y *= alphaFactor;
    z *= alphaFactor;
    return *this;
}
