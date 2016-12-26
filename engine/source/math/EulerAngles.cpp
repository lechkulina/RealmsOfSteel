/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Matrix3D.h>
#include <math/Matrix4D.h>
#include <math/Quaternion.h>
#include <math/EulerAngles.h>

const ros::EulerAngles ros::EulerAngles::ZERO(0.0f, 0.0f, 0.0f);

ros::EulerAngles::EulerAngles() {
}

ros::EulerAngles::EulerAngles(float heading, float pitch, float bank)
    : heading(heading)
    , pitch(pitch)
    , bank(bank) {
}


ros::EulerAngles::EulerAngles(const Matrix3D& orientation)
    : heading(0.0f)
    , pitch(0.0f)
    , bank(0.0f) {

    // extract angles from object-to-upright rotation matrix
    float pitchSin = -orientation.m23;
    if (isEqual(pitchSin, 1.0f) || isEqual(pitchSin, -1.0f)) {
        // in the case of gimbal lock, rotate only by the heading angle and set bank to zero
        heading = atan2f(-orientation.m31, orientation.m11);
        pitch = HALF_PI * sign(pitchSin);
        bank = 0.0f;
    } else {
        heading = atan2f(orientation.m13, orientation.m33);
        pitch = asinf(pitchSin);
        bank = atan2f(orientation.m21, orientation.m22);
    }
}

ros::EulerAngles::EulerAngles(const Matrix4D& orientation)
    : heading(0.0f)
    , pitch(0.0f)
    , bank(0.0f) {

    // extract angles from object-to-upright rotation matrix
    float pitchSin = -orientation.m23;
    if (isEqual(pitchSin, 1.0f) || isEqual(pitchSin, -1.0f)) {
        // in the case of gimbal lock, rotate only by the heading angle and set bank to zero
        heading = atan2f(-orientation.m31, orientation.m11);
        pitch = HALF_PI * sign(pitchSin);
        bank = 0.0f;
    } else {
        heading = atan2f(orientation.m13, orientation.m33);
        pitch = asinf(pitchSin);
        bank = atan2f(orientation.m21, orientation.m22);
    }
}

ros::EulerAngles::EulerAngles(const Quaternion& orientation)
    : heading(0.0f)
    , pitch(0.0f)
    , bank(0.0f) {

    // extract angles from object-to-upright quaternion
    float pitchSin = -2.0f * (orientation.y * orientation.z - orientation.w * orientation.x);
    if (isEqual(pitchSin, 1.0f) || isEqual(pitchSin, -1.0f)) {
        // in the case of gimbal lock, rotate only by the heading angle and set bank to zero
        float m31 = -orientation.x * orientation.z + orientation.w * orientation.y;
        float m11 = 0.5f - orientation.y * orientation.y - orientation.z * orientation.z;
        heading = atan2f(m31, m11);
        pitch = HALF_PI * sign(pitchSin);
        bank = 0.0f;
    } else {
        float m13 = orientation.x * orientation.z + orientation.w * orientation.y;
        float m33 = 0.5f - orientation.x * orientation.x - orientation.y * orientation.y;
        heading = atan2f(m13, m33);
        pitch = asinf(pitchSin);

        float m21 = orientation.x * orientation.y + orientation.w * orientation.z;
        float m22 = 0.5f - orientation.x * orientation.x - orientation.z * orientation.z;
        bank = atan2f(m21, m22);
    }
}
