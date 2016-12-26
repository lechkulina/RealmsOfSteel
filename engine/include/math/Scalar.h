/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SCALAR_H
#define ROS_SCALAR_H

#include <cmath>

namespace ros {
    const float EPSILON = 0.00001f;
    const float PI = static_cast<float>(M_PI);
    const float HALF_PI = PI / 2.0f;
    const float RADIANS_PER_DEGREE = ros::PI / 180.0f;
    const float DEGREES_PER_RADIAN = 180.0f / ros::PI;
    const float E = static_cast<float>(M_E);

    inline bool isEqual(float left, float right, float error = ros::EPSILON) {
        return fabsf(left - right) < error;
    }

    inline float sign(float value) {
        return value >= 0 ? 1.0f : -1.0f;
    }

    inline float degreesToRadians(float angle) {
        return angle * RADIANS_PER_DEGREE;
    }

    inline float radiansToDegrees(float angle) {
        return angle * DEGREES_PER_RADIAN;
    }

    float fovToZoom(float fov);
    float zoomToFov(float zoom);
}

#endif // ROS_SCALAR_H

