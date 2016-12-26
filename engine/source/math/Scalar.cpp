/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <math/Scalar.h>

float ros::fovToZoom(float fov) {
    float halfFovTan = tanf(fov / 2.0f);
    if (isEqual(halfFovTan, 0.0f)) {
        return 0.0f;
    }
    return 1.0f / halfFovTan;
}

float ros::zoomToFov(float zoom) {
    if (isEqual(zoom, 0.0f)) {
        return 0.0f;
    }
    return 2 * atanf(1.0f / zoom);
}
