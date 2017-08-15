/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Scene.h>

void ros::Scene::setRoot(SceneNodePtr root) {
    this->root = root;
}
