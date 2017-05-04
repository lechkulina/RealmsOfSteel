/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Texture.h>
#include <graphics/Vertex.h>

ros::Texture::Texture()
    : coordsIndex(0)
    , coordsMapping(TextureCoordsMapping_UV)
    , coordsWrapping(TextureCoordsWrapping_Wrap)
    , blendMode(TextureBlendMode_Multiply)
    , strength(1.0f) {
}

void ros::Texture::setName(const std::string& name) {
    this->name = name;
}

void ros::Texture::setCoordsIndex(U32 coordsIndex) {
    this->coordsIndex = coordsIndex > Vertex::MAX_TEXTURE_COORDS ? Vertex::MAX_TEXTURE_COORDS : coordsIndex;
}

void ros::Texture::setCoordsMapping(TextureCoordsMapping coordsMapping) {
    this->coordsMapping = coordsMapping;
}

void ros::Texture::setCoordsWrapping(TextureCoordsWrapping coordsWrapping) {
    this->coordsWrapping = coordsWrapping;
}

void ros::Texture::setBlendMode(TextureBlendMode blendMode) {
    this->blendMode = blendMode;
}

void ros::Texture::setStrength(float strength) {
    this->strength = strength > 1.0f ? 1.0f : (strength < 0.0f ? 0.0f : strength);
}
