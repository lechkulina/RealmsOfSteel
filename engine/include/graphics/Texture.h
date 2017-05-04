/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_TEXTURE_H
#define ROS_TEXTURE_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum TextureCoordsMapping {
        TextureCoordsMapping_UV,
        TextureCoordsMapping_Sphere,
        TextureCoordsMapping_Cylinder
    };

    enum TextureCoordsWrapping {
        TextureCoordsWrapping_Wrap,
        TextureCoordsWrapping_Clamp,
        TextureCoordsWrapping_Decal,
        TextureCoordsWrapping_Mirror
    };

    enum TextureBlendMode {
        TextureBlendMode_Multiply,
        TextureBlendMode_Add,
        TextureBlendMode_Subtract,
        TextureBlendMode_Divide,
        TextureBlendMode_SmoothAdd,
        TextureBlendMode_SignedAdd
    };

    class ROS_API Texture {
        public:
            Texture();

            const std::string& getName() const { return name; }
            void setName(const std::string& name);

            U32 getCoordsIndex() const { return coordsIndex; }
            void setCoordsIndex(U32 coordsIndex);

            TextureCoordsMapping getCoordsMapping() const { return coordsMapping; }
            void setCoordsMapping(TextureCoordsMapping coordsMapping);

            TextureCoordsWrapping getCoordsWrapping() const { return coordsWrapping; }
            void setCoordsWrapping(TextureCoordsWrapping coordsWrapping);

            TextureBlendMode getBlendMode() const { return blendMode; }
            void setBlendMode(TextureBlendMode blendMode);

            float getStrength() const { return strength; }
            void setStrength(float strength);

        private:
            std::string name;
            U32 coordsIndex;
            TextureCoordsMapping coordsMapping;
            TextureCoordsWrapping coordsWrapping;
            TextureBlendMode blendMode;
            float strength;
    };

    typedef std::vector<Texture> TexturesStack;
}

#endif // ROS_TEXTURE_H

