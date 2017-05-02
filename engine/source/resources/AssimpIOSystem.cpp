/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "AssimpIOSystem.h"
#include "AssimpIOStream.h"

void ros::AssimpIOSystem::Close(Assimp::IOStream* file) {
    AssimpIOStream* const stream = static_cast<AssimpIOStream*>(file);
    stream->Flush();
    stream->setBuffer(RawBufferPtr());
}

bool ros::AssimpIOSystem::Exists(const char* file) const {
    return FileSystem::getInstance()->hasFile(file);
}

char ros::AssimpIOSystem::getOsSeparator() const {
    return FileSystem::getInstance()->getSeparator();
}

Assimp::IOStream* ros::AssimpIOSystem::Open(const char* file, const char*) {
    return new AssimpIOStream(FileSystem::getInstance()->readFile(file));
}
