/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <Application/LogMessage.h>

std::ostream& ros::operator<<(std::ostream& stream, const LogMessage& message) {
    return stream << message.str() << std::endl;
}
