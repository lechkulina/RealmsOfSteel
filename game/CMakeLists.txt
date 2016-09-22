#
# Copyright (c) 2016 Lech Kulina
#
# This file is part of the Realms Of Steel.
# For conditions of distribution and use, see copyright details in the LICENSE file.
#
add_executable(Game
    Source/main.cpp
)
target_link_libraries(Game PUBLIC Engine)
install(TARGETS Game RUNTIME DESTINATION . COMPONENT runtime)
