/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <cstdlib>
#include <Application/SDLApplication.h>

using namespace ros;

int main() {
    SDLApplication app;
    app.init();
    app.run();
    app.quit();

    return EXIT_SUCCESS;
}
