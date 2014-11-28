#include <iostream>
#include "tuxWorld.h"
#include "tuxPlanetObject.h"

int main(void)
{
    tuxWorld *world = new tuxWorld;

    if (!world->init()) {
        std::cerr << "Error -- Unable to create world" << std::endl;
        return 1;
    }

    btVector3 gravityCenter(btVector3(0, -20, 0));
    world->setGravityCenter(gravityCenter);

    tuxPlanetObject *planet = new tuxPlanetObject(gravityCenter, 40);
    if (planet) {
        world->addObject(planet);
    }

    return 0;
}

