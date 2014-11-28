#ifndef TUX_PLANET_OBJECT_H
#define TUX_PLANET_OBJECT_H

#include "tuxObject.h"

class tuxPlanetObject : public tuxObject {
public:
    tuxPlanetObject(btVector3 pos, btScalar radius);

    virtual void applyGravity(tuxWorld *world) { }
};

#endif //TUX_PLANET_OBJECT_H

