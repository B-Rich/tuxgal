#ifndef TUX_PLANET_OBJECT_H
#define TUX_PLANET_OBJECT_H

#include "tuxStaticObject.h"

class tuxPlanetObject : public tuxStaticObject {
public:
    tuxPlanetObject(const btVector3 pos, const btScalar radius);

    virtual btVector3 getUpDir(const btVector3 pos) const;

private:
    btVector3 m_center;
};

#endif //TUX_PLANET_OBJECT_H

