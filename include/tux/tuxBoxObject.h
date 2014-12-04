#ifndef TUX_BOX_OBJECT_H
#define TUX_BOX_OBJECT_H

#include "tuxDynamicObject.h"

class tuxBoxObject : public tuxDynamicObject {
public:
    tuxBoxObject(
        const btVector3 pos,
        const btScalar  width = 1.75,
        const btScalar  height = 1.75,
        const btScalar  depth = 1.75,
        const btScalar  mass = 1.0,
        const btScalar  friction = 0.1
        );
};

#endif //TUX_BOX_OBJECT_H

