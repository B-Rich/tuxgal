#ifndef TUX_BOX_OBJECT_H
#define TUX_BOX_OBJECT_H

#include "tuxDynamicObject.h"

class tuxBoxObject : public tuxDynamicObject {
public:
    tuxBoxObject(
        btVector3 pos,
        btScalar  width = 1.75,
        btScalar  height = 1.75,
        btScalar  depth = 1.75,
        btScalar  mass = 1.0,
        btScalar  friction = 0.1
        );
};

#endif //TUX_BOX_OBJECT_H

