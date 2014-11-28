#ifndef TUX_CHARACTER_OBJECT_H
#define TUX_CHARACTER_OBJECT_H

#include "tuxObject.h"

class tuxCharacterObject : public tuxObject {
public:
    tuxCharacterObject(
        btVector3 pos,
        btScalar width = 1.75,
        btScalar height = 1.75,
        btScalar mass = 1.0,
        btScalar friction = 0.1
        );

    virtual void applyGravity(tuxWorld *world);
};

#endif //TUX_CHARACTER_OBJECT_H

