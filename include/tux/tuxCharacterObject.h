#ifndef TUX_CHARACTER_OBJECT_H
#define TUX_CHARACTER_OBJECT_H

#include "tuxDynamicObject.h"

class tuxCharacterObject : public tuxDynamicObject {
public:
    tuxCharacterObject(
        btVector3 pos,
        btScalar  width = 1.75,
        btScalar  height = 1.75,
        btScalar  mass = 1.0,
        btScalar  friction = 0.1
        );

    void turn(btScalar angle);
    void move(btScalar speed);
};

#endif //TUX_CHARACTER_OBJECT_H

