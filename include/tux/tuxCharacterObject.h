#ifndef TUX_CHARACTER_OBJECT_H
#define TUX_CHARACTER_OBJECT_H

#include "tuxDynamicObject.h"

class tuxCharacterObject : public tuxDynamicObject {
public:
    tuxCharacterObject(
        const btVector3 pos,
        const btScalar  width = 1.75,
        const btScalar  height = 1.75,
        const btScalar  mass = 1.0,
        const btScalar  friction = 0.1
        );

    void turn(const btScalar angle);
    void move(const btScalar speed);
};

#endif //TUX_CHARACTER_OBJECT_H

