#ifndef TUX_CHARACTER_OBJECT_H
#define TUX_CHARACTER_OBJECT_H

#include "tuxObject.h"

class tuxCharacterObject : public tuxObject {
public:
    tuxCharacterObject(
        btVector3 pos,
        btScalar  width = 1.75,
        btScalar  height = 1.75,
        btScalar  mass = 1.0,
        btScalar  friction = 0.1
        );

    btVector3 getUpDir() const { return m_upDir; }
    btVector3 getForwardDir() const { return m_forwardDir; }

    virtual void applyGravity(tuxWorld *world);

private:
    btVector3 m_upDir;
    btVector3 m_forwardDir;
};

#endif //TUX_CHARACTER_OBJECT_H

