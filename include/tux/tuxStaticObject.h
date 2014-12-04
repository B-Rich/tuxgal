#ifndef TUX_STATIC_OBJECT_H
#define TUX_STATIC_OBJECT_H

#include "tuxObject.h"

class tuxStaticObject : public tuxObject {
public:
    virtual btVector3 getUpDir(const btVector3 pos) const = 0;

    virtual void applyGravity(tuxWorld *world) { }

protected:
    tuxStaticObject() { }
};

#endif // TUX_STATIC_OBJET

