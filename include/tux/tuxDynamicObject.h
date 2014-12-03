#ifndef TUX_DYNAMIC_OBJECT_H
#define TUX_DYNAMIC_OBJECT_H

#include "tuxObject.h"

class tuxDynamicObject : public tuxObject {
public:
    btVector3 getUpDir() const { return m_upDir; }

    virtual void applyGravity(tuxWorld *world);

protected:
    tuxDynamicObject() {
        m_upDir = btVector3(0.0, 1.0, 0.0);
    }

private:
    btVector3 m_upDir;
};

#endif //TUX_DYNAMIC_OBJECT_H

