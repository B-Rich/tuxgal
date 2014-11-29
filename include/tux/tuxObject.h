#ifndef TUX_OBJECT_H
#define TUX_OBJECT_H

#include "btBulletDynamicsCommon.h"

class tuxWorld;

class tuxObject {
public:
    btCollisionShape* getShape() const { return m_shape; }
    btRigidBody* getBody() const { return m_body; }
    btCollisionObject* getCollisionObject() const {
        return (btCollisionObject *) m_body;
    }

    virtual void applyGravity(tuxWorld *world) = 0;

protected:
    tuxObject()
        : m_initialized(false),
          m_shape(0),
          m_body(0) { }

    void init(btCollisionShape *shape, btRigidBody *body);

private:
    bool m_initialized;
    btCollisionShape *m_shape;
    btRigidBody *m_body;
};

#endif // TUX_OBJECT_H

