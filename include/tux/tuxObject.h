#ifndef TUX_OBJECT_H
#define TUX_OBJECT_H

#include "btBulletDynamicsCommon.h"

class tuxWorld;

class tuxObject {
public:
    btRigidBody* getBody() const { return m_body; }
    btCollisionObject* getCollisionObject() const {
        return (btCollisionObject *) m_body;
    }

    virtual void applyGravity(tuxWorld *world) = 0;

protected:
    tuxObject()
        : m_body(0) { }

    void addCollisionShape(btCollisionShape *shape) {
        m_collisionShapes.push_back(shape);
    }

    void setBody(btRigidBody *body) { m_body = body; }

private:
    static btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btRigidBody *m_body;
};

#endif // TUX_OBJECT_H

