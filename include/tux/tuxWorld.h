#ifndef TUX_WORLD_H
#define TUX_WORLD_H

#include "tuxObject.h"

class tuxWorld {
public:
    tuxWorld()
        : m_initialized(false) { }

    ~tuxWorld();

    bool init();

    btDynamicsWorld* getDynamicsWorld() const { return m_dynamicsWorld; }
    btVector3 getGravityCenter() const { return m_gravityCenter; }

    void setGravityCenter(btVector3 center) { m_gravityCenter = center; }

    void addObject(tuxObject *object);

    void applyGravity();
    void applyTransform();

private:
    bool m_initialized;

    btDefaultCollisionConfiguration *m_collisionConfiguration;
    btCollisionDispatcher *m_dispatcher;
    btBroadphaseInterface *m_overlappingPairCache;
    btConstraintSolver *m_constraintSolver;
    btDynamicsWorld *m_dynamicsWorld;

    btAlignedObjectArray<btCollisionShape*> m_shapes;

    btVector3 m_gravityCenter;
};

#endif // TUX_WORLD_H
