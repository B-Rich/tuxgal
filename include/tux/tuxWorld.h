#ifndef TUX_WORLD_H
#define TUX_WORLD_H

#include "tuxStaticObject.h"

class tuxWorld {
public:
    tuxWorld()
        : m_initialized(false),
          m_staticObject(0) { }

    ~tuxWorld();

    bool init();

    btDynamicsWorld* getDynamicsWorld() const { return m_dynamicsWorld; }

    void addObject(tuxObject *object);
    void setStaticObject(tuxStaticObject *object);

    btVector3 getUpDir(const btVector3 pos) const;

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

    tuxStaticObject *m_staticObject;
};

#endif // TUX_WORLD_H
