#include "tuxWorld.h"

bool tuxWorld::init() {
    bool result = false;

    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    btVector3 worldMin(-1000., -1000., -1000.);
    btVector3 worldMax(1000., 1000., 1000.);
    btAxisSweep3* sweepBP = new btAxisSweep3(worldMin, worldMax);
    m_overlappingPairCache = sweepBP;

    m_constraintSolver = new btSequentialImpulseConstraintSolver();
    m_dynamicsWorld = new btDiscreteDynamicsWorld(
        m_dispatcher,
        m_overlappingPairCache,
        m_constraintSolver,
        m_collisionConfiguration
    );
    m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration=0.0001f;
    if (m_dynamicsWorld) {
        m_gravityCenter = btVector3(.0, .0, .0);
        result = true;
    }

    return result;
}

void tuxWorld::applyGravity()
{
    btCollisionObjectArray objects = m_dynamicsWorld->getCollisionObjectArray();
    for (int i = 0; i < objects.size(); i++) {

        btCollisionObject *collisionObject = objects[i];
        btRigidBody *body = btRigidBody::upcast(collisionObject);
        tuxObject *object = (tuxObject *) body->getUserPointer();
        if (object) {
            object->applyGravity(this);
        }
    }
}

