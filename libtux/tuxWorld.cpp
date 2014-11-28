#include "tuxWorld.h"

bool tuxWorld::init() {

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
        m_initialized = true;
    }

    return m_initialized;
}

tuxWorld::~tuxWorld() {

    if (m_initialized) {
        for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1;
             i >= 0;
             i--) {
            btCollisionObject* obj =
                m_dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                delete body->getMotionState();
            }
            m_dynamicsWorld->removeCollisionObject( obj );
            delete obj;
        }

        delete m_dynamicsWorld;
        delete m_constraintSolver;
        delete m_overlappingPairCache;
        delete m_dispatcher;
        delete m_collisionConfiguration;
    }
}

void tuxWorld::applyGravity() {

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
