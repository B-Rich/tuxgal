#include "tuxWorld.h"

void tuxWorld::addObject(tuxObject *object) {
    m_shapes.push_back(object->getShape());
    m_dynamicsWorld->addRigidBody(object->getBody());
}

bool tuxWorld::init() {
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    btVector3 worldMin(-1000.0, -1000.0, -1000.0);
    btVector3 worldMax(1000.0, 1000.0, 1000.0);
    btAxisSweep3* sweepBP = new btAxisSweep3(worldMin, worldMax);
    m_overlappingPairCache = sweepBP;

    m_constraintSolver = new btSequentialImpulseConstraintSolver();
    m_dynamicsWorld = new btDiscreteDynamicsWorld(
        m_dispatcher,
        m_overlappingPairCache,
        m_constraintSolver,
        m_collisionConfiguration
    );
    m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.0001;
    if (m_dynamicsWorld) {
        m_gravityCenter = btVector3(0.0, 0.0, 0.0);
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

            for (int j = 0; j < m_shapes.size(); j++) {
                btCollisionShape* shape = m_shapes[j];
                delete shape;
            }
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

void tuxWorld::applyTransform() {
    btCollisionObjectArray objects = m_dynamicsWorld->getCollisionObjectArray();
    for (int i = 0; i < objects.size(); i++) {

        btCollisionObject *collisionObject = objects[i];
        btRigidBody *body = btRigidBody::upcast(collisionObject);
        tuxObject *object = (tuxObject *) body->getUserPointer();
        if (object) {
            object->applyTransform();
        }
    }
}

