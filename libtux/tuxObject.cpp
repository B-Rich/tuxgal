#include "tuxObject.h"

void tuxObject::init(btCollisionShape *shape, btRigidBody *body) {
    m_shape = shape;
    m_body = body;
    m_initialized = true;
}

