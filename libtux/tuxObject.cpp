#include "tuxObject.h"

void tuxObject::init(btCollisionShape *shape, btRigidBody *body) {
    m_shape = shape;
    m_body = body;
    m_initialized = true;
}

void tuxObject::attachNode(Ogre::SceneNode *node) {
    m_node = node;
    m_attached = true;
}

void tuxObject::applyTransform() {

    if (m_initialized && m_attached) {
        btRigidBody *body = getBody();
        Ogre::SceneNode *node = m_node;

        if (body && node && body->getMotionState()) {
            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);
            btQuaternion orientation = trans.getRotation();
            node->setPosition(
                Ogre::Vector3(
                    trans.getOrigin().getX(),
                    trans.getOrigin().getY(),
                    trans.getOrigin().getZ()
                    )
                );
            node->setOrientation(
                Ogre::Quaternion(
                    orientation.getW(),
                    orientation.getX(),
                    orientation.getY(),
                    orientation.getZ()
                    )
                );
        }
    }
}

