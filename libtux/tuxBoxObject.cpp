#include "tuxBoxObject.h"

tuxBoxObject::tuxBoxObject(
    const btVector3 pos,
    const btScalar  width,
    const btScalar  height,
    const btScalar  depth,
    const btScalar  mass,
    const btScalar  friction
    ) {
    btCollisionShape* boxShape =
        new btBoxShape(btVector3(width, height, depth));

    //set the initial position and transform. We set tranform none
    btTransform startTransform;
    startTransform.setIdentity();

    btVector3 localInertia(0.0, 0.0, 0.0);

    startTransform.setOrigin(pos);
    boxShape->calculateLocalInertia(mass, localInertia);

    //actually construct the body and add it to the dynamics world
    btDefaultMotionState *boxMotionState =
        new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        mass,
        boxMotionState,
        boxShape,
        localInertia
        );
    rbInfo.m_friction = friction;
    btRigidBody *boxBody = new btRigidBody(rbInfo);
    if (boxBody) {
        boxBody->setAngularFactor(0.0);
        boxBody->setRestitution(0.0);
        boxBody->setUserPointer(this);
        init(boxShape, boxBody);
    }
}

